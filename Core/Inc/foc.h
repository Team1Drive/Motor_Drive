#pragma once
/*
 * foc.h  —  Field-Oriented Control for Motor_Drive (STM32H725)
 *
 * Designed to integrate cleanly with the real Motor_Drive repo.
 * All motor parameters and gains are concentrated here.
 *
 * Control hierarchy (runs in TIM8 update ISR at 20 kHz = FOC_TS):
 *
 *   [TIM4 encoder] → getPos_rad(), getRPM()  (RPM updated by TIM6 at 1 kHz)
 *   [ADC1/2/3 DMA] → Ia, Ib, Ic, Vdc
 *        │
 *   Clarke transform     (Ia,Ib,Ic → Iα,Iβ)
 *        │
 *   Park   transform     (Iα,Iβ,θe → Id,Iq)
 *        │
 *   ┌────┴──────────────┐
 *   Speed PI (outer,     Field-weakening PI
 *   decimated)           (negative Id when |u| > Vdc/√3)
 *   ω_ref → Iq_ref       → Id_ref
 *        │
 *   Current PI × 2 (inner, with decoupling feed-forward)
 *   Id_ref − Id → Vd_cmd = PI_d + R·Id − ωe·L·Iq
 *   Iq_ref − Iq → Vq_cmd = PI_q + R·Iq + ωe·L·Id + ωe·ψf
 *        │
 *   Inverse Park          (Vd,Vq,θe → Vα,Vβ)
 *        │
 *   SVPWM_COMP            (Vα,Vβ,Vdc → dA,dB,dC)
 *        │
 *   motorPWM.setDuty()    → TIM8 CCR1/2/3
 *
 * TUNING CHECKLIST (set defines below before first run):
 *   1. Motor params correct  — BLY172S-24V-4000 values pre-loaded
 *   2. FOC_KP_I / FOC_KI_I  — current PI, start very low
 *   3. FOC_KP_SP / FOC_KI_SP — speed PI, start very low
 *   4. Run encoder alignment before enabling FOC
 *   5. Check current offsets print correctly via 'status' command
 */

#include "stm32h7xx_hal.h"
#include "parameters.h"   /* MOTOR_POLE_PAIRS, ADCGain_t, M_PI */
#include <cmath>
#include <algorithm>

/* =========================================================================
 * MOTOR PARAMETERS — BLY172S-24V-4000 (Anaheim Automation)
 * These match the MATLAB reference simulation exactly.
 * R = 1.5 * R_LL,  L = 1.5 * L_LL  (line-to-line → per-phase star model)
 * ========================================================================= */

/** Per-phase stator resistance (Ω).  1.5 × 0.80 Ω (datasheet) */
#define FOC_R               1.20f

/** Per-phase stator inductance (H).  1.5 × 1.20e-3 H (datasheet) */
#define FOC_L               1.80e-3f

/**
 * PM flux linkage (Wb).
 * Kt_SI = 5.81 oz-in/A × 0.007062 = 0.04103 Nm/A
 * psi_f = Kt_SI / (1.5 × pole_pairs) = 0.04103 / (1.5 × 4) = 0.006838 Wb
 *
 * Note: MOTOR_POLE_PAIRS = 4 is defined in parameters.h
 */
#define FOC_PSI_F           0.006838f

/* =========================================================================
 * CURRENT & VOLTAGE LIMITS
 * ========================================================================= */

/** Peak phase current limit (A). Fault trips if any phase exceeds this. */
#define FOC_IMAX            3.5f

/** Most negative Id allowed during field weakening (A). */
#define FOC_ID_FW_MIN      -3.5f

/* =========================================================================
 * TIMING
 * ========================================================================= */

/** FOC / PWM period (s). Must match motorPWM.setFrequency(). */
#define FOC_TS              (1.0f / 20000.0f)

/**
 * Speed PI decimation.
 * Speed PI runs every FOC_SPEED_DIV FOC ticks.
 * 20 → runs at 1 kHz (matches TIM6 speed-update rate).
 */
#define FOC_SPEED_DIV       20U

/** Speed ramp rate (mechanical rad/s²). From MATLAB: 5000 RPM/s */
#define FOC_RAMP_RATE       (1000.0f * 2.0f * M_PI / 60.0f)

/* =========================================================================
 * PI GAINS
 * Start conservatively on real hardware. Increase cautiously.
 * MATLAB reference used: Kp_i=15, Ki_i=3000, Kp_sp=0.15, Ki_sp=3
 * ========================================================================= */

/** Current PI proportional gain. Start: 0.5, MATLAB ref: 15 */
#define FOC_KP_I            0.5f

/** Current PI integral gain. Start: 50, MATLAB ref: 3000 */
#define FOC_KI_I            50.0f

/** Current PI integrator clamp (V). Symmetric ±clamp. */
#define FOC_INT_I_CLAMP     6.0f

/** Speed PI proportional gain. Start: 0.05, MATLAB ref: 0.15 */
#define FOC_KP_SP           0.05f

/** Speed PI integral gain. Start: 1.0, MATLAB ref: 3.0 */
#define FOC_KI_SP           1.0f

/** Field-weakening PI proportional gain (typically 0 — only integral matters) */
#define FOC_KP_FW           0.0f

/** Field-weakening PI integral gain. From MATLAB ref: 500 */
#define FOC_KI_FW           500.0f

/* =========================================================================
 * PI CONTROLLER — inline struct and update function (ISR-safe, no malloc)
 * ========================================================================= */

typedef struct {
    float kp;
    float ki;
    float integrator;
    float clamp;    /* symmetric ±clamp applied to both integrator and output */
} PI_t;

/**
 * @brief Run one PI step. Returns clamped output.
 *        Anti-windup: integrator is clamped before output sum.
 */
static inline float PI_update(PI_t* pi, float error, float dt)
{
    pi->integrator += pi->ki * error * dt;
    if (pi->integrator >  pi->clamp) pi->integrator =  pi->clamp;
    if (pi->integrator < -pi->clamp) pi->integrator = -pi->clamp;
    float out = pi->kp * error + pi->integrator;
    if (out >  pi->clamp) out =  pi->clamp;
    if (out < -pi->clamp) out = -pi->clamp;
    return out;
}

/** Reset integrator to zero. Call on mode switch or fault. */
static inline void PI_reset(PI_t* pi) { pi->integrator = 0.0f; }

/* =========================================================================
 * FOC STATE — one global instance declared in main.cpp
 * ========================================================================= */

typedef struct {

    /* --- PI controllers --- */
    PI_t pi_d;
    PI_t pi_q;
    PI_t pi_speed;
    PI_t pi_fw;

    /* --- Setpoints --- */
    float target_rpm;   /* commanded speed (RPM), set via USB command    */
    float omega_ref;    /* ramped speed reference (mechanical rad/s)     */
    float Id_ref;       /* d-axis current reference (A)                  */
    float Iq_ref;       /* q-axis current reference (A)                  */

    /* --- System parameters --- */
    float ts;           /* FOC / PWM period (s). Must match motorPWM.setFrequency() */

    /* --- Observables — written each tick, read by main loop / telemetry --- */
    volatile float Id;
    volatile float Iq;
    volatile float Ia;
    volatile float Ib;
    volatile float Ic;
    volatile float Vdc;
    volatile float theta_e;
    volatile float omega_e;
    volatile float rpm;
    volatile float Vd_cmd;
    volatile float Vq_cmd;
    volatile float u_mag;

    /* --- Speed decimation counter --- */
    uint32_t speed_div_cnt;

    /* --- Fault flag — set on overcurrent, cleared by foc_reset() --- */
    volatile bool fault;

} FOC_State_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise all PI controllers and zero all state.
 *        Call once in main() after hardware init.
 */
void foc_init(FOC_State_t* foc);

/**
 * @brief Run one complete FOC tick.
 *        Called from HAL_TIM_PeriodElapsedCallback when TIM8 fires (20 kHz).
 *
 * @param foc      FOC state.
 * @param Ia       Phase A current (A), already offset-corrected.
 * @param Ib       Phase B current (A), already offset-corrected.
 * @param Ic       Phase C current (A), already offset-corrected.
 * @param Vdc      DC bus voltage (V).
 * @param theta_e  Electrical angle (rad, 0…2π).
 * @param omega_m  Mechanical angular velocity (rad/s).
 * @param[out] dutyA  Duty cycle for phase A → motorPWM.setDuty().
 * @param[out] dutyB  Duty cycle for phase B.
 * @param[out] dutyC  Duty cycle for phase C.
 */
void foc_run(FOC_State_t* foc,
             float Ia, float Ib, float Ic,
             float Vdc,
             float theta_e, float omega_m,
             float* dutyA, float* dutyB, float* dutyC);

/**
 * @brief Reset all PI integrators and clear fault flag.
 *        Call on MOTOR_STOP or when re-arming after a fault.
 */
void foc_reset(FOC_State_t* foc);

/**
 * @brief Apply a voltage vector along the d-axis to align the encoder zero.
 *        Call before enabling FOC to ensure correct angle tracking.
 */
void foc_align_zero(FOC_State_t* foc, float Vmag, float Vdc, float* dutyA, float* dutyB, float* dutyC);
