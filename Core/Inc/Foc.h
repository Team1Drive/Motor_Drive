#pragma once
/*
 * foc.h  —  Field-Oriented Control for Motor_Drive (STM32H725)
 *
 * Architecture mirrors the MATLAB reference simulation:
 *   foc_svpwm_switched_inverter_multimode.m
 *
 * Control hierarchy (all running in the TIM8 update ISR at Ts = 50 µs):
 *
 *   [Encoder] ──► theta_e, omega_m
 *   [ADC]     ──► Ia, Ib, Ic, Vdc
 *                     │
 *                 Clarke transform   (Ia,Ib,Ic  → Iα,Iβ)
 *                     │
 *                 Park   transform   (Iα,Iβ,θe  → Id,Iq)   ← feedback
 *                     │
 *            ┌────────┴──────────┐
 *      Speed PI (outer)      Field-weakening PI
 *      omega_ref → Iq_ref    Vdc/√3 − |u| → Id_ref (neg)
 *            │
 *      Current PI × 2 (inner, with decoupling feed-forward)
 *      Id_ref − Id → Vd_cmd
 *      Iq_ref − Iq → Vq_cmd
 *            │
 *      Inverse Park           (Vd,Vq,θe   → Vα,Vβ)
 *            │
 *      SVPWM_COMP             (Vα,Vβ,Vdc  → dA,dB,dC)
 *            │
 *      motorPWM.setDuty()     → TIM8 CCR1/2/3
 *
 * Motor assumed: surface-mount PMSM (Ld = Lq = L).
 *
 * TUNING CHECKLIST (set values in foc.h):
 *   1. FOC_POLE_PAIRS         — from motor datasheet (P/2)
 *   2. FOC_R, FOC_L           — per-phase resistance & inductance
 *   3. FOC_PSI_F              — PM flux linkage (= Kt / (1.5 * pp))
 *   4. FOC_IMAX               — peak current limit (A)
 *   5. FOC_KP_I / FOC_KI_I   — current PI (start low: Kp=0.5, Ki=50)
 *   6. FOC_KP_SP / FOC_KI_SP — speed  PI (start low: Kp=0.05, Ki=1)
 *   7. FOC_KI_FW              — field-weakening integral gain
 *   8. FOC_CURRENT_OFFSET_*   — calibrated at startup with motor at rest
 */

#include "stm32h7xx_hal.h"
#include "parameters.h"   /* M_PI, SQRT3, ENCODER_PPR */
#include <cmath>
#include <algorithm>

/* =========================================================================
 * MOTOR PARAMETERS  — set these for your specific motor
 * ========================================================================= */

/** Number of pole pairs (P/2). For an 8-pole motor use 4. */
#define FOC_POLE_PAIRS      4U

/**
 * Per-phase stator resistance (Ohm).
 * From MATLAB ref: R = 1.5 * R_LL  = 1.5 * 0.80 = 1.20 Ω
 * Adjust for your motor; measure at room temperature.
 */
#define FOC_R               1.20f

/**
 * Per-phase stator inductance (H).
 * From MATLAB ref: L = 1.5 * L_LL  = 1.5 * 1.20e-3 = 1.80e-3 H
 */
#define FOC_L               1.80e-3f

/**
 * PM flux linkage (Wb).  Derived from torque constant:
 *   Kt_SI = Kt_ozin * 0.007062  (Nm/A)
 *   psi_f = Kt_SI / (1.5 * pole_pairs)
 * For BLY172S-24V: Kt = 5.81 oz-in/A → 0.04103 Nm/A → psi_f ≈ 0.003419 Wb
 * Set to 0.0f if unknown; the FOC will still work but torque constant is wrong.
 */
#define FOC_PSI_F           0.003419f

/* =========================================================================
 * CONTROLLER LIMITS
 * ========================================================================= */

/** Maximum phase current magnitude (A). Keep below hardware overcurrent trip. */
#define FOC_IMAX            3.5f

/** Most negative Id allowed during field weakening (A). */
#define FOC_ID_FW_MIN      -3.5f

/** Voltage limit for field weakening: Vdc/sqrt(3). Computed at runtime. */
/* #define FOC_U_MAX_FW  computed as Vdc/sqrtf(3.0f) in foc_run() */

/* =========================================================================
 * CONTROL LOOP TIMING
 * ========================================================================= */

/** PWM / FOC period (s). Must match motorPWM.setFrequency(). */
#define FOC_TS              (1.0f / 20000.0f)

/**
 * Speed decimation factor.
 * Speed PI runs every FOC_SPEED_DIV FOC ticks = 20000/FOC_SPEED_DIV Hz.
 * Set to 1 to run at full 20 kHz (aggressive), 20 for 1 kHz (typical).
 */
#define FOC_SPEED_DIV       20U

/** Speed ramp rate (electrical rad/s per second = rad/s²). */
#define FOC_RAMP_RATE       (5000.0f * 2.0f * M_PI / 60.0f)

/* =========================================================================
 * CURRENT PI GAINS  (inner loop)
 * Starting point from MATLAB: Kp=15, Ki=3000.
 * On real hardware start with Kp=0.5, Ki=50 and ramp up cautiously.
 * ========================================================================= */
#define FOC_KP_I            0.5f
#define FOC_KI_I            50.0f

/** Current PI integrator clamp (V). Prevents windup. */
#define FOC_INT_I_CLAMP     (0.1f)

/* =========================================================================
 * SPEED PI GAINS  (outer loop)
 * Starting point from MATLAB: Kp=0.15, Ki=3.
 * ========================================================================= */
#define FOC_KP_SP           0.05f
#define FOC_KI_SP           1.0f

/* =========================================================================
 * FIELD-WEAKENING PI GAINS
 * Set Kp_fw=0 initially; only Ki_fw matters for steady-state FW.
 * ========================================================================= */
#define FOC_KP_FW           0.0f
#define FOC_KI_FW           500.0f

/* =========================================================================
 * ADC CURRENT OFFSETS  (calibrated at rest — motor not spinning)
 * adcToCurrent() assumes zero-current maps to Vref/2.
 * If there is a DC offset, measure the raw ADC value with motor stopped
 * and set these to the resulting zero-current ampere value.
 * ========================================================================= */
#define FOC_CURRENT_OFFSET_A    0.0f   /* (A) */
#define FOC_CURRENT_OFFSET_B    0.0f   /* (A) */
#define FOC_CURRENT_OFFSET_C    0.0f   /* (A) */

/* =========================================================================
 * PI CONTROLLER — generic struct and update function (inline, ISR-safe)
 * ========================================================================= */
typedef struct {
    float kp;
    float ki;
    float integrator;
    float clamp;      /* symmetric ±clamp on integrator AND output */
} PI_t;

/**
 * @brief Update a PI controller and return the clamped output.
 * Anti-windup: integrator is clamped independently from output.
 * Both clamps are symmetric: [-clamp, +clamp].
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

/** Reset a PI controller to zero. Call when switching modes or on stop. */
static inline void PI_reset(PI_t* pi)
{
    pi->integrator = 0.0f;
}

/* =========================================================================
 * FOC STATE  — one instance lives in main.cpp as a global
 * ========================================================================= */
typedef struct {

    /* --- PI controllers --- */
    PI_t pi_d;      /* d-axis current PI  */
    PI_t pi_q;      /* q-axis current PI  */
    PI_t pi_speed;  /* speed PI (outer)   */
    PI_t pi_fw;     /* field-weakening PI */

    /* --- Setpoints --- */
    float target_rpm;       /* commanded speed (RPM) */
    float omega_ref;        /* ramped speed reference (electrical rad/s) */
    float Id_ref;           /* d-axis current reference (A) */
    float Iq_ref;           /* q-axis current reference (A) */

    /* --- Observables (written each tick, read by telemetry loop) --- */
    volatile float Id;          /* measured d-axis current (A) */
    volatile float Iq;          /* measured q-axis current (A) */
    volatile float Ia;          /* phase A current (A) */
    volatile float Ib;          /* phase B current (A) */
    volatile float Ic;          /* phase C current (A) */
    volatile float Vdc;         /* measured DC bus voltage (V) */
    volatile float theta_e;     /* electrical angle (rad, 0‥2π) */
    volatile float omega_e;     /* electrical angular velocity (rad/s) */
    volatile float rpm;         /* mechanical RPM */
    volatile float Vd_cmd;      /* d-axis voltage command (V) */
    volatile float Vq_cmd;      /* q-axis voltage command (V) */
    volatile float u_mag;       /* |Vd,Vq| voltage vector magnitude (V) */

    /* --- Speed decimation counter --- */
    uint32_t speed_div_cnt;

    /* --- Fault flag --- */
    volatile bool fault;        /* set on overcurrent or invalid angle */

} FOC_State_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialise all PI controllers and reset state.
 *        Call once after hardware init, before enabling MOTOR_FOC_LINEAR.
 */
void foc_init(FOC_State_t* foc);

/**
 * @brief Run one complete FOC tick (Clarke → Park → PI → inv_Park → SVPWM).
 *        Called from HAL_TIM_PeriodElapsedCallback when TIM8 fires.
 *
 * @param foc      Pointer to the FOC state struct.
 * @param Ia       Phase A current, ADC-converted (A).
 * @param Ib       Phase B current, ADC-converted (A).
 * @param Ic       Phase C current, ADC-converted (A).
 * @param Vdc      DC bus voltage (V).
 * @param theta_e  Electrical angle from encoder (rad, 0‥2π).
 * @param omega_m  Mechanical angular velocity (rad/s), from encoder RPM.
 * @param[out] dutyA  Phase A duty cycle to pass to motorPWM.setDuty().
 * @param[out] dutyB  Phase B duty cycle.
 * @param[out] dutyC  Phase C duty cycle.
 */
void foc_run(FOC_State_t* foc,
             float Ia, float Ib, float Ic,
             float Vdc,
             float theta_e, float omega_m,
             float* dutyA, float* dutyB, float* dutyC);

/**
 * @brief Reset all integrators. Call on MOTOR_STOP or fault.
 */
void foc_reset(FOC_State_t* foc);