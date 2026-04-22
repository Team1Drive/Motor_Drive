/*
 * foc.cpp  —  Field-Oriented Control implementation
 *
 * Mirrors the MATLAB reference: foc_svpwm_switched_inverter_multimode.m
 *
 * Per-tick execution order:
 *   0. Overcurrent check         — fault + zero duty if tripped
 *   1. Clarke transform          — Ia,Ib,Ic → Iα,Iβ
 *   2. Park   transform          — Iα,Iβ,θe → Id,Iq
 *   3. Speed ramp                — advance omega_ref toward target
 *   4. Speed PI (decimated)      — omega_ref − omega_m → Iq_ref
 *   5. Field-weakening PI        — Vdc/√3 − |u_prev| → Id_ref (negative)
 *   6. Current magnitude limit   — clamp Iq_ref to √(Imax²−Id_ref²)
 *   7. Current PI + decoupling   — Id_ref−Id → Vd_cmd
 *                                  Iq_ref−Iq → Vq_cmd
 *   8. Inverse Park              — Vd,Vq,θe → Vα,Vβ
 *   9. SVPWM_COMP                — Vα,Vβ,Vdc → dA,dB,dC
 */

#include "foc.h"
#include "modulation.h"   /* clarke, park, inv_park, modulate, ModulationType */
#include "cordic_math.h" /* cordic::sinf, cordic::atan2f, etc. */
#include <cmath>
#include <algorithm>

/* =========================================================================
 * foc_init
 * ========================================================================= */
void foc_init(FOC_State_t* foc)
{
    /* Current PI — inner loop */
    foc->pi_d.kp            = FOC_KP_I;
    foc->pi_d.ki            = FOC_KI_I;
    foc->pi_d.integrator    = 0.0f;
    foc->pi_d.clamp_upper   = FOC_INT_I_CLAMP;
    foc->pi_d.clamp_lower   = -FOC_INT_I_CLAMP;
    foc->pi_q = foc->pi_d;   /* same gains for both axes */

    /* Speed PI — outer loop. Output clamp = FOC_IMAX (A). */
    foc->pi_speed.kp            = FOC_KP_SP;
    foc->pi_speed.ki            = FOC_KI_SP;
    foc->pi_speed.integrator    = 0.0f;
    foc->pi_speed.clamp_upper   = FOC_I_CLAMP_UPPER_SP;
    foc->pi_speed.clamp_lower   = FOC_I_CLAMP_LOWER_SP;

    /* Field-weakening PI — outer loop. Output clamp = FOC_IMAX (A). */
    foc->pi_fw.kp           = FOC_KP_FW;
    foc->pi_fw.ki           = FOC_KI_FW;
    foc->pi_fw.integrator   = 0.0f;
    foc->pi_fw.clamp_upper  = FOC_I_CLAMP_UPPER_FW;
    foc->pi_fw.clamp_lower  = FOC_I_CLAMP_LOWER_FW;

    /* Setpoints */
    foc->target_rpm  = 0.0f;
    foc->omega_ref   = 0.0f;
    foc->Id_ref      = 0.0f;
    foc->Iq_ref      = 0.0f;

    /* Observables */
    foc->Id      = 0.0f;
    foc->Iq      = 0.0f;
    foc->Ia      = 0.0f;
    foc->Ib      = 0.0f;
    foc->Ic      = 0.0f;
    foc->Vdc     = 0.0f;
    foc->theta_e = 0.0f;
    foc->omega_e = 0.0f;
    foc->rpm     = 0.0f;
    foc->Vd_cmd  = 0.0f;
    foc->Vq_cmd  = 0.0f;
    foc->u_mag   = 0.0f;

    foc->speed_div_cnt = 0U;
    foc->fault         = false;
}

/* =========================================================================
 * foc_reset
 * ========================================================================= */
void foc_reset(FOC_State_t* foc)
{
    PI_reset(&foc->pi_d);
    PI_reset(&foc->pi_q);
    PI_reset(&foc->pi_speed);
    PI_reset(&foc->pi_fw);
    foc->omega_ref = 0.0f;
    foc->Id_ref    = 0.0f;
    foc->Iq_ref    = 0.0f;
    foc->Id        = 0.0f;
    foc->Iq        = 0.0f;
    foc->Vd_cmd    = 0.0f;
    foc->Vq_cmd    = 0.0f;
    foc->u_mag     = 0.0f;
    foc->fault     = false;
}

void focResetPI(FOC_State_t* foc) {
    PI_reset(&foc->pi_d);
    PI_reset(&foc->pi_q);
    PI_reset(&foc->pi_speed);
    PI_reset(&foc->pi_fw);
}

/* =========================================================================
 * foc_run  — called from TIM8 update ISR at 20 kHz
 * ========================================================================= */
void foc_run(FOC_State_t* foc,
             float Ia, float Ib, float Ic,
             float Vdc,
             float theta_e, float omega_m,
             float* dutyA, float* dutyB, float* dutyC)
             {
    /* ------------------------------------------------------------------
     * 0. Store observables
     * ------------------------------------------------------------------ */
    const float omega_e  = omega_m * (float)MOTOR_POLE_PAIRS;
    const float rpm_meas = omega_m * (60.0f / (2.0f * M_PI));

    foc->Ia      = Ia;
    foc->Ib      = Ib;
    foc->Ic      = Ic;
    foc->Vdc     = Vdc;
    foc->theta_e = theta_e;
    foc->omega_e = omega_e;
    foc->rpm     = rpm_meas;

    

    /* ------------------------------------------------------------------
     * 1. Overcurrent protection
     *    Compare against Imax². All three phases checked.
     *    On trip: output 0.5 duty (mid-rail, no net current), latch fault.
     * ------------------------------------------------------------------ */
    const float Imax_sq = FOC_IMAX * FOC_IMAX;
    if (foc->Ia*foc->Ia > Imax_sq || foc->Ib*foc->Ib > Imax_sq || foc->Ic*foc->Ic > Imax_sq) {
        foc->fault = true;
        *dutyA = 0.5f;
        *dutyB = 0.5f;
        *dutyC = 0.5f;
        return;
    }

    /* ------------------------------------------------------------------
     * 2. Clarke transform: Ia,Ib,Ic → Iα,Iβ  (stationary frame)
     *
     *   Iα = (2·Ia − Ib − Ic) / 3
     *   Iβ = (Ib − Ic) / √3
     *
     * Uses clarke() from modulation.h (same formula).
     * ------------------------------------------------------------------ */
    float I_alpha, I_beta;
    clarke(Ia, Ib, Ic, &I_alpha, &I_beta);

    /* ------------------------------------------------------------------
     * 3. Park transform: Iα,Iβ,θe → Id,Iq  (rotating dq frame)
     *
     *   Id =  Iα·cos(θe) + Iβ·sin(θe)
     *   Iq = −Iα·sin(θe) + Iβ·cos(θe)
     * ------------------------------------------------------------------ */
    float Id, Iq;
    park(I_alpha, I_beta, theta_e, &Id, &Iq);

    foc->Id = Id;
    foc->Iq = Iq;

    /* ------------------------------------------------------------------
     * 4. Speed ramp: advance omega_ref toward target each tick
     * ------------------------------------------------------------------ */
    const float omega_target = foc->target_rpm * (2.0f * M_PI / 60.0f);
    const float max_step     = FOC_RAMP_RATE * RPM_TO_RAD_S * FOC_TS;
    float delta = omega_target - foc->omega_ref;
    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;
    foc->omega_ref += delta;

    /* ------------------------------------------------------------------
     * 5. Speed PI (outer, FOC_SPEED_DIV ticks = 1 kHz)
     *
     *   err_sp  = omega_ref − omega_m    (mechanical rad/s)
     *   Iq_ref  = Kp_sp · err_sp + Ki_sp · ∫err_sp
     *
     * Anti-windup: PI_update clamps integrator independently.
     * ------------------------------------------------------------------ */
    foc->speed_div_cnt++;
    if (foc->speed_div_cnt >= FOC_SPEED_DIV) {
        foc->speed_div_cnt = 0U;
        const float err_sp = foc->omega_ref - omega_m;
        foc->Iq_ref = PI_update(&foc->pi_speed, err_sp,
                                FOC_TS * (float)FOC_SPEED_DIV);
    }

    /* ------------------------------------------------------------------
     * 6. Field-weakening PI — DISABLED
     *    Id_ref is held at zero (no field weakening).
     *
     * Original field-weakening logic commented out below for reference:
     *
     * const float U_max_fw   = Vdc / 1.73205f;
     * const float u_mag_prev = foc->u_mag;
     * if (omega_e > 10.0f) {
     *     float fw_error = (U_max_fw - u_mag_prev) / omega_e;
     *     float id_fw    = PI_update(&foc->pi_fw, fw_error, FOC_TS);
     *     if (id_fw >  0.0f)          id_fw = 0.0f;
     *     if (id_fw <  FOC_ID_FW_MIN) id_fw = FOC_ID_FW_MIN;
     *     foc->Id_ref = id_fw;
     * } else {
     *     foc->Id_ref = 0.0f;
     *     PI_reset(&foc->pi_fw);
     * }
     * ------------------------------------------------------------------ */
    foc->Id_ref = 0.0f;   /* Id = 0: maximum torque per amp, no flux weakening */

    /* ------------------------------------------------------------------
     * 7. Current magnitude limit on Iq_ref
     *
     *   |I_total|² = Id_ref² + Iq_ref² ≤ Imax²
     *   → Iq_max = √(Imax² − Id_ref²)
     * ------------------------------------------------------------------ */
    float Iq_max = sqrtf(std::max(FOC_IMAX * FOC_IMAX
                                  - foc->Id_ref * foc->Id_ref, 0.0f));
    if (foc->Iq_ref >  Iq_max) foc->Iq_ref =  Iq_max;
    if (foc->Iq_ref < -Iq_max) foc->Iq_ref = -Iq_max;

    /* ------------------------------------------------------------------
     * 8. Current PI controllers with decoupling feed-forward
     *
     * Mirrors MATLAB exactly:
     *   vd_cmd = Kp·err_id + Ki·∫err_id + R·Id − ωe·L·Iq
     *   vq_cmd = Kp·err_iq + Ki·∫err_iq + R·Iq + ωe·L·Id + ωe·ψf
     *
     * The feed-forward terms cancel cross-coupling and back-EMF so the
     * PI only needs to correct residual error.
     * Output hard-clamped to ±Vdc/2 (inverter rail limit).
     * ------------------------------------------------------------------ */
    const float err_id = foc->Id_ref - Id;
    const float err_iq = foc->Iq_ref - Iq;

    float Vd_pi = PI_update(&foc->pi_d, err_id, FOC_TS);
    float Vq_pi = PI_update(&foc->pi_q, err_iq, FOC_TS);

    float Vd_cmd = Vd_pi + FOC_R * Id  - omega_e * FOC_L * Iq;
    float Vq_cmd = Vq_pi + FOC_R * Iq  + omega_e * FOC_L * Id
                         + omega_e * FOC_PSI_F;

    /* Hard clamp to inverter voltage limits */
    const float Vdc_half = Vdc * 0.5f;
    if (Vd_cmd >  Vdc_half) Vd_cmd =  Vdc_half;
    if (Vd_cmd < -Vdc_half) Vd_cmd = -Vdc_half;
    if (Vq_cmd >  Vdc_half) Vq_cmd =  Vdc_half;
    if (Vq_cmd < -Vdc_half) Vq_cmd = -Vdc_half;

    /* Store for telemetry and FW feedback next tick */
    foc->Vd_cmd = Vd_cmd;
    foc->Vq_cmd = Vq_cmd;
    foc->u_mag  = sqrtf(Vd_cmd * Vd_cmd + Vq_cmd * Vq_cmd);

    /* ------------------------------------------------------------------
     * 9. Inverse Park: Vd,Vq,θe → Vα,Vβ
     *
     *   Vα =  Vd·cos(θe) − Vq·sin(θe)
     *   Vβ =  Vd·sin(θe) + Vq·cos(θe)
     * ------------------------------------------------------------------ */
    float V_alpha, V_beta;
    inv_park(Vd_cmd, Vq_cmd, theta_e, &V_alpha, &V_beta);

    /* ------------------------------------------------------------------
     * 10. SVPWM_COMP modulation: Vα,Vβ → dA,dB,dC
     *     Matches 'svpwm_comp' mode in the MATLAB reference.
     *     Ts passed for compensated timing calculation.
     * ------------------------------------------------------------------ */
    modulate(ModulationType::SVPWM,
             V_alpha, V_beta,
             Vdc,
             foc->ts,
             dutyA, dutyB, dutyC);
}

void focAlignZero(FOC_State_t* foc, float Vmag, float Vdc, float* dutyA, float* dutyB, float* dutyC) {
    /* Apply voltage vector along alpha axis (d-axis) to align encoder zero. */
    modulate(ModulationType::SVPWM,
             Vmag, 0.0f,
             Vdc,
             foc->ts,
             dutyA, dutyB, dutyC);
}

void focTest(FOC_State_t* foc,
             float ia, float ib, float ic,
             float vdc,
             float theta_e, float omega_m,
             float* dutyA, float* dutyB, float* dutyC) {

    foc->Ia = ia;
    foc->Ib = ib;
    foc->Ic = ic;
    foc->Vdc = vdc;
    foc->theta_e = theta_e;
    foc->omega_m = omega_m;
    foc->omega_e  = omega_m * (float)MOTOR_POLE_PAIRS;

    // Current Clarke Transform
    float i_alpha, i_beta;
    clarke(ia, ib, ic, &i_alpha, &i_beta);

    // Current Park Transform
    float id, iq;
    park(i_alpha, i_beta, theta_e, &id, &iq);

    foc->Id = id;
    foc->Iq = iq;

    if (system_flag & FLAG_AUDIBLE) {
      focInjection(foc, 500.0f);
      //if (foc->rpm < 800.0f) focInjection(foc, 500.0f);
      //else if (foc->rpm < 1200.0f) focInjection(foc, 1000.0f);
    }
    
    // Calculate PI outputs
    float err_id = foc->Id_ref - id;
    float err_iq = foc->Iq_ref - iq;
    
    float vd_pi = PI_update(&foc->pi_d, err_id, foc->ts);
    float vq_pi = PI_update(&foc->pi_q, err_iq, foc->ts);

    // Calculate voltage commands with decoupling feed-forward
    foc->Vd_cmd = vd_pi - foc->omega_e * FOC_L * iq;
    foc->Vq_cmd = vq_pi + foc->omega_e * FOC_L * id + foc->omega_e * FOC_PSI_F;
    foc->u_mag  = cordic::hypotf(foc->Vd_cmd, foc->Vq_cmd);

    float v_max = vdc / SQRT3;  // Maximum voltage magnitude for SVPWM (line-line voltage limit)
    if (foc->u_mag > v_max) {
        float scale = v_max / foc->u_mag;
        foc->Vd_cmd *= scale;
        foc->Vq_cmd *= scale;
    }
    
    float v_alpha, v_beta;
    inv_park(foc->Vd_cmd, foc->Vq_cmd, theta_e, &v_alpha, &v_beta);     

    modulate(ModulationType::SVPWM,
             v_alpha, v_beta,
             vdc,
             foc->ts,
             dutyA, dutyB, dutyC);
}

void focInjection(FOC_State_t* foc, float freq) {
    const float amplitude_max = 0.002f;
    const float speed_multiplier = 1.5f;
    static float inj_phase = 0.0f;
    static float accumulated_time = 0.0f;

    accumulated_time += foc->ts;
    if (accumulated_time < 0.167f * speed_multiplier) inj_phase += 2.0f * M_PI * 783.99 * foc->ts;
    else if (accumulated_time < 0.333f * speed_multiplier) inj_phase += 2.0f * M_PI * 698.46f * foc->ts;
    else if (accumulated_time < 0.667f * speed_multiplier) inj_phase += 2.0f * M_PI * 440.00f * foc->ts;
    else if (accumulated_time < 1.0f * speed_multiplier) inj_phase += 2.0f * M_PI * 493.88f * foc->ts;

    else if (accumulated_time < 1.167f * speed_multiplier) inj_phase += 2.0f * M_PI * 659.26f * foc->ts;
    else if (accumulated_time < 1.333f * speed_multiplier) inj_phase += 2.0f * M_PI * 587.33f * foc->ts;
    else if (accumulated_time < 1.667f * speed_multiplier) inj_phase += 2.0f * M_PI * 349.23f * foc->ts;
    else if (accumulated_time < 2.0f * speed_multiplier) inj_phase += 2.0f * M_PI * 392.00f * foc->ts;

    else if (accumulated_time < 2.267f * speed_multiplier) inj_phase += 2.0f * M_PI * 587.33f * foc->ts;
    else if (accumulated_time < 2.333f * speed_multiplier) inj_phase += 2.0f * M_PI * 523.25f * foc->ts;
    else if (accumulated_time < 2.667f * speed_multiplier) inj_phase += 2.0f * M_PI * 329.63f * foc->ts;
    else if (accumulated_time < 3.0f * speed_multiplier) inj_phase += 2.0f * M_PI * 392.00f * foc->ts;
    else if (accumulated_time < 3.33f * speed_multiplier) inj_phase += 2.0f * M_PI * 523.25f * foc->ts;
    else if (accumulated_time < 5.0f * speed_multiplier) inj_phase = 0.0f;
    else {
        accumulated_time = 0.0f; // Reset after one full cycle
        inj_phase = 0.0f; // Reset phase to prevent overflow
    }

    //inj_phase += 2.0f * M_PI * freq * foc->ts;
    if (inj_phase > 2.0f * M_PI) {
        inj_phase -= 2.0f * M_PI;
    }

    foc->Id_ref += amplitude_max * cordic::sinf(inj_phase) * foc->Vdc;
}