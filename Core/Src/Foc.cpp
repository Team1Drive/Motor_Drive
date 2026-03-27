/*
 * foc.cpp  —  Field-Oriented Control implementation for Motor_Drive
 *
 * Matches the architecture of foc_svpwm_switched_inverter_multimode.m:
 *
 *   Speed PI (outer, decimated)
 *     └─► Iq_ref  (clamped by Imax circle)
 *   Field-weakening PI
 *     └─► Id_ref  (negative, when |u| > Vdc/√3)
 *   Current PI × 2  (inner, with decoupling feed-forward)
 *     d: Vd = Kp*(Id_ref−Id) + Ki*∫(Id_ref−Id) + R*Id − ωe*L*Iq
 *     q: Vq = Kp*(Iq_ref−Iq) + Ki*∫(Iq_ref−Iq) + R*Iq + ωe*L*Id + ωe*ψf
 *   Inverse Park  → Vα, Vβ
 *   SVPWM_COMP    → dutyA, dutyB, dutyC
 */

#include "foc.h"
#include "modulation.h"   /* clarke, park, inv_park, Modulate, ModulationType */
#include <cmath>
#include <algorithm>

/* =========================================================================
 * foc_init
 * ========================================================================= */
void foc_init(FOC_State_t* foc)
{
    /* --- Current PI: inner loop --- */
    foc->pi_d.kp         = FOC_KP_I;
    foc->pi_d.ki         = FOC_KI_I;
    foc->pi_d.integrator = 0.0f;
    foc->pi_d.clamp      = FOC_INT_I_CLAMP;
    foc->pi_q = foc->pi_d;   /* same gains for d and q */

    /* --- Speed PI: outer loop --- */
    foc->pi_speed.kp         = FOC_KP_SP;
    foc->pi_speed.ki         = FOC_KI_SP;
    foc->pi_speed.integrator = 0.0f;
    foc->pi_speed.clamp      = FOC_IMAX;

    /* --- Field-weakening PI --- */
    foc->pi_fw.kp         = FOC_KP_FW;
    foc->pi_fw.ki         = FOC_KI_FW;
    foc->pi_fw.integrator = 0.0f;
    foc->pi_fw.clamp      = -FOC_ID_FW_MIN;

    /* --- Setpoints --- */
    foc->target_rpm  = 0.0f;
    foc->omega_ref   = 0.0f;
    foc->Id_ref      = 0.0f;
    foc->Iq_ref      = 0.0f;

    /* --- Observables --- */
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
 * foc_reset  — zero all integrators (call on stop or fault)
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
    foc->fault     = false;
}

/* =========================================================================
 * foc_run  — one complete FOC tick, called from TIM8 update ISR
 * ========================================================================= */
void foc_run(FOC_State_t* foc,
             float Ia, float Ib, float Ic,
             float Vdc,
             float theta_e, float omega_m,
             float* dutyA, float* dutyB, float* dutyC)
{
    /* -----------------------------------------------------------------------
     * 0. Overcurrent protection — fault and zero duty if exceeded
     * --------------------------------------------------------------------- */
    float I_max_sq = FOC_IMAX * FOC_IMAX;
    if (Ia*Ia > I_max_sq || Ib*Ib > I_max_sq || Ic*Ic > I_max_sq) {
        foc->fault = true;
        *dutyA = 0.5f;   /* mid-rail — both switches off, no shoot-through */
        *dutyB = 0.5f;
        *dutyC = 0.5f;
        return;
    }

    /* -----------------------------------------------------------------------
     * 1. Store observables
     * --------------------------------------------------------------------- */
    const float omega_e  = omega_m * (float)FOC_POLE_PAIRS;
    const float rpm_meas = omega_m * (60.0f / (2.0f * M_PI));

    foc->Ia      = Ia;
    foc->Ib      = Ib;
    foc->Ic      = Ic;
    foc->Vdc     = Vdc;
    foc->theta_e = theta_e;
    foc->omega_e = omega_e;
    foc->rpm     = rpm_meas;

    /* -----------------------------------------------------------------------
     * 2. Clarke transform:  Ia, Ib, Ic  →  Iα, Iβ  (stationary frame)
     *
     *   Iα = (2/3)*(Ia − Ib/2 − Ic/2)   = (2*Ia − Ib − Ic) / 3
     *   Iβ = (2/3)*(√3/2)*(Ib − Ic)      = (Ib − Ic) / √3
     *
     * Using the existing clarke() from modulation.h (same formula).
     * --------------------------------------------------------------------- */
    float I_alpha, I_beta;
    clarke(Ia, Ib, Ic, &I_alpha, &I_beta);

    /* -----------------------------------------------------------------------
     * 3. Park transform:  Iα, Iβ, θe  →  Id, Iq  (rotating dq frame)
     *
     *   Id =  Iα·cos(θe) + Iβ·sin(θe)
     *   Iq = −Iα·sin(θe) + Iβ·cos(θe)
     * --------------------------------------------------------------------- */
    float Id, Iq;
    park(I_alpha, I_beta, theta_e, &Id, &Iq);

    foc->Id = Id;
    foc->Iq = Iq;

    /* -----------------------------------------------------------------------
     * 4. Speed ramp — advance omega_ref toward target each tick
     * --------------------------------------------------------------------- */
    const float omega_target = foc->target_rpm * (2.0f * M_PI / 60.0f);
    const float max_step     = FOC_RAMP_RATE * FOC_TS;
    float delta = omega_target - foc->omega_ref;
    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;
    foc->omega_ref += delta;

    /* -----------------------------------------------------------------------
     * 5. Outer speed PI  (decimated to FOC_SPEED_DIV ticks)
     *
     *   err_sp  = omega_ref − omega_m   (mechanical rad/s)
     *   Iq_ref  = Kp_sp * err_sp + Ki_sp * ∫err_sp
     *
     * Anti-windup: freeze integrator when Iq_ref is saturated.
     * --------------------------------------------------------------------- */
    foc->speed_div_cnt++;
    if (foc->speed_div_cnt >= FOC_SPEED_DIV) {
        foc->speed_div_cnt = 0U;

        const float err_sp = foc->omega_ref - omega_m;
        float Iq_ref = PI_update(&foc->pi_speed, err_sp, FOC_TS * (float)FOC_SPEED_DIV);
        foc->Iq_ref  = Iq_ref;
    }

    /* -----------------------------------------------------------------------
     * 6. Field-weakening PI
     *
     * When the voltage vector magnitude |u| exceeds Vdc/√3, the motor is
     * voltage-limited.  FW PI drives a negative Id to reduce the back-EMF.
     *
     *   U_max_fw  = Vdc / √3
     *   fw_error  = (U_max_fw − |u_prev|) / ωe      (avoid divide-by-zero)
     *   id_fw     = Kp_fw * fw_error + Ki_fw * ∫fw_error   ∈ [Id_fw_min, 0]
     *
     * Only active above a minimum electrical speed to avoid instability at
     * standstill.
     * --------------------------------------------------------------------- */
    const float U_max_fw = Vdc / 1.7321f;   /* Vdc / √3 */
    const float u_mag_prev = foc->u_mag;

    if (omega_e > 10.0f)   /* only above ~10 electrical rad/s (~95 RPM for 4pp) */
    {
        float fw_error  = (U_max_fw - u_mag_prev) / omega_e;
        float id_fw     = PI_update(&foc->pi_fw, fw_error, FOC_TS);

        /* Clamp to [FOC_ID_FW_MIN, 0] — FW only weakens, never boosts flux */
        if (id_fw >  0.0f)          id_fw = 0.0f;
        if (id_fw <  FOC_ID_FW_MIN) id_fw = FOC_ID_FW_MIN;

        foc->Id_ref = id_fw;
    }
    else
    {
        foc->Id_ref = 0.0f;
        PI_reset(&foc->pi_fw);
    }

    /* -----------------------------------------------------------------------
     * 7. Current magnitude limit on Iq_ref
     *
     *   Iq_max = √(Imax² − Id_ref²)
     *
     * Ensures the total current vector stays within the Imax circle.
     * --------------------------------------------------------------------- */
    float Iq_max = sqrtf(std::max(FOC_IMAX * FOC_IMAX - foc->Id_ref * foc->Id_ref, 0.0f));
    if (foc->Iq_ref >  Iq_max) foc->Iq_ref =  Iq_max;
    if (foc->Iq_ref < -Iq_max) foc->Iq_ref = -Iq_max;

    /* -----------------------------------------------------------------------
     * 8. Inner current PI controllers with decoupling feed-forward
     *
     * The decoupling terms cancel the cross-coupling between d and q axes:
     *   Vd = PI_d(err_id) + R*Id − ωe*L*Iq
     *   Vq = PI_q(err_iq) + R*Iq + ωe*L*Id + ωe*ψf
     *
     * This exactly mirrors the MATLAB reference:
     *   vd_cmd = Kp_i*err_id + Ki_i*int_id + R*id_k − omega_e_k*L*iq_k;
     *   vq_cmd = Kp_i*err_iq + Ki_i*int_iq + R*iq_k + omega_e_k*L*id_k
     *            + omega_e_k*psi_f;
     *
     * Output of PI_update is clamped to ±FOC_INT_I_CLAMP (V).
     * Then we add feed-forward, and hard-clamp the total command to ±Vdc/2.
     * --------------------------------------------------------------------- */
    const float err_id = foc->Id_ref - Id;
    const float err_iq = foc->Iq_ref - Iq;

    float Vd_pi = PI_update(&foc->pi_d, err_id, FOC_TS);
    float Vq_pi = PI_update(&foc->pi_q, err_iq, FOC_TS);

    /* Decoupling feed-forward (R·I and ωe·L cross-coupling + back-EMF) */
    float Vd_cmd = Vd_pi + FOC_R * Id  - omega_e * FOC_L * Iq;
    float Vq_cmd = Vq_pi + FOC_R * Iq  + omega_e * FOC_L * Id
                         + omega_e * FOC_PSI_F;

    /* Hard clamp total command to inverter limits */
    const float Vdc_half = Vdc * 0.5f;
    if (Vd_cmd >  Vdc_half) Vd_cmd =  Vdc_half;
    if (Vd_cmd < -Vdc_half) Vd_cmd = -Vdc_half;
    if (Vq_cmd >  Vdc_half) Vq_cmd =  Vdc_half;
    if (Vq_cmd < -Vdc_half) Vq_cmd = -Vdc_half;

    /* Store for telemetry and field-weakening feedback */
    foc->Vd_cmd = Vd_cmd;
    foc->Vq_cmd = Vq_cmd;
    foc->u_mag  = sqrtf(Vd_cmd * Vd_cmd + Vq_cmd * Vq_cmd);

    /* -----------------------------------------------------------------------
     * 9. Inverse Park transform:  Vd, Vq, θe  →  Vα, Vβ
     *
     *   Vα =  Vd·cos(θe) − Vq·sin(θe)
     *   Vβ =  Vd·sin(θe) + Vq·cos(θe)
     * --------------------------------------------------------------------- */
    float V_alpha, V_beta;
    inv_park(Vd_cmd, Vq_cmd, theta_e, &V_alpha, &V_beta);

    /* -----------------------------------------------------------------------
     * 10. SVPWM modulation:  Vα, Vβ  →  dutyA, dutyB, dutyC
     *
     * Using SVPWM_COMP which includes overmodulation handling
     * (matches 'svpwm_comp' mode in the MATLAB reference).
     * Ts is passed for the compensated timing calculation.
     * --------------------------------------------------------------------- */
    Modulate(ModulationType::SVPWM_COMP,
             V_alpha, V_beta,
             Vdc,
             FOC_TS,
             dutyA, dutyB, dutyC);
}