#include "modulation.h"
#include "lut.h"
#include <cmath>
#include <algorithm>

// ═════════════════════════════════════════════════════════════════════════════
//  Internal helpers
// ═════════════════════════════════════════════════════════════════════════════

static inline float clamp01(float x)
{
    return std::min(std::max(x, 0.0f), 1.0f);
}

// Shared sector → phase time mapping used by all SVPWM variants
static inline void sector_to_times(int sector,
                                   float T0, float T1, float T2,
                                   float& Ta, float& Tb, float& Tc)
{
    switch (sector)
    {
        case 1: Ta=T0/2+T1+T2; Tb=T0/2+T2;    Tc=T0/2;         break;
        case 2: Ta=T0/2+T1;    Tb=T0/2+T1+T2; Tc=T0/2;         break;
        case 3: Ta=T0/2;       Tb=T0/2+T1+T2; Tc=T0/2+T2;      break;
        case 4: Ta=T0/2;       Tb=T0/2+T1;    Tc=T0/2+T1+T2;   break;
        case 5: Ta=T0/2+T2;    Tb=T0/2;       Tc=T0/2+T1+T2;   break;
        default:Ta=T0/2+T1+T2; Tb=T0/2;       Tc=T0/2+T1;      break;
    }
}

// Shared sector + angle extraction from v_alpha / v_beta
static inline void get_sector_and_angle(float v_alpha, float v_beta,
                                        int& sector, float& theta_s, float omega_e = 0.0f, float Ts = 0.0f)
{
    float theta = lut::atan2f(v_beta, v_alpha);
    theta += omega_e * Ts * COMP_RATIO;
    if (theta >= 2.0f * M_PI) theta -= 2.0f * M_PI;
    if (theta < 0.0f) theta += 2.0f * M_PI;
    sector  = (int)(theta / (M_PI / 3.0f)) + 1;   // 1-based (1..6)
    if (sector > 6) sector = 6;
    theta_s = theta - (sector - 1) * (M_PI / 3.0f);
}


// ═════════════════════════════════════════════════════════════════════════════
//  Coordinate transforms
// ═════════════════════════════════════════════════════════════════════════════

void clarke(float a, float b, float c, float* alpha, float* beta)
{
    *alpha = (2.0f * a - (b + c)) / 3.0f;
    *beta  = (b - c) / SQRT3;
}

void inv_clarke(float alpha, float beta, float* a, float* b, float* c)
{
    *a =  alpha;
    *b = (-alpha + SQRT3 * beta) / 2.0f;
    *c = (-alpha - SQRT3 * beta) / 2.0f;
}

void park(float alpha, float beta, float theta, float* d, float* q)
{
    float cos_t = lut::cosf(theta);
    float sin_t = lut::sinf(theta);
    *d =  alpha * cos_t + beta * sin_t;
    *q = -alpha * sin_t + beta * cos_t;
}

void inv_park(float d, float q, float theta, float* alpha, float* beta)
{
    float cos_t = lut::cosf(theta);
    float sin_t = lut::sinf(theta);
    *alpha = d * cos_t - q * sin_t;
    *beta  = d * sin_t + q * cos_t;
}


// ═════════════════════════════════════════════════════════════════════════════
//  Modulation algorithms
// ═════════════════════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────
//  1. Standard SVPWM
//     Inputs: v_alpha, v_beta, v_dc
// ─────────────────────────────────────────────
static void svpwm_standard(float v_alpha, float v_beta, float v_dc,
                           float* da, float* db, float* dc, float* applied_mag)
{
    int   sector;
    float theta_s;
    get_sector_and_angle(v_alpha, v_beta, sector, theta_s);

    float v_ref   = lut::hypotf(v_alpha, v_beta);
    float v_ratio = v_ref * SQRT3 / v_dc;                        // normalised magnitude
    //const float csc60 = 2.0f / SQRT3;
    const float sin60 = SQRT3 / 2.0f;
    const float cos60 = 0.5f;

    float sin_theta_s, cos_theta_s;
    sin_theta_s = lut::sinf(theta_s);
    cos_theta_s = lut::cosf(theta_s);
    //cordic::sincosf(theta_s, &sin_theta_s, &cos_theta_s);
    float sin_comp_theta_s = sin60 * cos_theta_s - cos60 * sin_theta_s;   // sin(60°-θs) = √3/2·cos(θs) - 1/2·sin(θs)

    float d2 = sin_theta_s      * v_ratio;
    float d1 = sin_comp_theta_s * v_ratio;
    float d0 = 1.0f - d1 - d2;

    // Normalise d1 and d2 proportionally
    bool saturated = false;
    float scale = 1.0f;

    if (d0 < 0.0f) {
        float sum = d1 + d2;
        scale = 1.0f / sum;
        d1 *= scale;
        d2 *= scale;
        d0 = 0.0f;
        saturated = true;
    }

    if (applied_mag != nullptr) {
        *applied_mag = saturated ? (v_ref * scale) : v_ref;
    }

    float Ta, Tb, Tc;
    sector_to_times(sector, d0, d1, d2, Ta, Tb, Tc);

    *da = clamp01(Ta);
    *db = clamp01(Tb);
    *dc = clamp01(Tc);
}

// ─────────────────────────────────────────────
//  2. SVPWM Compensated  (Ts-based timing)
//     Inputs: v_alpha, v_beta, v_dc, Ts
//     Ts converts duty ratios into real times,
//     giving more accurate switching at high speed
// ─────────────────────────────────────────────
static void svpwm_comp(float v_alpha, float v_beta, float v_dc, float Ts,
                       float* da, float* db, float* dc)
{
    int   sector;
    float theta_s;
    get_sector_and_angle(v_alpha, v_beta, sector, theta_s);

    float v_ref = lut::hypotf(v_alpha, v_beta);
    float K     = (SQRT3 * Ts / v_dc) * v_ref;          // time scaling factor

    float T1 = std::max(K * lut::sinf(M_PI / 3.0f - theta_s), 0.0f);
    float T2 = std::max(K * lut::sinf(theta_s),               0.0f);
    float T0 = std::max(Ts - T1 - T2,                    0.0f);

    float Ta, Tb, Tc;
    sector_to_times(sector, T0, T1, T2, Ta, Tb, Tc);

    *da = clamp01(Ta / Ts);
    *db = clamp01(Tb / Ts);
    *dc = clamp01(Tc / Ts);
}

// ─────────────────────────────────────────────
//  3. SVPWM Superposition  (smooth overmodulation)
//     Inputs: v_alpha, v_beta, v_dc, Ts
//     m is computed internally from |Vref| / (Vdc/2)
//     Blends linearly into six-step as m → 1
// ─────────────────────────────────────────────
static void svpwm_superposition(float v_alpha, float v_beta, float v_dc, float Ts, float omega_e,
                                float* da, float* db, float* dc)
{
    int   sector;
    float theta_s;
    get_sector_and_angle(v_alpha, v_beta, sector, theta_s, omega_e, Ts);

    // Compute normalised modulation index from voltage vector magnitude
    float v_ref = lut::hypotf(v_alpha, v_beta);
    //float m     = v_ref / (v_dc * 0.5f);                // [0..1], >1 = overmod
    float m = v_ref / (2.0f * v_dc / M_PI);
    m = std::min(m, 1.0f);                              // hard cap at six-step

    float s1  = lut::sinf(M_PI / 3.0f - theta_s);
    float s2  = lut::sinf(theta_s);
    float cd  = std::max(lut::cosf(theta_s - M_PI / 6.0f), 1e-6f);

    float T1, T2;

    if (m <= 0.907f)
    {
        // Linear region – identical to standard SVPWM
        float eta = (2.0f * SQRT3 / M_PI) * m;
        T1 = eta * s1 * Ts;
        T2 = eta * s2 * Ts;
    }
    else if (m <= 0.9514f)
    {
        // Overmodulation region I
        float e = (m - 0.907f) / (0.9514f - 0.907f);
        T1 = ((1.0f - e) * s1 + e * (s1 / cd)) * Ts;
        T2 = ((1.0f - e) * s2 + e * (s2 / cd)) * Ts;
    }
    else
    {
        // Overmodulation region II – approaching six-step
        float e = (m - 0.9514f) / (1.0f - 0.9514f);
        if (theta_s <= M_PI / 6.0f)
        {
            T1 = (1.0f - e) * (s1 / cd) * Ts + e * Ts;
            T2 = (1.0f - e) * (s2 / cd) * Ts;
        }
        else
        {
            T1 = (1.0f - e) * (s1 / cd) * Ts;
            T2 = (1.0f - e) * (s2 / cd) * Ts + e * Ts;
        }
    }

    float T0 = std::max(Ts - T1 - T2, 0.0f);

    float Ta, Tb, Tc;
    sector_to_times(sector, T0, T1, T2, Ta, Tb, Tc);

    *da = clamp01(Ta / Ts);
    *db = clamp01(Tb / Ts);
    *dc = clamp01(Tc / Ts);
}

// ─────────────────────────────────────────────
//  Helper: reconstruct per-phase voltages from
//  v_alpha / v_beta using inverse Clarke.
//  Returns Va, Vb, Vc normalised by Vdc/2
//  so the range is [-1, 1].
// ─────────────────────────────────────────────
static inline void get_phase_refs(float v_alpha, float v_beta, float v_dc,
                                  float& Va, float& Vb, float& Vc, float omega_e = 0.0f, float Ts = 0.0f)
{
    // inv_clarke gives phase voltages in volts
    //float alpha_n = v_alpha / (v_dc * 0.5f);   // normalise to [-1,1]
    //float beta_n  = v_beta  / (v_dc * 0.5f);
    //float alpha_n = v_alpha / (v_dc / SQRT3);   // Corrected: normalise to [-1,1] using Vdc/√3
    //float beta_n  = v_beta  / (v_dc / SQRT3);
    float alpha_n = v_alpha / (2 * v_dc / M_PI);   // Corrected: normalise to [-1,1] using 2Vdc/π
    float beta_n  = v_beta  / (2 * v_dc / M_PI);
    
    float theta_comp = omega_e * Ts * COMP_RATIO;
    float cos_t = lut::cosf(theta_comp);
    float sin_t = lut::sinf(theta_comp);

    float alpha_c = alpha_n * cos_t - beta_n * sin_t;
    float beta_c  = alpha_n * sin_t + beta_n * cos_t;

    Va = alpha_c;
    Vb = (-alpha_c + SQRT3 * beta_c) / 2.0f;
    Vc = (-alpha_c - SQRT3 * beta_c) / 2.0f;
}

// ─────────────────────────────────────────────
//  4. SYM_PWM – symmetrical / third-harmonic injection
//     Inputs: v_alpha, v_beta, v_dc
//     Zero-sequence centres min/max → same DC bus
//     utilisation as SVPWM with simpler maths
// ─────────────────────────────────────────────
static void sym_pwm(float v_alpha, float v_beta, float v_dc, float Ts, float omega_e,
                    float* da, float* db, float* dc)
{
    float Va, Vb, Vc;
    get_phase_refs(v_alpha, v_beta, v_dc, Va, Vb, Vc, omega_e, Ts);

    // Zero-sequence = midpoint of max and min
    float z = -(std::min({Va, Vb, Vc}) + std::max({Va, Vb, Vc})) / 2.0f;

    *da = clamp01(0.5f * (Va + z + 1.0f));
    *db = clamp01(0.5f * (Vb + z + 1.0f));
    *dc = clamp01(0.5f * (Vc + z + 1.0f));
}

// ─────────────────────────────────────────────
//  5-8. DPWM variants 0-3
//     Inputs: v_alpha, v_beta, v_dc
//     Each variant clamps a different phase to
//     a rail for 120°/cycle → ~33% fewer switches
// ─────────────────────────────────────────────
static void dpwm(float v_alpha, float v_beta, float v_dc, float Ts, float omega_e,
                 int variant,
                 float* da, float* db, float* dc)
{
    float Va, Vb, Vc;
    get_phase_refs(v_alpha, v_beta, v_dc, Va, Vb, Vc, omega_e, Ts);

    float Vab = Va - Vb;
    float Vbc = Vb - Vc;
    float Vca = Vc - Va;

    float F = 0.0f, G = 0.0f;

    switch (variant)
    {
        case 0: // Clamp phase nearest to negative line-line peak
            F = std::fabs(std::min({Vab,Vbc,Vca})) - std::fabs(std::max({Vab,Vbc,Vca}));
            G = (F <= 0.0f) ? (std::max({Va,Vb,Vc}) - 1.0f)
                            : (std::min({Va,Vb,Vc}) + 1.0f);
            break;

        case 1: // Clamp phase with largest absolute value
            F = std::fabs(std::min({Va,Vb,Vc})) - std::fabs(std::max({Va,Vb,Vc}));
            G = (F <= 0.0f) ? (std::max({Va,Vb,Vc}) - 1.0f)
                            : (std::min({Va,Vb,Vc}) + 1.0f);
            break;

        case 2: // Clamp phase nearest to positive line-line peak
            F = std::fabs(std::max({Vab,Vbc,Vca})) - std::fabs(std::min({Vab,Vbc,Vca}));
            G = (F <= 0.0f) ? (std::max({Va,Vb,Vc}) - 1.0f)
                            : (std::min({Va,Vb,Vc}) + 1.0f);
            break;

        case 3: // Clamp phase furthest from zero
            F = std::fabs(std::max({Va,Vb,Vc})) - std::fabs(std::min({Va,Vb,Vc}));
            G = (F <= 0.0f) ? (std::max({Va,Vb,Vc}) - 1.0f)
                            : (std::min({Va,Vb,Vc}) + 1.0f);
            break;
    }

    float z = -G;

    *da = clamp01(0.5f * (Va + z + 1.0f));
    *db = clamp01(0.5f * (Vb + z + 1.0f));
    *dc = clamp01(0.5f * (Vc + z + 1.0f));
}

// ═════════════════════════════════════════════════════════════════════════════
//  optimal_final — internal subfunctions
// ═════════════════════════════════════════════════════════════════════════════

// Maps sector + T1/T2/T0 times to per-phase duty cycles.
// Direct port of sector_to_duty_opt / svpwm_times_to_abc from MATLAB.
static void sector_to_duty_opt(int sector, float T1, float T2, float T0, float Ts,
                                float& da, float& db, float& dc)
{
    float T0h = T0 * 0.5f;
    switch (sector)
    {
        case 1: da=(T1+T2+T0h)/Ts; db=(T2+T0h)/Ts;    dc=(T0h)/Ts;         break;
        case 2: da=(T1+T0h)/Ts;    db=(T1+T2+T0h)/Ts; dc=(T0h)/Ts;         break;
        case 3: da=(T0h)/Ts;       db=(T1+T2+T0h)/Ts; dc=(T2+T0h)/Ts;      break;
        case 4: da=(T0h)/Ts;       db=(T1+T0h)/Ts;    dc=(T1+T2+T0h)/Ts;   break;
        case 5: da=(T2+T0h)/Ts;    db=(T0h)/Ts;       dc=(T1+T2+T0h)/Ts;   break;
        default:da=(T1+T2+T0h)/Ts; db=(T0h)/Ts;       dc=(T1+T0h)/Ts;      break;
    }
    da = clamp01(da);
    db = clamp01(db);
    dc = clamp01(dc);
}

// Standard SVPWM via midpoint zero-sequence injection.
// MATLAB: svpwm_linear_opt — takes phase voltages va/vb/vc directly.
static void svpwm_linear_opt(float va, float vb, float vc, float v_dc,
                              float& da, float& db, float& dc)
{
    float v_max = std::max({va, vb, vc});
    float v_min = std::min({va, vb, vc});
    float v0    = -0.5f * (v_max + v_min);          // centring zero-sequence
    da = clamp01((va + v0) / v_dc + 0.5f);
    db = clamp01((vb + v0) / v_dc + 0.5f);
    dc = clamp01((vc + v0) / v_dc + 0.5f);
}

// Overmodulation region I SVPWM.
// MATLAB: svpwm_om1_opt — blends linear and OM1 T1/T2 using eta.
static void svpwm_om1_opt(float va, float vb, float vc,
                           float m_filt, float theta_e, float Ts, float v_dc,
                           float& da, float& db, float& dc)
{
    (void)va; (void)vb; (void)vc; (void)v_dc;   // timing-based — phase voltages not used directly

    float theta  = std::fmod(theta_e, 2.0f * M_PI);
    if (theta < 0.0f) theta += 2.0f * M_PI;
    int   sector = (int)(theta / (M_PI / 3.0f)) + 1;
    if (sector > 6) sector = 6;
    float alpha  = theta - (float)(sector - 1) * (M_PI / 3.0f);

    // eta: 0 = pure linear SVPWM, 1 = full OM1 (six-step boundary)
    // MATLAB: num = (m_filt*2/pi) - (2/sqrt(3));  den = (sqrt(3)*log(3/pi)) - (1/sqrt(3));
    float num = (m_filt * 2.0f / M_PI) - (2.0f / SQRT3);
    float den = (SQRT3 * std::log(3.0f / M_PI)) - (1.0f / SQRT3);  //either ln or log
    float eta = clamp01(num / den);

    float cd  = std::max(lut::cosf(M_PI / 6.0f - alpha), 1e-6f);
    float sin_comp = lut::sinf(M_PI / 3.0f - alpha);
    float sin_a    = lut::sinf(alpha);

    float T1 = ((1.0f - eta) * sin_comp + eta * (sin_comp / cd)) * Ts;
    float T2 = ((1.0f - eta) * sin_a    + eta * (sin_a    / cd)) * Ts;
    float T0 = std::max(0.0f, Ts - T1 - T2);

    sector_to_duty_opt(sector, T1, T2, T0, Ts, da, db, dc);
}

// Blend between linear SVPWM and OM1.
// MATLAB: blend_linear_om1_opt
static void blend_linear_om1_opt(float va, float vb, float vc,
                                  float m_filt, float theta_e, float Ts, float v_dc,
                                  float m_lo, float m_hi,
                                  float& da, float& db, float& dc)
{
    float lambda = clamp01((m_filt - m_lo) / (m_hi - m_lo));
    float da_lin, db_lin, dc_lin;
    float da_om1, db_om1, dc_om1;
    svpwm_linear_opt(va, vb, vc, v_dc, da_lin, db_lin, dc_lin);
    svpwm_om1_opt   (va, vb, vc, m_filt, theta_e, Ts, v_dc, da_om1, db_om1, dc_om1);
    da = clamp01((1.0f - lambda) * da_lin + lambda * da_om1);
    db = clamp01((1.0f - lambda) * db_lin + lambda * db_om1);
    dc = clamp01((1.0f - lambda) * dc_lin + lambda * dc_om1);
}

// Generalised Discontinuous PWM.
// MATLAB: gdpwm_opt — clamps the dominant phase to a rail.
static void gdpwm_opt(float va, float vb, float vc,
                      float psi, float v_dc,
                      float& da, float& db, float& dc)
{
    float psi_m = psi - M_PI / 6.0f;
    float cos_p = lut::cosf(psi_m);
    float sin_p = lut::sinf(psi_m);

    // Rotate phase voltages to find dominant phase (MATLAB: va_r, vb_r, vc_r)
    float va_r =  va * cos_p - (vc - vb) / SQRT3 * sin_p;
    float vb_r =  vb * cos_p + (0.5f * (vc - vb) / SQRT3 - (SQRT3 / 2.0f) * va) * sin_p;
    float vc_r = -va_r - vb_r;

    // Find phase with largest absolute rotated value
    float abs_a = std::fabs(va_r);
    float abs_b = std::fabs(vb_r);
    float abs_c = std::fabs(vc_r);
    float vx;
    if (abs_a >= abs_b && abs_a >= abs_c)      vx = va;
    else if (abs_b >= abs_a && abs_b >= abs_c) vx = vb;
    else                                        vx = vc;

    // Zero-sequence to clamp dominant phase to nearest rail (prolly this is the wrong place)
    float sign_vx = (vx > 0.0f) ? 1.0f : ((vx < 0.0f) ? -1.0f : 0.0f);
    float v0 = sign_vx * (v_dc * 0.5f) - vx;

    da = clamp01((va + v0) / v_dc + 0.5f);
    db = clamp01((vb + v0) / v_dc + 0.5f);
    dc = clamp01((vc + v0) / v_dc + 0.5f);
}

// Blend between OM1 and GDPWM.
// MATLAB: blend_om1_gdpwm_opt
static void blend_om1_gdpwm_opt(float va, float vb, float vc,
                                 float m_filt, float theta_e, float Ts, float v_dc,
                                 float m_lo, float m_hi, float psi,
                                 float& da, float& db, float& dc)
{
    float lambda = clamp01((m_filt - m_lo) / (m_hi - m_lo));
    float da_om1, db_om1, dc_om1;
    float da_gd,  db_gd,  dc_gd;
    svpwm_om1_opt(va, vb, vc, m_filt, theta_e, Ts, v_dc, da_om1, db_om1, dc_om1);
    gdpwm_opt    (va, vb, vc, psi, v_dc,             da_gd,  db_gd,  dc_gd);
    da = clamp01((1.0f - lambda) * da_om1 + lambda * da_gd);
    db = clamp01((1.0f - lambda) * db_om1 + lambda * db_gd);
    dc = clamp01((1.0f - lambda) * dc_om1 + lambda * dc_gd);
}

// Six-step synchronised output.
// MATLAB: six_step_synchronized_opt
// Returns T_next (seconds) for hold_counter calculation.
static float six_step_synchronized_opt(float theta_el, float omega_e, float f_target,
                                        float Kp, float Ts,
                                        float& da, float& db, float& dc)
{
    float f_e    = omega_e / (2.0f * M_PI);
    float n_ideal = f_target / f_e;
    int   n       = std::max(6, (int)(std::round(n_ideal / 6.0f) * 6.0f));

    float T_int     = 2.0f * M_PI / ((float)n * omega_e);
    //float theta_est = theta + omega_e * T_int;
    float theta_est = theta_el + omega_e * T_int;
    //int   sector_i  = (int)(theta / (M_PI / 3.0f)) + 1;
    int   sector_i  = (int)(theta_el / (M_PI / 3.0f)) + 1;
    if (sector_i > 6) sector_i = 6;
    float target_th = (float)sector_i * (M_PI / 3.0f);

    // Phase-corrected timing
    float delta_th  = std::fmod(target_th - theta_est + M_PI, 2.0f * M_PI) - M_PI;
    float half_step = M_PI / (float)n;
    delta_th        = std::max(-half_step, std::min(half_step, delta_th));
    float delta_T   = Kp * (delta_th / omega_e);
    float max_corr  = 0.1f * T_int;
    delta_T         = std::max(-max_corr, std::min(max_corr, delta_T));
    float T_next    = std::max(1e-6f, T_int + delta_T);

    // Voltage-vector sector → gate pattern
    int   sector_v = (int)(theta_el / (M_PI / 3.0f));
    sector_v       = ((sector_v % 6) + 6) % 6 + 1;   // clamp to 1..6

    // Six-step gate pattern (100/110/010/011/001/101 sequence)
    switch (sector_v)
    {
        case 1: da=1.0f; db=0.0f; dc=0.0f; break;
        case 2: da=1.0f; db=1.0f; dc=0.0f; break;
        case 3: da=0.0f; db=1.0f; dc=0.0f; break;
        case 4: da=0.0f; db=1.0f; dc=1.0f; break;
        case 5: da=0.0f; db=0.0f; dc=1.0f; break;
        default:da=1.0f; db=0.0f; dc=1.0f; break;
    }
    return T_next;
}


// ═════════════════════════════════════════════════════════════════════════════
//  modulate_optimal_final  — public entry point
//
//  MATLAB inputs used:
//    v_alpha, v_beta   → inv_clarke to get va_ref/vb_ref/vc_ref
//    Vdc               → v_dc
//    Ts                → Ts
//    theta_e_k         → theta_e   (electrical angle, rad)
//    MiA(k)            → m_act     (normalised modulation index)
//    omega_e_k         → omega_e   (electrical angular velocity, rad/s)
//    opt_state         → S         (persistent state struct, passed by reference)
// ═════════════════════════════════════════════════════════════════════════════
void modulate_optimal_final(float v_alpha, float v_beta, float v_dc, float Ts,
                             float theta_e, float theta_m, float m_act, float m_sixstep, float omega_e,
                             float iq_ref, float iq_max, OptimalFinalState& S,
                             float* dutyA, float* dutyB, float* dutyC,
                             bool* just_exited, bool* is_active) {
    // Default safe output
    *dutyA = *dutyB = *dutyC = 0.5f;
    if (just_exited) *just_exited = false;
    if (is_active) *is_active = S.six_step_active;

    // ── Phase voltage references from inverse Clarke ──────────────────────
    // MATLAB: va_ref = v_alpha;  vb_ref = -0.5*v_alpha + (√3/2)*v_beta;  etc.
    float va_ref, vb_ref, vc_ref;
    inv_clarke(v_alpha, v_beta, &va_ref, &vb_ref, &vc_ref);

    // ── Region determination (with one-step hysteresis) ───────────────────
    int nat_region;
    if      (m_act < S.M_LIN_MAX)   nat_region = 1;
    else if (m_act < S.M_BLEND1_HI) nat_region = 2;
    else if (m_act < S.M_OM1_MAX)   nat_region = 3;
    else if (m_act < S.M_BLEND2_HI) nat_region = 4;
    else if (m_act < S.M_SIX_ENTER) nat_region = 5;
    else                             nat_region = 6;

    if      (nat_region > S.region_curr) S.region_curr++;
    else if (nat_region < S.region_curr) S.region_curr--;

    // ── Six-step entry ────────────────────────────────────────────────────
    // MATLAB: wait for next sector boundary before committing to six-step
    if (S.region_curr == 6 && !S.six_step_active) {

        if (!S.enter_pending) {
            int sector = (int)(std::fmod(theta_e, 2.0f * M_PI) / (M_PI / 3.0f));
            S.target_theta_entry = std::fmod((float)(sector + 1) * (M_PI / 3.0f), 2.0f * M_PI);
            S.enter_pending = true;
        }
        
        float angle_err = theta_e - S.target_theta_entry;
        float x = std::abs(std::atan2(std::sin(angle_err), std::cos(angle_err)));
        float y = std::max(std::abs(omega_e) * Ts, 0.01f);

        if (x < y) {
            S.six_step_active = true;
            S.enter_pending   = false;
        }
    }

    // ── Six-step exit ─────────────────────────────────────────────────────
    bool exit_condition = (m_sixstep < S.M_SIX_EXIT) || (iq_ref <= 0.0f);
    if (S.six_step_active && exit_condition)
    {
        S.region_curr = 5;
        if (!S.exit_pending)
        {
            int sector = (int)(std::fmod(theta_e, 2.0f * M_PI) / (M_PI / 3.0f));
            S.target_theta_exit = std::fmod((float)(sector + 1) * (M_PI / 3.0f), 2.0f * M_PI);
            S.exit_pending = true;
        }
        
    float angle_err = theta_e - S.target_theta_exit;
    float x = std::abs(std::atan2(std::sin(angle_err), std::cos(angle_err)));
    float y = std::max(std::abs(omega_e) * Ts, 0.01f);
    if (x < y)

        {
            S.six_step_active = false;
            S.exit_pending    = false;
            if (just_exited) *just_exited = true;
        }
    }

    // ── Modulator selection ───────────────────────────────────────────────
    float da_out = 0.5f, db_out = 0.5f, dc_out = 0.5f;

    if (S.six_step_active)
    {

        float pole_pairs = 4.0f;                     // should be passed or made global
        float theta_six = (pole_pairs * theta_m) - M_PI / 3.0f + M_PI;
        float iq_max_safe = std::max(iq_max, 1e-6f);
        float phase_advance = (iq_ref / iq_max_safe) * 1.7f;
        theta_six = std::fmod(theta_six + phase_advance, 2.0f * M_PI);
        if (theta_six < 0.0f) theta_six += 2.0f * M_PI;

        // Recompute pattern only when hold expires
        if (S.hold_counter <= 0)
        {
            float omega_safe = std::max(omega_e, 2.0f * M_PI * 50.0f);
            float T_next = six_step_synchronized_opt(
                theta_six, omega_safe,
                S.f_target_six, S.Kp_six, Ts,
                S.hold_da, S.hold_db, S.hold_dc);
            S.hold_counter = std::max(1, (int)std::round(T_next / Ts));
        }
        da_out = S.hold_da;
        db_out = S.hold_db;
        dc_out = S.hold_dc;
        S.hold_counter--;
    }
    else
    {
        S.hold_counter = 0;
        switch (S.region_curr)
        {
            case 1:
                svpwm_linear_opt(va_ref, vb_ref, vc_ref, v_dc,
                                 da_out, db_out, dc_out);
                break;

            case 2:
                blend_linear_om1_opt(va_ref, vb_ref, vc_ref,
                                     m_act, theta_e, Ts, v_dc,
                                     S.M_LIN_MAX, S.M_BLEND1_HI,
                                     da_out, db_out, dc_out);
                break;

            case 3:
                svpwm_om1_opt(va_ref, vb_ref, vc_ref,
                              m_act, theta_e, Ts, v_dc,
                              da_out, db_out, dc_out);
                break;

            case 4:
                blend_om1_gdpwm_opt(va_ref, vb_ref, vc_ref,
                                    m_act, theta_e, Ts, v_dc,
                                    S.M_OM1_MAX, S.M_BLEND2_HI, S.psi_gdpwm,
                                    da_out, db_out, dc_out);
                break;

            case 5:
                gdpwm_opt(va_ref, vb_ref, vc_ref, S.psi_gdpwm, v_dc,
                          da_out, db_out, dc_out);
                break;

            default:
                // Fallback: stay in linear — should not normally reach here
                svpwm_linear_opt(va_ref, vb_ref, vc_ref, v_dc,
                                 da_out, db_out, dc_out);
                break;
        }
    }

    *dutyA = clamp01(da_out);
    *dutyB = clamp01(db_out);
    *dutyC = clamp01(dc_out);

    if (is_active) *is_active = S.six_step_active;
}



// ═════════════════════════════════════════════════════════════════════════════
//  Master entry point
// ═════════════════════════════════════════════════════════════════════════════

void modulate(
    ModulationType type,
    float v_alpha,
    float v_beta,
    float v_dc,
    float Ts,
    float* dutyA, float* dutyB, float* dutyC,
    float omega_e,
    float* applied_mag) 
{
    switch (type)
    {
        case ModulationType::SVPWM:
            svpwm_standard(v_alpha, v_beta, v_dc,
                           dutyA, dutyB, dutyC,
                           applied_mag);
            break;

        case ModulationType::SVPWM_COMP:
            svpwm_comp(v_alpha, v_beta, v_dc, Ts,
                       dutyA, dutyB, dutyC);
            break;

        case ModulationType::SVPWM_SUPERPOS:
            svpwm_superposition(v_alpha, v_beta, v_dc, Ts, omega_e,
                                dutyA, dutyB, dutyC);
            break;

        case ModulationType::SYM_PWM:
            sym_pwm(v_alpha, v_beta, v_dc, Ts, omega_e,
                    dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM0:
            dpwm(v_alpha, v_beta, v_dc, Ts, omega_e, 0, dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM1:
            dpwm(v_alpha, v_beta, v_dc, Ts, omega_e, 1, dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM2:
            dpwm(v_alpha, v_beta, v_dc, Ts, omega_e, 2, dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM3:
            dpwm(v_alpha, v_beta, v_dc, Ts, omega_e, 3, dutyA, dutyB, dutyC);
            break;

        case ModulationType::OPTIMAL_FINAL:
            // S must be held by the caller across PWM periods
            // (declare OptimalFinalState as a member of your FOC controller)
            // This case should not be reached via the generic modulate() path;
            // call modulate_optimal_final() directly with the state struct.
            *dutyA = *dutyB = *dutyC = 0.5f;
            break;

        default:
            *dutyA = *dutyB = *dutyC = -1.0f;
            break;
    }
}