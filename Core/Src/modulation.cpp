#include "modulation.h"
#include "cordic_math.h"
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
                                        int& sector, float& theta_s)
{
    float theta = cordic::atan2f(v_beta, v_alpha);
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
    float cos_t;
    float sin_t;
    cordic::sincosf(theta, &sin_t, &cos_t);
    *d =  alpha * cos_t + beta * sin_t;
    *q = -alpha * sin_t + beta * cos_t;
}

void inv_park(float d, float q, float theta, float* alpha, float* beta)
{
    float cos_t;
    float sin_t;
    cordic::sincosf(theta, &sin_t, &cos_t);
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
                           float* da, float* db, float* dc)
{
    int   sector;
    float theta_s;
    get_sector_and_angle(v_alpha, v_beta, sector, theta_s);

    float v_ref   = hypotf(v_alpha, v_beta);
    float v_ratio = v_ref * SQRT3 / v_dc;                        // normalised magnitude
    //const float csc60 = 2.0f / SQRT3;
    const float sin60 = SQRT3 / 2.0f;
    const float cos60 = 0.5f;

    float sin_theta_s, cos_theta_s;
    sin_theta_s = sinf(theta_s);
    cos_theta_s = cosf(theta_s);
    //cordic::sincosf(theta_s, &sin_theta_s, &cos_theta_s);
    float sin_comp_theta_s = sin60 * cos_theta_s - cos60 * sin_theta_s;   // sin(60°-θs) = √3/2·cos(θs) - 1/2·sin(θs)

    float d2 = sin_theta_s      * v_ratio;
    float d1 = sin_comp_theta_s * v_ratio;
    float d0 = 1.0f - d1 - d2;

    // Normalise d1 and d2 proportionally
    if (d0 < 0.0f) {
        float sum = d1 + d2;
        d1 /= sum;
        d2 /= sum;
        d0 = 0.0f;
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

    float v_ref = cordic::hypotf(v_alpha, v_beta);
    float K     = (SQRT3 * Ts / v_dc) * v_ref;          // time scaling factor

    float T1 = std::max(K * cordic::sinf(M_PI / 3.0f - theta_s), 0.0f);
    float T2 = std::max(K * cordic::sinf(theta_s),               0.0f);
    float T0 = std::max(Ts - T1 - T2,                            0.0f);

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
static void svpwm_superposition(float v_alpha, float v_beta, float v_dc, float Ts,
                                float* da, float* db, float* dc)
{
    int   sector;
    float theta_s;
    get_sector_and_angle(v_alpha, v_beta, sector, theta_s);

    // Compute normalised modulation index from voltage vector magnitude
    float v_ref = cordic::hypotf(v_alpha, v_beta);
    float m     = v_ref / (v_dc * 0.5f);                // [0..1], >1 = overmod
    m = std::min(m, 1.0f);                              // hard cap at six-step

    float s1  = cordic::sinf(M_PI / 3.0f - theta_s);
    float s2  = cordic::sinf(theta_s);
    float cd  = std::max(cordic::cosf(theta_s - M_PI / 6.0f), 1e-6f);

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
                                  float& Va, float& Vb, float& Vc)
{
    // inv_clarke gives phase voltages in volts
    float a, b, c;
    float alpha_n = v_alpha / (v_dc * 0.5f);   // normalise to [-1,1]
    float beta_n  = v_beta  / (v_dc * 0.5f);
    a =  alpha_n;
    b = (-alpha_n + SQRT3 * beta_n) / 2.0f;
    c = (-alpha_n - SQRT3 * beta_n) / 2.0f;
    Va = a; Vb = b; Vc = c;
}

// ─────────────────────────────────────────────
//  4. SYM_PWM – symmetrical / third-harmonic injection
//     Inputs: v_alpha, v_beta, v_dc
//     Zero-sequence centres min/max → same DC bus
//     utilisation as SVPWM with simpler maths
// ─────────────────────────────────────────────
static void sym_pwm(float v_alpha, float v_beta, float v_dc,
                    float* da, float* db, float* dc)
{
    float Va, Vb, Vc;
    get_phase_refs(v_alpha, v_beta, v_dc, Va, Vb, Vc);

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
static void dpwm(float v_alpha, float v_beta, float v_dc,
                 int variant,
                 float* da, float* db, float* dc)
{
    float Va, Vb, Vc;
    get_phase_refs(v_alpha, v_beta, v_dc, Va, Vb, Vc);

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
//  Master entry point
// ═════════════════════════════════════════════════════════════════════════════

void modulate(
    ModulationType type,
    float v_alpha,
    float v_beta,
    float v_dc,
    float Ts,
    float* dutyA, float* dutyB, float* dutyC)
{
    switch (type)
    {
        case ModulationType::SVPWM:
            svpwm_standard(v_alpha, v_beta, v_dc,
                           dutyA, dutyB, dutyC);
            break;

        case ModulationType::SVPWM_COMP:
            svpwm_comp(v_alpha, v_beta, v_dc, Ts,
                       dutyA, dutyB, dutyC);
            break;

        case ModulationType::SVPWM_SUPERPOS:
            svpwm_superposition(v_alpha, v_beta, v_dc, Ts,
                                dutyA, dutyB, dutyC);
            break;

        case ModulationType::SYM_PWM:
            sym_pwm(v_alpha, v_beta, v_dc,
                    dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM0:
            dpwm(v_alpha, v_beta, v_dc, 0, dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM1:
            dpwm(v_alpha, v_beta, v_dc, 1, dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM2:
            dpwm(v_alpha, v_beta, v_dc, 2, dutyA, dutyB, dutyC);
            break;

        case ModulationType::DPWM3:
            dpwm(v_alpha, v_beta, v_dc, 3, dutyA, dutyB, dutyC);
            break;

        default:
            *dutyA = *dutyB = *dutyC = 0.0f;
            break;
    }
}