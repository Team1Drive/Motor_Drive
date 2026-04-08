#include "cordic_math.h"
#include "stm32h7xx_hal.h"
#include <math.h>

static const uint32_t CSR_COS_SIN = (14 << CORDIC_CSR_PRECISION_Pos) |
                                    (0 << CORDIC_CSR_SCALE_Pos) |
                                    (1 << CORDIC_CSR_NRES_Pos) |
                                    (0 << CORDIC_CSR_NARGS_Pos) |
                                    (0 << CORDIC_CSR_FUNC_Pos);

static const uint32_t CSR_HYPOT   = (14 << CORDIC_CSR_PRECISION_Pos) |
                                    (0 << CORDIC_CSR_SCALE_Pos) |
                                    (0 << CORDIC_CSR_NRES_Pos) |
                                    (1 << CORDIC_CSR_NARGS_Pos) |
                                    (3 << CORDIC_CSR_FUNC_Pos);

static const uint32_t CSR_ATAN2   = (14 << CORDIC_CSR_PRECISION_Pos) |
                                    (0 << CORDIC_CSR_SCALE_Pos) |
                                    (0 << CORDIC_CSR_NRES_Pos) |
                                    (1 << CORDIC_CSR_NARGS_Pos) |
                                    (2 << CORDIC_CSR_FUNC_Pos);

static uint32_t current_csr = 0;

static const float HYPOT_GAIN = CORDIC_GAIN * SQRT2;  // ≈ 2.3284271247461900976033774484194

float normalize_rad(float angle_rad) {
    return angle_rad * 0.318309886f;
}

int32_t float_to_q31(float value) {
    if (value >= 1.0f) return 2147483647;
    if (value < -1.0f) return -2147483648;
    return static_cast<int32_t>(value * 2147483647.0f);
}

float q31_to_float(int32_t q31_val) {
    return static_cast<float>(q31_val) / 2147483647.0f;
}

float wrap_to_pi(float angle_rad) {
    float angle_wrapped = fmodf(angle_rad, 2.0f * M_PI);
    if (angle_wrapped > M_PI) angle_wrapped -= 2.0f * M_PI;
    if (angle_wrapped < -M_PI) angle_wrapped += 2.0f * M_PI;
    return angle_wrapped;
}

void cordic::cordic_set_mode(uint32_t csr_value) {
    if (current_csr != csr_value) {
        CORDIC->CSR = csr_value;
        current_csr = csr_value;
    }
}

void cordic::sincos(int32_t angle_q31, int32_t* sin_out, int32_t* cos_out) {
    // Ensure CORDIC is in the correct mode for Cos/Sin calculations
    cordic_set_mode(CSR_COS_SIN);
    // CORDIC expects angles normalized between -1.0 and 1.0 (representing -PI to PI)
    // Send the data to CORDIC
    CORDIC->WDATA = angle_q31;

    // Reading RDATA stalls the Cortex-M7 CPU for ~4-6 clock cycles until done.
    // Reading it pops the values out in the order they completed.
    *cos_out = CORDIC->RDATA;
    *sin_out = CORDIC->RDATA;
}

void cordic::sincosf(float angle_rad, float* sin_out, float* cos_out) {
    int32_t s, c;
    cordic::sincos(float_to_q31(normalize_rad(wrap_to_pi(angle_rad))), &s, &c);
    *sin_out = q31_to_float(*sin_out);
    *cos_out = q31_to_float(*cos_out);
}

float cordic::sinf(float angle_rad) {
    int32_t s, c;
    cordic::sincos(float_to_q31(normalize_rad(wrap_to_pi(angle_rad))), &s, &c);
    return q31_to_float(s);
}

float cordic::cosf(float angle_rad) {
    int32_t s, c;
    cordic::sincos(float_to_q31(normalize_rad(wrap_to_pi(angle_rad))), &s, &c);
    return q31_to_float(c);
}

float cordic::hypotf(float x, float y) {
    // Ensure CORDIC is in the correct mode for Hypotenuse calculations
    cordic_set_mode(CSR_HYPOT);

    // Normalize inputs to prevent overflow and maintain precision
    float max_abs = fmaxf(fabsf(x), fabsf(y));

    // If both x and y are very close to zero, return zero to avoid division issues
    if (max_abs < 1e-6f) return 0.0f;

    // Scale down by max_abs * 2.5 to ensure the inputs fit well within the CORDIC range and maintain precision
    float x_norm = x / (max_abs * 2.5f);
    float y_norm = y / (max_abs * 2.5f);

    // Send the normalized data to CORDIC
    CORDIC->WDATA = float_to_q31(x_norm);
    CORDIC->WDATA = float_to_q31(y_norm);

    // Reading RDATA stalls the Cortex-M7 CPU for ~4-6 clock cycles until done.
    int32_t q31_mod = CORDIC->RDATA;
    return q31_to_float(q31_mod) * (max_abs * 2.5f);
}

float cordic::atan2f(float y, float x) {
    // Ensure CORDIC is in the correct mode for Atan2 calculations
    cordic_set_mode(CSR_ATAN2);

    // Normalize inputs to prevent overflow and maintain precision
    float x_abs = fabsf(x);
    float y_abs = fabsf(y);
    float max_abs = (x_abs > y_abs) ? x_abs : y_abs;

    // If both x and y are very close to zero, return zero to avoid division issues
    if (max_abs < 1e-6f) return 0.0f;

    // Scale down by max_abs to ensure the inputs fit well within the CORDIC range and maintain precision
    float x_norm = x / max_abs;
    float y_norm = y / max_abs;

    // Send the normalized data to CORDIC
    CORDIC->WDATA = float_to_q31(x_norm);
    CORDIC->WDATA = float_to_q31(y_norm);

    // Reading RDATA stalls the Cortex-M7 CPU for ~4-6 clock cycles until done.
    int32_t q31_angle = CORDIC->RDATA;
    return q31_to_float(q31_angle) * M_PI;
}