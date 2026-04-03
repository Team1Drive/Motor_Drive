#ifndef CORDIC_MATH_H
#define CORDIC_MATH_H

#include <stdint.h>

#define M_PI   3.14159265358979323846264338327950288f
#define SQRT3  1.73205080756887729352744634150587236f

int32_t float_to_q31(float angle_rad);

float q31_to_float(int32_t q31_val);

float wrap_to_pi(float angle);

void cordic_set_mode(uint32_t csr_value);

/**
 * @brief Calculates both Sine and Cosine in a single zero-overhead call.
 * Best used for FOC Park/Inverse Park transforms!
 * @param angle_rad Angle in radians. Must be pre-bounded between -PI and PI.
 */
void cordic_sincos(float angle_rad, float* sin_out, float* cos_out);

/**
 * @brief Simple wrapper to get just Sine.
 */
float cordic_sin(float angle_rad);

/**
 * @brief Simple wrapper to get just Cosine.
 */
float cordic_cos(float angle_rad);

float cordic_hypot(float x, float y);

float cordic_atan2(float y, float x);

#endif /* CORDIC_MATH_H */