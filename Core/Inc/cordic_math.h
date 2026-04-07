#pragma once

#include <stdint.h>

#define M_PI   3.14159265358979323846264338327950288f
#define SQRT3  1.73205080756887729352744634150587236f

int32_t float_to_q31(float angle_rad);

float q31_to_float(int32_t q31_val);

float wrap_to_pi(float angle);

namespace cordic {
    /**
     * @brief Sets the CORDIC mode by writing to the CSR register.
     * This is an internal function and should not be called directly by user code.
     */
    void cordic_set_mode(uint32_t csr_value);

    /**
     * @brief Calculates both Sine and Cosine in a single zero-overhead call.
     * Best used for FOC Park/Inverse Park transforms!
     * @param angle_rad Angle in radians. Must be pre-bounded between -PI and PI.
     */
    void sincos(float angle_rad, float* sin_out, float* cos_out);

    /**
     * @brief Calculates Sine using CORDIC.
     * @return Sine of the input angle.
     * @note If Cosine is also needed, use `cordic_sincos()` instead to avoid redundant CORDIC calls and get both results with zero overhead.
     */
    float sinf(float angle_rad);

    /**
     * @brief Calculates Cosine using CORDIC.
     * @return Cosine of the input angle.
     * @note If Sine is also needed, use `cordic_sincos()` instead to avoid redundant CORDIC calls and get both results with zero overhead.
     */
    float cosf(float angle_rad);

    /**
     * @brief Calculates the magnitude (hypotenuse) of a vector given its x and y components using CORDIC.
     * @return The magnitude of the vector defined by (x, y).
     */
    float hypotf(float x, float y);

    /**
     * @brief Calculates the arctangent of y/x using CORDIC, returning the angle in radians.
     * @return The angle in radians between the positive x-axis and the point (x, y).
     */
    float atan2f(float y, float x);
}
