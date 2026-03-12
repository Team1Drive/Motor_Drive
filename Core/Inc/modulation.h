#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

#define M_PI   3.14159265358979323846264338327950288f
#define SQRT3  1.73205080756887729352744634150587236f

// ─────────────────────────────────────────────────────────────────────────────
//  Modulation type selector
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Selects the PWM modulation strategy used by Modulate().
 *
 * @note SVPWM_COMP and SVPWM_SUPERPOS require a valid Ts (PWM period) to be
 *       passed to Modulate(). All other modes ignore Ts — pass 0.0f.
 */
enum class ModulationType : uint8_t
{
    SVPWM,          ///< Standard Space Vector PWM
    SVPWM_COMP,     ///< SVPWM with period-compensated timing (needs Ts)
    SVPWM_SUPERPOS, ///< SVPWM with superposition overmodulation (needs Ts)
    SYM_PWM,        ///< Symmetrical sinusoidal PWM with third-harmonic injection
    DPWM0,          ///< Discontinuous PWM variant 0 — clamps phase near negative line-line peak
    DPWM1,          ///< Discontinuous PWM variant 1 — clamps phase with largest absolute value
    DPWM2,          ///< Discontinuous PWM variant 2 — clamps phase near positive line-line peak
    DPWM3,          ///< Discontinuous PWM variant 3 — clamps phase furthest from zero
};

// ─────────────────────────────────────────────────────────────────────────────
//  Coordinate transforms
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Clarke transformation. Converts three-phase quantities (a, b, c) into
 *        two orthogonal components (α, β) in the stationary reference frame.
 * @param a     Phase A quantity (e.g., voltage or current)
 * @param b     Phase B quantity (e.g., voltage or current)
 * @param c     Phase C quantity (e.g., voltage or current)
 * @param alpha Pointer to store calculated α component
 * @param beta  Pointer to store calculated β component
 * @note The function assumes balanced three-phase inputs where a + b + c = 0.
 *       For unbalanced inputs, the zero-sequence component is ignored.
 */
void clarke(float a, float b, float c, float* alpha, float* beta);

/**
 * @brief Inverse Clarke transformation. Converts α-β components back into
 *        three-phase quantities (a, b, c).
 * @param alpha α component in the stationary reference frame
 * @param beta  β component in the stationary reference frame
 * @param a     Pointer to store calculated phase A quantity
 * @param b     Pointer to store calculated phase B quantity
 * @param c     Pointer to store calculated phase C quantity
 */
void inv_clarke(float alpha, float beta, float* a, float* b, float* c);

/**
 * @brief Park transformation. Converts α-β components in the stationary
 *        reference frame into d-q components in the rotating reference frame.
 * @param alpha α component in the stationary reference frame
 * @param beta  β component in the stationary reference frame
 * @param theta Electrical angle of the rotating reference frame (in radians)
 * @param d     Pointer to store calculated d-axis component (flux-producing)
 * @param q     Pointer to store calculated q-axis component (torque-producing)
 */
void park(float alpha, float beta, float theta, float* d, float* q);

/**
 * @brief Inverse Park transformation. Converts d-q components in the rotating
 *        reference frame back into α-β components in the stationary reference frame.
 * @param d     d-axis component in the rotating reference frame
 * @param q     q-axis component in the rotating reference frame
 * @param theta Electrical angle of the rotating reference frame (in radians)
 * @param alpha Pointer to store calculated α component in the stationary reference frame
 * @param beta  Pointer to store calculated β component in the stationary reference frame
 */
void inv_park(float d, float q, float theta, float* alpha, float* beta);

// ─────────────────────────────────────────────────────────────────────────────
//  Master modulation call
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Unified PWM modulation function. Converts α-β voltage components into
 *        duty cycles for a three-phase inverter using the selected modulation strategy.
 *
 * @param type    Modulation strategy to use (see ModulationType)
 * @param v_alpha α-axis voltage component from inv_park() (in volts)
 * @param v_beta  β-axis voltage component from inv_park() (in volts)
 * @param v_dc    DC bus voltage (in volts)
 * @param Ts      PWM period in seconds (e.g. 1.0f/20000.0f for 20 kHz).
 *                Only required for SVPWM_COMP and SVPWM_SUPERPOS — pass 0.0f
 *                for all other modes. Derive from hardware using:
 *                (float)(htim8.Init.Period + 1) / HAL_RCC_GetPCLK2Freq()
 * @param dutyA   Pointer to store calculated duty cycle for phase A (0.0 to 1.0)
 * @param dutyB   Pointer to store calculated duty cycle for phase B (0.0 to 1.0)
 * @param dutyC   Pointer to store calculated duty cycle for phase C (0.0 to 1.0)
 *
 * @note v_alpha and v_beta come directly from inv_park(). The modulation index
 *       and phase references are computed internally — do not pre-calculate them.
 *
 * @note Example usage in the TIM8 PWM interrupt (FOC loop):
 * @code
 *   float V_alpha, V_beta;
 *   inv_park(Vd, Vq, theta_e, &V_alpha, &V_beta);
 *
 *   float dutyA, dutyB, dutyC;
 *   Modulate(ModulationType::SVPWM, V_alpha, V_beta, Vdc, 0.0f,
 *            &dutyA, &dutyB, &dutyC);
 *
 *   motorPWM.setDuty(dutyA, dutyB, dutyC);
 * @endcode
 */
void Modulate(
    ModulationType type,
    float v_alpha,
    float v_beta,
    float v_dc,
    float Ts,
    float* dutyA, float* dutyB, float* dutyC
);