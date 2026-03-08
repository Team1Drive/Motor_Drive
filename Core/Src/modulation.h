#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

/**
 * @brief Space Vector PWM modulation function. Converts α-β voltage components into duty cycles for three-phase inverter.
 * @param v_alpha α-axis voltage component (in volts)
 * @param v_beta β-axis voltage component (in volts)
 * @param v_dc DC bus voltage (in volts)
 * @param dutyA Pointer to store calculated duty cycle for phase A (0.0 to 1.0)
 * @param dutyB Pointer to store calculated duty cycle for phase B (0.0 to 1.0)
 * @param dutyC Pointer to store calculated duty cycle for phase C (0.0 to 1.0)
 * @note The function assumes that the input voltages are already in the α-β reference
 */
void SVPWM(float v_alpha, float v_beta, float v_dc, float* dutyA, float* dutyB, float* dutyC);

/**
 * @brief Clarke transformation. Converts three-phase quantities (a, b, c) into two orthogonal components (α, β) in the stationary reference frame.
 * @param a Phase A quantity (e.g., voltage or current)
 * @param b Phase B quantity (e.g., voltage or current)
 * @param c Phase C quantity (e.g., voltage or current)
 * @param alpha Pointer to store calculated α component
 * @param beta Pointer to store calculated β component
 * @note The function assumes balanced three-phase inputs where a + b + c = 0. For unbalanced inputs, the zero-sequence component is ignored.
 */
void clarke(float a, float b, float c, float* alpha, float* beta);

/**
 * @brief Inverse Clarke transformation. Converts α-β components back into three-phase quantities (a, b, c).
 * @param alpha α component in the stationary reference frame
 * @param beta β component in the stationary reference frame
 * @param a Pointer to store calculated phase A quantity
 * @param b Pointer to store calculated phase B quantity
 * @param c Pointer to store calculated phase C quantity
 */
void inv_clarke(float alpha, float beta, float* a, float* b, float* c);

/**
 * @brief Park transformation. Converts α-β components in the stationary reference frame into d-q components in the rotating reference frame.
 * @param alpha α component in the stationary reference frame
 * @param beta β component in the stationary reference frame
 * @param theta Electrical angle of the rotating reference frame (in radians)
 * @param d Pointer to store calculated d-axis component
 * @param q Pointer to store calculated q-axis component
 */
void park(float alpha, float beta, float theta, float* d, float* q);

/**
 * @brief Inverse Park transformation. Converts d-q components in the rotating reference frame back into α-β components in the stationary reference frame.
 * @param d d-axis component in the rotating reference frame
 * @param q q-axis component in the rotating reference frame
 * @param theta Electrical angle of the rotating reference frame (in radians)
 * @param alpha Pointer to store calculated α component in the stationary reference frame
 * @param beta Pointer to store calculated β component in the stationary reference frame
 */
void inv_park(float d, float q, float theta, float* alpha, float* beta);