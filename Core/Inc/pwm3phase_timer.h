#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class ThreePhasePWMOut {
    private:
        TIM_HandleTypeDef* htim_;
        float duty_A, duty_B, duty_C;
        bool enabled_A, enabled_B, enabled_C;
        uint32_t pwm_freq_Hz;

        void updateCompareValues(void);

    public:
        ThreePhasePWMOut(TIM_HandleTypeDef* htim);

        /**
         * Initialize the timer for 3-phase PWM output. This configures the timer base, channels, and GPIOs.
         * @return HAL status code indicating success or failure of initialization.
         */
        HAL_StatusTypeDef init(void);

        /**
         * Start the PWM outputs on all three channels and their complementary outputs. This enables the timer and starts generating PWM signals.
         * @return HAL status code indicating success or failure of starting the PWM outputs.
         */
        HAL_StatusTypeDef start(void);

        HAL_StatusTypeDef stop(void);

        /**
         * Set the duty cycle for each of the three phases. The duty cycle values should be in the range [0.0, 1.0], where 0.0 corresponds to 0% duty and 1.0 corresponds to 100% duty.
         * If a duty cycle is set to a negative value smaller than -0.5f, the corresponding channel will be disabled.
         * @param duty_A Duty cycle for phase A (0.0 to 1.0), negative values < -0.5f will disable the channel
         * @param duty_B Duty cycle for phase B (0.0 to 1.0), negative values < -0.5f will disable the channel
         * @param duty_C Duty cycle for phase C (0.0 to 1.0), negative values < -0.5f will disable the channel
         */
        void setDuty(float duty_A, float duty_B, float duty_C);

        /**
         * Get the current duty cycle for a specified phase. The returned value is in the range [0.0, 1.0], where 0.0 corresponds to 0% duty and 1.0 corresponds to 100% duty.
         * @param phase Phase index (0 for A, 1 for B, 2 for C)
         * @return Current duty cycle for the specified phase, or -1.0f if the phase index is invalid.
         */
        float getDuty(uint8_t phase);

        /**
         * Configure the dead time for the complementary outputs. The dead time is specified in nanoseconds and will be converted to timer ticks based on the timer clock frequency.
         * @param deadtime_ns Desired dead time in nanoseconds
         * @return HAL status code indicating success or failure of configuring the dead time.
         */
        HAL_StatusTypeDef setDeadTime(uint32_t deadtime_ns);

        /**
         * Set the PWM switching frequency by adjusting the timer's prescaler and auto-reload register. The function calculates the appropriate values to achieve the desired frequency based on the timer clock.
         * @param freq_Hz Desired PWM frequency in Hertz
         * @return HAL status code indicating success or failure of setting the frequency.
         */
        HAL_StatusTypeDef setFrequency(uint32_t freq_Hz);

        /**
         * Get the current PWM switching frequency.
         * @return Current PWM frequency in Hertz.
         */
        uint32_t getFrequency(void);
};