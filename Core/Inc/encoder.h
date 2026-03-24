#pragma once

#include "stm32h7xx_hal.h"
#include "ustimer.h"
#include "parameters.h"
#include <cstdint>

class Encoder {
    private:
        static Encoder* instance_;
        TIM_HandleTypeDef* htim_;
        const uint32_t counts_per_rev_;
        const float speedloop_period;
        uint16_t indexPin_;
        int32_t overflow_count_;
        uint8_t stall_threshold_;

        uint16_t index_offset_;
        uint16_t last_hw_cnt_;
        bool is_synchronized_;

        float rpm;
        uint8_t stall_counter_;

        //int8_t direction_decode(void);

        //void chA_rise(void);

        //void chA_fall(void);

        //void chB_rise(void);

        //void chB_fall(void);

        void index_rise(void);

        void updateSpeed(void);

        void counterOverflow(void);

    public:
        Encoder(TIM_HandleTypeDef* htim, uint16_t index_pin, uint32_t pulses_per_rev, uint32_t speedloop_freq, uint8_t stall_threshold);

        void init(void);

        HAL_StatusTypeDef start(void);

        static void irqHandlerIndex(uint16_t pin);

        static void irqHandlerSpeed(void);

        static void irqHandlerOverflow(void);

        void reset(void);

        /**
         * @brief Gets the current count of encoder pulses. This count is incremented on each rising edge of channel A and channel B, and can be reset to zero using the reset() function. The count represents the total number of pulses detected since the last reset, and can be used to calculate position, speed, and direction of rotation.
         * @return The current count of encoder pulses.
         */
        uint16_t getCount(void);

        /**
         * @brief Gets the current direction of rotation based on the state of the encoder channels. The direction is determined by the sequence of rising and falling edges on channel A and channel B, and is typically represented as 1 for clockwise rotation, -1 for counterclockwise rotation, and 0 for no movement or invalid state.
         * @return The current direction of rotation: 1 for clockwise, -1 for counterclockwise, and 0 for no movement or invalid state.
          * The direction is decoded using the previous and current state of the encoder channels to determine the direction of rotation.
         */
        int8_t getDirection(void);

        /**
         * @brief Gets the current speed of rotation in revolutions per minute (RPM). This is calculated based on the time interval between encoder pulses and the number of pulses per revolution (PPR) defined for the encoder. The function uses the pulse interval measured by the microsecond timer to calculate the RPM, which can be used for speed control and monitoring of the motor.
         * @return The current speed of rotation in RPM. If the pulse interval is zero (indicating no movement), the function returns 0 to avoid division by zero.
         * The RPM is calculated using the formula: RPM = (Frequency * 60) / (PPR * 4), where Frequency is the inverse of the pulse interval in seconds, and PPR is the number of pulses per revolution for the encoder. The factor of 4 accounts for quadrature encoding, which provides four counts per pulse (rising and falling edges of both channels).
          * A simple low-pass filter can be applied to the pulse interval to smooth out RPM calculations, but this is optional and can be adjusted based on the application's requirements.
         */
        float getRPM(void);

        /**
         * @brief Gets the current position of the encoder in degrees. This is calculated based on the count of encoder pulses and the number of pulses per revolution (PPR) defined for the encoder. The position is represented as an angle between 0 and 360 degrees, where 0 degrees corresponds to the index pulse (if available) or the initial position at startup, and increases in the direction of rotation.
         * @return The current position of the encoder in degrees. The position is calculated using the formula: Position (degrees) = (Count % (PPR * 4)) / (PPR * 4) * 360, where Count is the current pulse count, PPR is the number of pulses per revolution, and the factor of 4 accounts for quadrature encoding.
         */
        float getPos_deg(void);

        /**
         * @brief Gets the current position of the encoder in radians. This is calculated based on the count of encoder pulses and the number of pulses per revolution (PPR) defined for the encoder. The position is represented as an angle between 0 and 2π radians, where 0 radians corresponds to the index pulse (if available) or the initial position at startup, and increases in the direction of rotation.
         * @return The current position of the encoder in radians. The position is calculated using the formula: Position (radians) = (Count % (PPR * 4)) / (PPR * 4) * 2π, where Count is the current pulse count, PPR is the number of pulses per revolution, and the factor of 4 accounts for quadrature encoding.
         */
        float getPos_rad(void);
};