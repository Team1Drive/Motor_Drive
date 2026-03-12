#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

/**
 * @brief MicrosecondTimer class provides a high-resolution timer using a hardware timer peripheral. It allows tracking multiple independent timers using unique identifiers and provides functions to get elapsed time in microseconds, milliseconds, and seconds.
 * The class uses a static instance pointer to handle timer overflow interrupts and maintain an overflow count.
 */
class MicrosecondTimer {
    private:
        static MicrosecondTimer* instance_;
        TIM_HandleTypeDef* htim_;
        volatile uint32_t overflow_count_;
        uint32_t start_tick[256];
        uint32_t start_overflow_count[256];

    public:
        /**
         * @brief Constructor for MicrosecondTimer. Initializes the timer with the given TIM handle and starts it in interrupt mode.
         * @param htim Pointer to a TIM_HandleTypeDef structure that contains the configuration information for the timer to be used as a microsecond timer. The timer should be configured to overflow every 65536 microseconds (for a 1 MHz timer clock).
         * The constructor sets the instance pointer to this object and calls the init() function to start the timer.
         * @note ONLY ONE INSTANCE of MicrosecondTimer should be created, as it uses a static instance pointer for the interrupt handler. The timer specified by htim should be configured to generate an update interrupt on overflow, and the interrupt handler should call MicrosecondTimer::irqHandler() to increment the overflow count.
         */
        MicrosecondTimer(TIM_HandleTypeDef* htim);

        /**
         * @brief Initializes the microsecond timer by resetting overflow counts and starting the timer in interrupt mode.
          * This function is called automatically in the constructor after setting the instance pointer.
         */
        HAL_StatusTypeDef init(void);

        /**
         * @brief Interrupt handler for the timer overflow. Increments the overflow count each time the timer overflows.
         * @param htim Pointer to the TIM_HandleTypeDef structure for which the interrupt occurred.
         */
        static void irqHandler(TIM_HandleTypeDef* htim);

        /**
         * @brief Starts a timing session for a given identifier. Records the current timer tick and overflow count for later elapsed time calculation.
         * @param identifier A unique identifier (0-255) for the timing session. This allows multiple independent timers to be tracked simultaneously.
         */
        void start(uint8_t identifier);

        /**
         * @brief Gets the current timer tick count as a 64-bit value combining the overflow count and the current timer counter. This represents the total elapsed time in microseconds since the timer was initialized.
         * @return The current timer tick count in microseconds.
          * The upper 48 bits represent the overflow count (number of times the timer has overflowed), and the lower 16 bits represent the current timer counter value.
         */
        uint64_t getTick(void);

        /**
         * @brief Gets the elapsed time in microseconds for a given identifier since the last call to start() for that identifier. This is calculated by taking the difference between the current timer tick and the recorded start tick for that identifier.
         * @param identifier The unique identifier (0-255) for the timing session to check the elapsed time for.
         * @return The elapsed time in microseconds since the last call to start() for the given identifier.
         */
        uint64_t getElapsedTime_us(uint8_t identifier);

        /**
         * @brief Gets the elapsed time in milliseconds for a given identifier since the last call to start() for that identifier. This is calculated by taking the difference between the current timer tick and the recorded start tick for that identifier, and then dividing by 1000.
         * @param identifier The unique identifier (0-255) for the timing session to check the elapsed time for.
         * @return The elapsed time in rounded milliseconds since the last call to start() for the given identifier.
         */
        uint64_t getElapsedTime_ms(uint8_t identifier);

        /**
         * @brief Gets the elapsed time in milliseconds as a floating-point value for a given identifier since the last call to start() for that identifier. This is calculated by taking the difference between the current timer tick and the recorded start tick for that identifier, and then dividing by 1000.0f to get a more precise value.
         * @param identifier The unique identifier (0-255) for the timing session to check the elapsed time for.
         * @return The elapsed time in milliseconds as a floating-point value since the last call to start() for the given identifier.
         */
        float getElapsedTimef_ms(uint8_t identifier);

        /**
         * @brief Gets the elapsed time in seconds for a given identifier since the last call to start() for that identifier. This is calculated by taking the difference between the current timer tick and the recorded start tick for that identifier, and then dividing by 1,000,000.
         * @param identifier The unique identifier (0-255) for the timing session to check the elapsed time for.
         * @return The elapsed time in rounded seconds since the last call to start() for the given identifier.
         */
        uint64_t getElapsedTime_s(uint8_t identifier);

        /**
         * @brief Gets the elapsed time in seconds as a floating-point value for a given identifier since the last call to start() for that identifier. This is calculated by taking the difference between the current timer tick and the recorded start tick for that identifier, and then dividing by 1,000,000.0f to get a more precise value.
         * @param identifier The unique identifier (0-255) for the timing session to check the elapsed time for.
         * @return The elapsed time in seconds as a floating-point value since the last call to start() for the given identifier.
         */
        float getElapsedTimef_s(uint8_t identifier);

        /**
         * @brief Resets the timer for a given identifier, stores the current tick, and returns the elapsed time in microseconds since the last reset. This is calculated by taking the difference between the current timer tick and the recorded start tick for that identifier, then updating the start tick and overflow count to the current values.
         * @param identifier The unique identifier (0-255) for the timing session to reset and check the elapsed time for.
         * @return The elapsed time in microseconds since the last call to start() or reset() for the given identifier.
         */
        uint64_t reset(uint8_t identifier);
};