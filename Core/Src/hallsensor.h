#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class HallSensor {
    private:
        static HallSensor* instance_;
        GPIO_TypeDef* portA_;
        GPIO_TypeDef* portB_;
        GPIO_TypeDef* portC_;
        uint16_t pinA_;
        uint16_t pinB_;
        uint16_t pinC_;
        uint8_t state;
        uint8_t p_state;

        void chA_rise(void);

        void chA_fall(void);

        void chB_rise(void);

        void chB_fall(void);

        void chC_rise(void);

        void chC_fall(void);

    public:
        HallSensor(GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB, GPIO_TypeDef* portC, uint16_t pinC);

        void init(void);

        uint8_t read(void);

        static void irqHandlerRising(uint16_t pin);

        static void irqHandlerFalling(uint16_t pin);

        uint8_t getState(void);

        /**
         * @brief Fills the provided buffer with a string representation of the current Hall sensor state in the format "CBA", where A, B, and C are '0' or '1' representing the state of each sensor.
         * @param buffer A character array with enough space to hold at least 4 characters (3 for the state and 1 for the null terminator).
         */
        void printState(char *buffer);
};