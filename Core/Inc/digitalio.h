#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class DigitalOut {
    private:
        GPIO_TypeDef* port_;
        uint16_t pin_;
    
    public:
        DigitalOut(GPIO_TypeDef* port, uint16_t pin);

        // Set the output level of the pin
        void write(bool value);

        // Toggle the output level of the pin
        void toggle(void);
};

class DigitalIn {
    private:
        GPIO_TypeDef* port_;
        uint16_t pin_;
    
    public:
        DigitalIn(GPIO_TypeDef* port, uint16_t pin);

        // Read the input level of the pin
        bool read(void);
};