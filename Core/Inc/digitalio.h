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
        inline void write(bool value) {
            HAL_GPIO_WritePin(port_, pin_, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        // Toggle the output level of the pin
        inline void toggle(void) {
            HAL_GPIO_TogglePin(port_, pin_);
        }
};

class DigitalIn {
    private:
        GPIO_TypeDef* port_;
        uint16_t pin_;
    
    public:
        DigitalIn(GPIO_TypeDef* port, uint16_t pin);

        // Read the input level of the pin
        inline bool read(void) const {
            return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
        }
};