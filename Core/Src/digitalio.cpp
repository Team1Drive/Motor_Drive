#include "digitalio.h"
//#include <cmath>
#include <cstdint>

DigitalOut::DigitalOut(GPIO_TypeDef* port, uint16_t pin):
    port_(port),
    pin_(pin)
    {}

void DigitalOut::write(bool value) {
        HAL_GPIO_WritePin(port_, pin_, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

void DigitalOut::toggle(void) {
        HAL_GPIO_TogglePin(port_, pin_);
    }

DigitalIn::DigitalIn(GPIO_TypeDef* port, uint16_t pin):
    port_(port),
    pin_(pin)
    {}

bool DigitalIn::read(void) const {
        return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    }