#include "digitalio.h"
//#include <cmath>
#include <cstdint>

DigitalOut::DigitalOut(GPIO_TypeDef* port, uint16_t pin):
    port_(port),
    pin_(pin)
    {}

DigitalIn::DigitalIn(GPIO_TypeDef* port, uint16_t pin):
    port_(port),
    pin_(pin)
    {}