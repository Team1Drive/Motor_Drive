#include "hallsensor.h"
//#include <cmath>
#include <cstdint>

HallSensor* HallSensor::instance_ = nullptr;

HallSensor::HallSensor(GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB, GPIO_TypeDef* portC, uint16_t pinC):
    portA_(portA),
    portB_(portB),
    portC_(portC),
    pinA_(pinA),
    pinB_(pinB),
    pinC_(pinC) {
    instance_ = this;
    init();
}

void HallSensor::chA_rise(void) {
    p_state = state;
    state |= 0b001;
}

void HallSensor::chA_fall(void) {
    p_state = state;
    state &= 0b110;
}

void HallSensor::chB_rise(void) {
    p_state = state;
    state |= 0b010;
}

void HallSensor::chB_fall(void) {
    p_state = state;
    state &= 0b101;
}

void HallSensor::chC_rise(void) {
    p_state = state;
    state |= 0b100;
}

void HallSensor::chC_fall(void) {
    p_state = state;
    state &= 0b011;
}

void HallSensor::init(void) {
    state = 0;
    p_state = 0;
}

uint8_t HallSensor::read(void) {
    uint8_t a = HAL_GPIO_ReadPin(portA_, pinA_) == GPIO_PIN_SET ? 1 : 0;
    uint8_t b = HAL_GPIO_ReadPin(portB_, pinB_) == GPIO_PIN_SET ? 1 : 0;
    uint8_t c = HAL_GPIO_ReadPin(portC_, pinC_) == GPIO_PIN_SET ? 1 : 0;
    return (c << 2) | (b << 1) | a;
}

void HallSensor::irqHandlerRising(uint16_t pin){
    if (instance_ != nullptr) {
        if (pin == instance_->pinA_) instance_->chA_rise();
        if (pin == instance_->pinB_) instance_->chB_rise();
        if (pin == instance_->pinC_) instance_->chC_rise();
    }
}

void HallSensor::irqHandlerFalling(uint16_t pin){
    if (instance_ != nullptr) {
        if (pin == instance_->pinA_) instance_->chA_fall();
        if (pin == instance_->pinB_) instance_->chB_fall();
        if (pin == instance_->pinC_) instance_->chC_fall();
    }
}

uint8_t HallSensor::getState(void) {
    return state;
}

void HallSensor::printState(char *buffer) {
    uint8_t a = (state & 0b001) ? 1 : 0;
    uint8_t b = (state & 0b010) ? 1 : 0;
    uint8_t c = (state & 0b100) ? 1 : 0;
    buffer[0] = '0' + a;
    buffer[1] = '0' + b;
    buffer[2] = '0' + c;
    buffer[3] = '\0';
}