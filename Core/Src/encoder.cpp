#include "encoder.h"
//#include <cmath>
#include <cstdint>

Encoder* Encoder::instance_ = nullptr;

Encoder::Encoder(uint16_t chA, uint16_t chB, uint16_t index, MicrosecondTimer& timer):
    pinA_(chA),
    pinB_(chB),
    indexPin_(index),
    timer_(timer) {
    instance_ = this;
    init();
}

int8_t Encoder::direction_decode(void) {
    // ACW
    if (
        (p_state == 0b00 && state == 0b01) || 
        (p_state == 0b01 && state == 0b11) || 
        (p_state == 0b11 && state == 0b10) || 
        (p_state == 0b10 && state == 0b00)
    ) return 1;
    // CW
    if (
        (p_state == 0b00 && state == 0b10) || 
        (p_state == 0b10 && state == 0b11) || 
        (p_state == 0b11 && state == 0b01) || 
        (p_state == 0b01 && state == 0b00)
    ) return -1;
    return 0;
}

void Encoder::chA_rise(void) {
    count++;
    p_state = state;
    state |= 0b01;
    p_pulse_interval = pulse_interval;
    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
}

void Encoder::chA_fall(void) {
    count++;
    p_state = state;
    state &= 0b10;
    p_pulse_interval = pulse_interval;
    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
}

void Encoder::chB_rise(void) {
    count++;
    p_state = state;
    state |= 0b10;
    p_pulse_interval = pulse_interval;
    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
}

void Encoder::chB_fall(void) {
    count++;
    p_state = state;
    state &= 0b01;
    p_pulse_interval = pulse_interval;
    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
}

void Encoder::index_rise(void) {
    count = 0;
    timer_.reset(USTIMER_ENCODER_INDEX_ID);
}

void Encoder::index_fall(void) {
}

void Encoder::init(void) {
    count = 0;
    state = 0;
    p_state = 0;
    timer_.start(USTIMER_ENCODER_PULSE_ID);
    timer_.start(USTIMER_ENCODER_INDEX_ID);
}

void Encoder::irqHandlerRising(uint16_t pin){
    if (instance_ != nullptr) {
        if (pin == instance_->pinA_) instance_->chA_rise();
        if (pin == instance_->pinB_) instance_->chB_rise();
        if (pin == instance_->indexPin_) instance_->index_rise();
    }
}

void Encoder::irqHandlerFalling(uint16_t pin){
    if (instance_ != nullptr) {
        if (pin == instance_->pinA_) instance_->chA_fall();
        if (pin == instance_->pinB_) instance_->chB_fall();
        if (pin == instance_->indexPin_) instance_->index_fall();
    }
}

void Encoder::reset(void) {
    count = 0;
}

uint32_t Encoder::getCount(void) {
    return count;
}

uint8_t Encoder::getDirection(void) {
    return direction_decode();
}

float Encoder::getRPM(void) {
    if (pulse_interval == 0) return 0; // Avoid division by zero
    float frequency = 1e6f / (pulse_interval); // Convert pulse interval in microseconds to frequency in Hz
    //float frequency = 1e6f / (pulse_interval * 0.8f + p_pulse_interval * 0.2f); // Freqency with simple low-pass filtering
    float rpm = (frequency * 60.0f) / (ENCODER_PPR * 4U); // Calculate RPM from frequency and pulses per revolution
    return rpm;
}

float Encoder::getPos_deg(void) {
    float position = ((float)(count % (ENCODER_PPR * 4U))) / (ENCODER_PPR * 4U) * 360.0f;
    return position;
}

float Encoder::getPos_rad(void) {
    float position = ((float)(count % (ENCODER_PPR * 4U))) / (ENCODER_PPR * 4U) * 2.0f * M_PI;
    return position;
}