#include "encoder.h"
//#include <cmath>
#include <cstdint>

Encoder* Encoder::instance_ = nullptr;

Encoder::Encoder(TIM_HandleTypeDef* htim, MicrosecondTimer timer, uint16_t index_pin, uint32_t pulses_per_rev, uint32_t speedloop_freq, uint8_t stall_threshold):
    htim_(htim),
    timer_(timer),
    indexPin_(index_pin),
    counts_per_rev_(pulses_per_rev * 4),
    speedloop_period(1.0f / (float)speedloop_freq),
    stall_threshold_(stall_threshold) {
        instance_ = this;
        init();
    }

//int8_t Encoder::direction_decode(void) {
//    // ACW
//    if (
//        (p_state == 0b00 && state == 0b01) || 
//        (p_state == 0b01 && state == 0b11) || 
//        (p_state == 0b11 && state == 0b10) || 
//        (p_state == 0b10 && state == 0b00)
//    ) return -1;
//    // CW
//    if (
//        (p_state == 0b00 && state == 0b10) || 
//        (p_state == 0b10 && state == 0b11) || 
//        (p_state == 0b11 && state == 0b01) || 
//        (p_state == 0b01 && state == 0b00)
//    ) return 1;
//    return 0;
//}

//void Encoder::chA_rise(void) {
//    count++;
//    p_state = state;
//    state |= 0b01;
//    p_pulse_interval = pulse_interval;
//    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
//}
//
//void Encoder::chA_fall(void) {
//    count++;
//    p_state = state;
//    state &= 0b10;
//    p_pulse_interval = pulse_interval;
//    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
//}
//
//void Encoder::chB_rise(void) {
//    count++;
//    p_state = state;
//    state |= 0b10;
//    p_pulse_interval = pulse_interval;
//    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
//}
//
//void Encoder::chB_fall(void) {
//    count++;
//    p_state = state;
//    state &= 0b01;
//    p_pulse_interval = pulse_interval;
//    pulse_interval = timer_.reset(USTIMER_ENCODER_PULSE_ID);
//}

void Encoder::index_rise(void) {
    index_offset_ = (uint16_t)htim_->Instance->CNT;
    is_synchronized_ = true;
}

void Encoder::updateSpeed(void) {
    uint16_t current_hw_cnt = (uint16_t)htim_->Instance->CNT;
    
    // 1. Handle 16-bit signed delta (Handles 0-65535 wrap automatically)
    int16_t delta = (int16_t)(current_hw_cnt - last_hw_cnt_);
    last_hw_cnt_ = current_hw_cnt;

    if (delta == 0) {
        stall_counter_++;
    } else {
        stall_counter_ = 0;
    }

    if (stall_counter_ >= stall_threshold_) {
        rpm = 0.0f;
        return;
    }

    // 2. RPM Calculation (Moving Average Filter)
    float rpm_raw = ((float)delta / counts_per_rev_) * (60.0f / speedloop_period);
        
    rpm_sum_ -= rpm_buffer_[filter_idx_];
    rpm_buffer_[filter_idx_] = rpm_raw;
    rpm_sum_ += rpm_raw;
    filter_idx_ = (filter_idx_ + 1) % 10U;
    
    filtered_rpm_ = rpm_sum_ / (float)10U;

    //rpm = rpm_raw * 0.8f + rpm * 0.2f;
    rpm = filtered_rpm_;
}

void Encoder::counterOverflow(void) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim_)) {
            overflow_count_--;
        } else {
            overflow_count_++;
        }
}

void Encoder::init(void) {
    overflow_count_ = 0;
    index_offset_ = 0;
    last_hw_cnt_ = 0;
    is_synchronized_ = false;
    rpm = 0.0f;
    stall_counter_ = 0;
}

HAL_StatusTypeDef Encoder::start(void) {
    return HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
}

void Encoder::irqHandlerIndex(uint16_t pin){
    if (instance_ != nullptr) {
        if (pin == instance_->indexPin_) instance_->index_rise();
    }
}

void Encoder::irqHandlerSpeed(void){
    if (instance_ != nullptr) {
        instance_->updateSpeed();
    }
}

void Encoder::irqHandlerOverflow(void) {
    if (instance_ != nullptr) {
        instance_->counterOverflow();
    }
}

void Encoder::reset(void) {
    __HAL_TIM_SET_COUNTER(htim_, 0);
    overflow_count_ = 0;
    index_offset_ = 0;
    last_hw_cnt_ = 0;
    is_synchronized_ = false;
    rpm = 0.0f;
    stall_counter_ = 0;
}

uint16_t Encoder::getPos(void) const {
    if (!is_synchronized_) return 0;
    uint16_t current_hw_cnt = (uint16_t)htim_->Instance->CNT;
    return (uint16_t)(current_hw_cnt - index_offset_) & (counts_per_rev_ - 1);
}

int8_t Encoder::getDirection(void) const {
    return __HAL_TIM_IS_TIM_COUNTING_DOWN(htim_) ? -1 : 1;
}

float Encoder::getRPM(void) const {
    return rpm;
}

float Encoder::getPos_deg(void) const {
    float position = ((float)(getPos() % counts_per_rev_)) / counts_per_rev_ * 360.0f;
    return position;
}

float Encoder::getPos_rad(void) const {
    float position = ((float)(getPos() % counts_per_rev_)) / counts_per_rev_ * 2.0f * M_PI;
    return position;
}