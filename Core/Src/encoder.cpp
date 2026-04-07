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
    elec_pos_range_(counts_per_rev_ / MOTOR_POLE_PAIRS),
    stall_threshold_(stall_threshold) {
        instance_ = this;
        init();
    }

void Encoder::index_rise(void) {
    index_offset_ = (uint16_t)htim_->Instance->CNT;
    if (!is_synchronized_) is_synchronized_ = true;
    if (zero_aligned_ && !is_zeroed_) {
        elec_zero_offset_ = calcElecOffset(elec_zero_pos_, index_offset_);
        is_zeroed_ = true;
    }
}

uint16_t Encoder::calcElecOffset(uint16_t elec_zero_pos, uint16_t index_offset) {
    int32_t diff =  (int32_t)elec_zero_pos - (int32_t)index_offset;
    diff %= elec_pos_range_;
    if (diff < 0) diff += elec_pos_range_;
    return (uint16_t)diff;
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
    zero_aligned_ = false;
    is_synchronized_ = false;
    is_zeroed_ = false;
    rpm = 0.0f;
    stall_counter_ = 0;
}

HAL_StatusTypeDef Encoder::start(void) {
    return HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
}

void Encoder::elecZeroAlign(void) {
    elec_zero_pos_ = (uint16_t)htim_->Instance->CNT;
    if (!zero_aligned_) zero_aligned_ = true;
    if (is_synchronized_ && !is_zeroed_) {
        elec_zero_offset_ = calcElecOffset(elec_zero_pos_, index_offset_);
        is_zeroed_ = true;
    }
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

uint16_t Encoder::getElecPos(void) const {
    if (!is_synchronized_ || !is_zeroed_) return 0;
    uint16_t current_hw_cnt = (uint16_t)htim_->Instance->CNT;
    int32_t elec_pos = (int32_t)current_hw_cnt - (int32_t)index_offset_ - (int32_t)elec_zero_offset_;
    elec_pos %= (int32_t)elec_pos_range_;
    if (elec_pos < 0) elec_pos += (int32_t)elec_pos_range_;
    return (uint16_t)elec_pos;
}

int8_t Encoder::getDirection(void) const {
    return __HAL_TIM_IS_TIM_COUNTING_DOWN(htim_) ? -1 : 1;
}

float Encoder::getRPM(void) const {
    return rpm;
}

float Encoder::getPos_deg(void) const {
    if (!is_synchronized_) return 0.0f;
    float position = ((float)(getPos() % counts_per_rev_)) / counts_per_rev_ * 360.0f;
    return position;
}

float Encoder::getPos_rad(void) const {
    if (!is_synchronized_) return 0.0f;
    float position = ((float)(getPos() % counts_per_rev_)) / counts_per_rev_ * 2.0f * M_PI;
    return position;
}

float Encoder::getElecPos_rad(void) const {
    if (!is_synchronized_ || !is_zeroed_) return 0.0f;
    float elec_position = ((float)(getElecPos())) / ((float)elec_pos_range_) * 2.0f * M_PI;
    return elec_position;
}