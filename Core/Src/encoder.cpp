#include "encoder.h"
//#include <cmath>
#include <cstdint>

Encoder* Encoder::instance_ = nullptr;

Encoder::Encoder(TIM_HandleTypeDef* htim, TIM_HandleTypeDef* htim_t, uint16_t index_pin, uint32_t pulses_per_rev, uint32_t speedloop_freq, uint8_t stall_threshold):
    htim_(htim),
    htim_t_(htim_t),
    indexPin_(index_pin),
    counts_per_rev_(pulses_per_rev * 4),
    speedloop_period(1.0f / (float)speedloop_freq),
    elec_pos_range_(counts_per_rev_ / MOTOR_POLE_PAIRS),
    stall_threshold_(stall_threshold) {
        instance_ = this;
        init();
    }

void Encoder::indexRise(void) {
    index_offset_ = (uint16_t)__HAL_TIM_GET_COUNTER(htim_);
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
    // ---------- 0. Read current hardware count and timer capture ----------
    uint16_t current_hw_cnt = __HAL_TIM_GET_COUNTER(htim_);
    uint16_t t_period_ticks = __HAL_TIM_GET_COMPARE(htim_t_, TIM_CHANNEL_1);
    uint64_t current_ticks = HighResTimer::getTicks();
    
    // ---------- 1. Handle 16-bit signed delta (Handles 0-65535 wrap automatically) ----------
    int16_t delta = (int16_t)(current_hw_cnt - last_hw_cnt_);
    last_hw_cnt_ = current_hw_cnt;
    uint64_t delta_time_ticks = current_ticks - last_ticks_;
    last_ticks_ = current_ticks;

    // ---------- 2. Stall Detection ----------
    if (delta == 0) {
        stall_counter_++;
    } else {
        stall_counter_ = 0;
    }

    if (stall_counter_ >= stall_threshold_) {
        rpm = 0.0f;
        return;
    }

    // ---------- 3. RPM Calculation (Moving Average Filter) ----------
    float rpm_raw_m, rpm_raw_t;

    // 3.a M method RPM (based on pulse interval)
    if (delta_time_ticks > 0) {
        float dt = (float)delta_time_ticks / HighResTimer::TIMER_FREQ;
        rpm_raw_m = ((float)delta / counts_per_rev_) * (60.0f / dt);
    }
    else {
        rpm_raw_m = 0.0f; // Avoid division by zero
    }

    // 3.b T method RPM (based on timer capture)
    if (t_period_ticks > 0) {
        rpm_raw_t = ((float)TIM15_FREQ_HZ * 60.0f) / ((counts_per_rev_ >> 2) * (float)t_period_ticks);
    }
    else {
        rpm_raw_t = 0.0f; // Avoid division by zero
    }
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim_)) rpm_raw_t = -rpm_raw_t;

    // 4. Linear interpolation weights (based on the absolute speed after filtering in the previous cycle)
    float abs_rpm = fabsf(rpm);
    float alpha = 0.0f;  // Weight for M method
    if (abs_rpm >= ENCODER_M_THRESHOLD) {
        alpha = 1.0f;
    } else if (abs_rpm <= ENCODER_T_THRESHOLD) {
        alpha = 0.0f;
    } else {
        alpha = (abs_rpm - ENCODER_T_THRESHOLD) / (ENCODER_M_THRESHOLD - ENCODER_T_THRESHOLD);
    }
    float rpm_raw = alpha * rpm_raw_m + (1.0f - alpha) * rpm_raw_t;    
    if (std::isnan(rpm_raw_m) || std::isinf(rpm_raw_m)) rpm_raw_m = 0.0f;
        
    // 5. Update moving average filter
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

void Encoder::timerOverflow(void) {
    stall_counter_ = stall_threshold_;
}

void Encoder::init(void) {
    overflow_count_ = 0;
    index_offset_ = 0;
    last_hw_cnt_ = 0;
    zero_aligned_ = false;
    is_synchronized_ = false;
    is_zeroed_ = false;
    rpm = 0.0f;
    for (uint8_t i = 0; i < 10; i++) {
        rpm_buffer_[i] = 0.0f;
    }
    stall_counter_ = 0;
}

HAL_StatusTypeDef Encoder::start(void) {    
    HAL_StatusTypeDef status = HAL_OK;
    if (HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL) != HAL_OK) status = HAL_ERROR;
    __HAL_TIM_URS_ENABLE(htim_t_);
    if (HAL_TIM_Base_Start_IT(htim_t_) != HAL_OK) status = HAL_ERROR;
    if (HAL_TIM_IC_Start(htim_t_, TIM_CHANNEL_1) != HAL_OK) status = HAL_ERROR;
    return status;
}

void Encoder::elecZeroAlign(void) {
    elec_zero_pos_ = (uint16_t)__HAL_TIM_GET_COUNTER(htim_);
    if (!zero_aligned_) zero_aligned_ = true;
    if (is_synchronized_ && !is_zeroed_) {
        elec_zero_offset_ = calcElecOffset(elec_zero_pos_, index_offset_);
        is_zeroed_ = true;
    }
}

void Encoder::irqHandlerIndex(uint16_t pin){
    if (instance_ != nullptr) {
        if (pin == instance_->indexPin_) instance_->indexRise();
    }
}

void Encoder::irqHandlerSpeed(void){
    if (instance_ != nullptr) {
        instance_->updateSpeed();
    }
}

void Encoder::irqHandlerEncoderOverflow(void) {
    if (instance_ != nullptr) {
        instance_->counterOverflow();
    }
}

void Encoder::irqHandlerTimerOverflow(void) {
    if (instance_ != nullptr) {
        instance_->timerOverflow();
    }
}

void Encoder::reset(void) {
    __HAL_TIM_SET_COUNTER(htim_, 0);
    overflow_count_ = 0;
    index_offset_ = 0;
    last_hw_cnt_ = 0;
    is_synchronized_ = false;
    rpm = 0.0f;
    for (uint8_t i = 0; i < 10; i++) {
        rpm_buffer_[i] = 0.0f;
    }
    stall_counter_ = 0;
}

uint16_t Encoder::getPos(void) const {
    if (!is_synchronized_) return 0;
    uint16_t current_hw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim_);
    return (uint16_t)(current_hw_cnt - index_offset_) & (counts_per_rev_ - 1);
}

uint16_t Encoder::getPosBypass(void) const {
    uint16_t current_hw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim_);
    return (uint16_t)(current_hw_cnt - index_offset_) & (counts_per_rev_ - 1);
}

uint16_t Encoder::getElecPos(void) const {
    if (!is_synchronized_ || !is_zeroed_) return 0;
    uint16_t current_hw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim_);
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