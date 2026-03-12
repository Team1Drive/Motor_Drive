#include "ustimer.h"
//#include <cmath>
#include <cstdint>

MicrosecondTimer* MicrosecondTimer::instance_ = nullptr;

MicrosecondTimer::MicrosecondTimer(TIM_HandleTypeDef* htim):
    htim_(htim),
    overflow_count_(0) {
        instance_ = this;
    }

HAL_StatusTypeDef MicrosecondTimer::init(void) {
    for (int i = 0; i < 256; i++) {
        start_tick[i] = 0;
        start_overflow_count[i] = 0;
    }
    __HAL_TIM_CLEAR_IT(htim_, TIM_IT_UPDATE);
    return HAL_TIM_Base_Start_IT(htim_);
}

void MicrosecondTimer::irqHandler(TIM_HandleTypeDef* htim) {
    if (instance_ != nullptr && htim == instance_->htim_) {
        instance_->overflow_count_++;
    }
}

void MicrosecondTimer::start(uint8_t identifier) {
    if (identifier >= 256) return;
    __disable_irq();
    start_tick[identifier] = __HAL_TIM_GET_COUNTER(htim_);
    start_overflow_count[identifier] = overflow_count_;
    __enable_irq();
}

uint64_t MicrosecondTimer::getTick(void) {
    uint32_t overflow;
    uint32_t tick;
    __disable_irq();
    overflow = overflow_count_;
    tick = __HAL_TIM_GET_COUNTER(htim_);
    __enable_irq();
    return ((uint64_t)overflow << 16) | tick;
}

uint64_t MicrosecondTimer::getElapsedTime_us(uint8_t identifier) {
    if (identifier >= 256) return 0;
    uint64_t current_tick = getTick();
    uint64_t start_tick_full = ((uint64_t)start_overflow_count[identifier] << 16) | start_tick[identifier];
    return current_tick - start_tick_full;
}

uint64_t MicrosecondTimer::getElapsedTime_ms(uint8_t identifier) {
    if (identifier >= 256) return 0;
    return getElapsedTime_us(identifier) / 1000U;
}

float MicrosecondTimer::getElapsedTimef_ms(uint8_t identifier) {
    if (identifier >= 256) return 0.0f;
    return getElapsedTime_us(identifier) / 1000.0f;
}

uint64_t MicrosecondTimer::getElapsedTime_s(uint8_t identifier) {
    if (identifier >= 256) return 0;
    return getElapsedTime_us(identifier) / 1000000U;
}

float MicrosecondTimer::getElapsedTimef_s(uint8_t identifier) {
    if (identifier >= 256) return 0.0f;
    return getElapsedTime_us(identifier) / 1000000.0f;
}

uint64_t MicrosecondTimer::reset(uint8_t identifier) {
    if (identifier >= 256) return 0;
    uint64_t elapsed = getElapsedTime_us(identifier);
    __disable_irq();
    start_tick[identifier] = __HAL_TIM_GET_COUNTER(htim_);
    start_overflow_count[identifier] = overflow_count_;
    __enable_irq();
    return elapsed;
}