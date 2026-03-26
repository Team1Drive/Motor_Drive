#include "timer.h"
//#include <cmath>
#include <cstdint>

Timer* Timer::instance_[16] = {nullptr};

uint32_t Timer::getInstanceIndex(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1) return 0;
    if (htim->Instance == TIM2) return 1;
    if (htim->Instance == TIM3) return 2;
    if (htim->Instance == TIM4) return 3;
    if (htim->Instance == TIM5) return 4;
    if (htim->Instance == TIM6) return 5;
    if (htim->Instance == TIM7) return 6;
    if (htim->Instance == TIM8) return 7;
    if (htim->Instance == TIM12) return 8;
    if (htim->Instance == TIM13) return 9;
    if (htim->Instance == TIM14) return 10;
    if (htim->Instance == TIM15) return 11;
    if (htim->Instance == TIM16) return 12;
    if (htim->Instance == TIM17) return 13;
    if (htim->Instance == TIM23) return 14;
    if (htim->Instance == TIM24) return 15;
    return -1; // Invalid case
}

void Timer::interruptHandler(void) {
    // Handle timer interrupt (if needed)
}

Timer::Timer(TIM_HandleTypeDef* htim):
    htim_(htim) {
        uint32_t i = getInstanceIndex(htim_);
        instance_[i] = this;
    }

void Timer::irqHandler(TIM_HandleTypeDef* htim) {
    uint32_t i = getInstanceIndex(htim);
    if (instance_[i] != nullptr) {
        instance_[i]->interruptHandler();
    }
}

HAL_StatusTypeDef Timer::init(void) {
    // Initialize the timer (if needed)
    return HAL_OK;
}

HAL_StatusTypeDef Timer::start(void) {
    return HAL_TIM_Base_Start(htim_);
}

HAL_StatusTypeDef Timer::startIT(void) {
    return HAL_TIM_Base_Start_IT(htim_);
}

HAL_StatusTypeDef Timer::setFrequency(uint32_t freq_Hz) {
    if (!htim_ || freq_Hz == 0) return HAL_ERROR;

    // Get timer clock frequency, distinguishing APB2 for TIM1/TIM8 and APB1 for others
    uint32_t timer_clock = 0;
    TIM_TypeDef* tim = htim_->Instance;
    if (tim == TIM1 || tim == TIM8) timer_clock = HAL_RCC_GetPCLK2Freq();
    else timer_clock = HAL_RCC_GetPCLK1Freq();

    // Distinguish 32-bit timers (TIM2, TIM5)
    uint32_t max_arr = 0xFFFF;
    if (tim == TIM2 || tim == TIM5) max_arr = 0xFFFFFFFF;

    uint32_t psc = 0;
    uint32_t arr = 0;

    uint64_t product = timer_clock / freq_Hz;
    if (product == 0) return HAL_ERROR;

    if (product <= max_arr) {
        // No need for prescaler
        arr = (uint32_t)product - 1;
        psc = 0;
    } else {
        arr = max_arr;
        psc = (uint32_t)(product / (arr + 1)) - 1;
        if (psc > 0xFFFF) {
            arr = (uint32_t)(product / 0x10000) - 1;
            psc = 0xFFFF;
        }
    }

    if (psc > 0xFFFF) psc = 0xFFFF;
    if (arr > max_arr) arr = max_arr;

    __HAL_TIM_DISABLE(htim_);
    __HAL_TIM_SET_PRESCALER(htim_, psc);
    __HAL_TIM_SET_AUTORELOAD(htim_, arr);
    __HAL_TIM_ENABLE(htim_);

    htim_->Init.Prescaler = psc;
    htim_->Init.Period = arr;

    return HAL_OK;
}