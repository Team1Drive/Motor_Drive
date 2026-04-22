#include "pwm3phase_timer.h"
//#include <cmath>
#include <cstdint>

ThreePhasePWMOut::ThreePhasePWMOut(TIM_HandleTypeDef* htim):
    htim_(htim) {
        duty_A = 0.0f;
        duty_B = 0.0f;
        duty_C = 0.0f;
        enabled_A = false;
        enabled_B = false;
        enabled_C = false;
    }

void ThreePhasePWMOut::updateCompareValues(void) {
    // Get current auto-reload value
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim_);
    uint32_t max_compare = period;  // For 100% duty, set compare = period

    // Compute compare values with saturation
    uint32_t compare_A = (uint32_t)(duty_A * (period + 1U));
    if (compare_A > max_compare) compare_A = max_compare;
    uint32_t compare_B = (uint32_t)(duty_B * (period + 1U));
    if (compare_B > max_compare) compare_B = max_compare;
    uint32_t compare_C = (uint32_t)(duty_C * (period + 1U));
    if (compare_C > max_compare) compare_C = max_compare;

    // Update compare registers for each channel
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, compare_A);
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, compare_B);
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, compare_C);
}

HAL_StatusTypeDef ThreePhasePWMOut::init(void) {
    if (HAL_TIM_Base_Start(htim_) != HAL_OK) {
        return HAL_ERROR;
    }
    __HAL_TIM_ENABLE_IT(htim_, TIM_IT_UPDATE);
    return HAL_OK;
}

HAL_StatusTypeDef ThreePhasePWMOut::start(void) {
    if (HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    enabled_A = true;
    if (HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }
    enabled_B = true;
    if (HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_3) != HAL_OK) {
        return HAL_ERROR;
    }
    enabled_C = true;
    return HAL_OK;
}

HAL_StatusTypeDef ThreePhasePWMOut::stop(void) {
    if (HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    enabled_A = false;
    if (HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR;
    }
    enabled_B = false;
    if (HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_3) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_3) != HAL_OK) {
        return HAL_ERROR;
    }
    enabled_C = false;
    return HAL_OK;
}

void ThreePhasePWMOut::setDuty(float dutyA, float dutyB, float dutyC) {
    if (duty_A >= -0.5f && !enabled_A) {
        HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_1);
        enabled_A = true;
    }
    else if (duty_A < -0.5f) {
        HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_1);
        enabled_A = false;
    }
    if (duty_B >= -0.5f && !enabled_B) {
        HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_2);
        enabled_B = true;
    }
    else if (duty_B < -0.5f) {
        HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_2);
        enabled_B = false;
    }
    if (duty_C >= -0.5f && !enabled_C) {
        HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(htim_, TIM_CHANNEL_3);
        enabled_C = true;
    }
    else if (duty_C < -0.5f) {
        HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(htim_, TIM_CHANNEL_3);
        enabled_C = false;
    }

    // Clamp duty cycle inputs to [0.0, 1.0]
    duty_A = std::max(0.0f, std::min(1.0f, dutyA));
    duty_B = std::max(0.0f, std::min(1.0f, dutyB));
    duty_C = std::max(0.0f, std::min(1.0f, dutyC));

    // Get current auto-reload value
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim_);
    uint32_t max_compare = period;  // For 100% duty, set compare = period

    // Compute compare values with saturation
    uint32_t compare_A = (uint32_t)(duty_A * (period + 1U));
    if (compare_A > max_compare) compare_A = max_compare;
    uint32_t compare_B = (uint32_t)(duty_B * (period + 1U));
    if (compare_B > max_compare) compare_B = max_compare;
    uint32_t compare_C = (uint32_t)(duty_C * (period + 1U));
    if (compare_C > max_compare) compare_C = max_compare;

    // Update compare registers for each channel
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_1, compare_A);
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, compare_B);
    __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, compare_C);
}

float ThreePhasePWMOut::getDuty(uint8_t phase) const {
    switch (phase) {
        case 0:
            return duty_A;
            break;
        case 1:
            return duty_B;
            break;
        case 2:
            return duty_C;
            break;
        default:
            return -1.0f; // Invalid phase
    }
    return -1.0f; // Invalid phase
}

HAL_StatusTypeDef ThreePhasePWMOut::setDeadTime(uint32_t deadtime_ns) {
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    if (LL_RCC_GetAPB2Prescaler() != LL_RCC_APB2_DIV_1) pclk2 *= 2;

    float tick_ns = 1e9f / pclk2;  // seconds per timer tick
    if ((uint32_t)ceilf(deadtime_ns / tick_ns) > 1008) return HAL_ERROR; // Desired dead time exceeds maximum representable value

    uint32_t dtg = __LL_TIM_CALC_DEADTIME(pclk2, LL_TIM_GetClockDivision(htim_->Instance), deadtime_ns);
    LL_TIM_OC_SetDeadTime(htim_->Instance, dtg);
    
    return HAL_OK;
}

HAL_StatusTypeDef ThreePhasePWMOut::setFrequency(uint32_t freq_Hz) {
    if (!htim_ || freq_Hz == 0) return HAL_ERROR;

    pwm_freq_Hz = freq_Hz;
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();
    if (LL_RCC_GetAPB2Prescaler() != LL_RCC_APB2_DIV_1) timer_clock *= 2;
    uint32_t psc;
    uint32_t arr;
    if (LL_TIM_GetCounterMode(htim_->Instance) != LL_TIM_COUNTERMODE_UP && LL_TIM_GetCounterMode(htim_->Instance) != LL_TIM_COUNTERMODE_DOWN) {
        timer_clock /= 2;
        psc = 0;
        arr = (uint32_t)(timer_clock / pwm_freq_Hz);
        if (arr > 0xFFFF) {
            // Case for ARR overflow: increase prescaler to bring ARR within 16-bit range
            psc = arr / 0xFFFF;
            arr = (timer_clock / (pwm_freq_Hz * (psc + 1U)));
        }
    }
    else {
        psc = 0;
        arr = (uint32_t)(timer_clock / pwm_freq_Hz) - 1U;
        if (arr > 0xFFFF) {
            // Case for ARR overflow: increase prescaler to bring ARR within 16-bit range
            psc = arr / 0xFFFF;
            arr = (timer_clock / (pwm_freq_Hz * (psc + 1U))) - 1U;
        }
    }
    if (arr < 1) arr = 1;
    __HAL_TIM_SET_PRESCALER(htim_, psc);
    __HAL_TIM_SET_AUTORELOAD(htim_, arr);

    updateCompareValues();

    return HAL_OK;
}

uint32_t ThreePhasePWMOut::getFrequency(void) const {
    return pwm_freq_Hz;
}
