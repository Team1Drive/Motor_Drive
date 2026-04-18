#pragma once

#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_rcc.h"
#include <cstdint>

class Timer {
    private:
        TIM_HandleTypeDef* htim_;
        static Timer* instance_[16];
        uint32_t frequency;

        static uint32_t getInstanceIndex(TIM_HandleTypeDef* htim);

        void updateFrequency(void);

        void interruptHandler(void);

    public:
        Timer(TIM_HandleTypeDef* htim);

        static void irqHandler(TIM_HandleTypeDef* htim);

        HAL_StatusTypeDef init(void);

        HAL_StatusTypeDef start(void);

        HAL_StatusTypeDef startIT(void);

        HAL_StatusTypeDef setFrequency(uint32_t freq_Hz);

        uint32_t getFrequency(void);
};