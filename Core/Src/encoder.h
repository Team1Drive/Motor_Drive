#pragma once

#include "stm32h7xx_hal.h"
#include "ustimer.h"
#include "parameters.h"
#include <cstdint>

class Encoder {
    private:
        static Encoder* instance_;
        uint16_t pinA_, pinB_, indexPin_;
        MicrosecondTimer& timer_;
        uint32_t count;
        uint8_t state, p_state;
        uint32_t pulse_interval, p_pulse_interval;

        int8_t direction_decode(void);

        void chA_rise(void);

        void chA_fall(void);

        void chB_rise(void);

        void chB_fall(void);

        void index_rise(void);

        void index_fall(void);

    public:
        Encoder(uint16_t chA, uint16_t chB, uint16_t index, MicrosecondTimer& timer);

        void init(void);

        static void irqHandlerRising(uint16_t pin);

        static void irqHandlerFalling(uint16_t pin);

        void reset(void);

        uint32_t getCount(void);

        uint8_t getDirection(void);

        float getRPM(void);

        float getPos_deg(void);

        float getPos_rad(void);
};