#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include "digitalio.h"
#include "timer.h"

#define TRANSMISSION_FREQ_HZ    5000
#define TRANSMISSION_TIMEOUT_MS 10
#define TRANSMISSION_TIMEOUT_HZ (1000 / TRANSMISSION_TIMEOUT_MS)
#define WAVE_CNT_THRESHOLD      5 // Number of edge transitions within the timeout period to consider the signal as a wave
#define SIMULATION_FREQ_HZ      10
#define SIMULATION_TIMEOUT_MS   500
#define SIMULATION_TIMEOUT_HZ   (1000 / SIMULATION_TIMEOUT_MS)

enum class CouplingMode : uint8_t {
    COUPLING_DEFAULT        = 0,

    COUPLING_MASTER         = 1,
    COUPLING_MASTER_PARING  = 2,
    COUPLING_MASTER_STARTUP = 3,
    COUPLING_MASTER_READY   = 4,
    COUPLING_MASTER_RUN     = 5,
    COUPLING_MASTER_FINISH  = 6,
    COUPLING_MASTER_LOST    = 63,

    COUPLING_SLAVE          = 127,
    COUPLING_SLAVE_PARING   = 126,
    COUPLING_SLAVE_STARTUP  = 125,
    COUPLING_SLAVE_READY    = 124,
    COUPLING_SLAVE_RUN      = 123,
    COUPLING_SLAVE_FINISH   = 122,
    COUPLING_SLAVE_LOST     = 64
};

enum class IncomingSignalType : uint8_t {
    SIGNAL_LOW  = 0,
    SIGNAL_HIGH = 1,
    SIGNAL_WAVE = 2,
    SIGNAL_RUN  = 3,

    SIGNAL_NONE = 255
};

class CoupledCommunication {
    private:
        static CoupledCommunication* instance_;
        DigitalOut tx_;
        DigitalIn rx_;
        TIM_HandleTypeDef* htim_clk_;
        TIM_HandleTypeDef* htim_timeout_;
        Timer clock_;
        Timer timeout_;

        GPIO_PinState rx_state_;
        CouplingMode coupling_mode_;
        IncomingSignalType receiving_signal_;
        uint32_t receive_event_counter_;

        void (*callback_)(float speed, float torque, bool error) = nullptr;
        uint32_t running_step_;

        inline void rxRising(void) {
            rx_state_ = GPIO_PIN_SET;
            distinguishSignal();
        }

        inline void rxFalling(void) {
            rx_state_ = GPIO_PIN_RESET;
            distinguishSignal();
        }

        void handleClock(void);

        void handleTimeout(void);

        inline void startClock(TIM_HandleTypeDef* htim) {
            __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
        }

        inline void stopClock(TIM_HandleTypeDef* htim) {
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
        }

        inline void startTimeout(uint32_t timeout_ms) {
            timeout_.setFrequency(1000 / timeout_ms);
            __HAL_TIM_SET_COUNTER(htim_timeout_, 0);
            __HAL_TIM_ENABLE_IT(htim_timeout_, TIM_IT_UPDATE);
        }

        inline void resetTimeout(void) {
            __HAL_TIM_SET_COUNTER(htim_timeout_, 0);
        }

        inline void stopTimeout(void) {
            __HAL_TIM_DISABLE_IT(htim_timeout_, TIM_IT_UPDATE);
        }

        void distinguishSignal(void);

        void sendSignal(IncomingSignalType signal);

        void handleRxSignal(void);

        void handleLow(void);

        void handleHigh(void);
    
        void handleWave(void);

        void handleRun(void);

        void handleNone(void);

        inline void errorHandler(void) {
            sendSignal(IncomingSignalType::SIGNAL_LOW);
            callback_(0.0f, 0.0f, true);
        }

    public:

        CoupledCommunication(GPIO_TypeDef* tx_port,
                             uint16_t tx_pin,
                             GPIO_TypeDef* rx_port,
                             uint16_t rx_pin,
                             TIM_HandleTypeDef* htim_clk,
                             TIM_HandleTypeDef* htim_timeout);

        static void irqHandlerRxRising(void);

        static void irqHandlerRxFalling(void);

        static void irqHandlerClock(void);

        static void irqHandlerTimeout(void);

        HAL_StatusTypeDef init(void);

        inline CouplingMode getCouplingMode(void) const {
            return coupling_mode_;
        }

        inline void setRunnningCallback(void (*callback)(float speed, float torque, bool error)) {
            callback_ = callback;
        }

        void run(void);
};