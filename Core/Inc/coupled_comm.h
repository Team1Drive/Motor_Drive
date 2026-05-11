#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include "digitalio.h"
#include "timer.h"

#define TRANSMISSION_FREQ_HZ    5000
#define TRANSMISSION_TIMEOUT_MS 10
#define TRANSMISSION_TIMEOUT_HZ (1000 / TRANSMISSION_TIMEOUT_MS)
#define TRANSMISSION_WATCHDOG_MS    50
#define TRANSMISSION_WATCHDOG_HZ    (1000 / TRANSMISSION_WATCHDOG_MS)
#define WAVE_CNT_THRESHOLD      5 // Number of edge transitions within the timeout period to consider the signal as a wave
#define SIMULATION_FREQ_HZ      10
#define SIMULATION_TIMEOUT_MS   500
#define SIMULATION_TIMEOUT_HZ   (1000 / SIMULATION_TIMEOUT_MS)
#define SIMULATION_FINISH_DELAY_MS 1000
#define SIMULATION_FINISH_DELAY_HZ (1000 / SIMULATION_FINISH_DELAY_MS)

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
    COUPLING_SLAVE_LOST     = 64,

    COUPLING_ERROR_CALLBACK = 253,
    COUPLING_ERROR_ARRAY    = 254,
    COUPLING_ERROR          = 255
};

enum class IncomingSignalType : uint8_t {
    SIGNAL_LOW,
    SIGNAL_HIGH,
    SIGNAL_WAVE,
    SIGNAL_RUN,

    SIGNAL_NONE = 255
};

enum class TimeoutPurpose : uint8_t {
    TIMEOUT_NONE = 0,
    TIMEOUT_SIGNAL,
    TIMEOUT_TRANSMISSION_WATCHDOG,
    TIMEOUT_SIMULATION,
    TIMEOUT_SIMULATION_FINISH_DELAY
};

typedef struct {
    float master_speed;
    float master_torque;
    float slave_speed;
    float slave_torque;
} CoupledSimulationStep_t;

enum class CoupledControlResult : uint8_t {
    OK,
    BUSY,
    NOT_READY,
    PROTECTION
};

struct CoupledSimulationCallbacks {
    CoupledControlResult (*canStart)(void);
    bool (*start)(bool master);
    void (*applyStep)(float speed, float torque);
    void (*stop)(bool fault);
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
        GPIO_PinState tx_state_;
        CouplingMode coupling_mode_;
        IncomingSignalType receiving_signal_;
        TimeoutPurpose timeout_purpose_;
        uint32_t receive_event_counter_;


        CoupledSimulationCallbacks callbacks_ = {
            .canStart = nullptr,
            .start = nullptr,
            .applyStep = nullptr,
            .stop = nullptr
        };

        uint32_t running_step_;
        bool simulation_active_;
        bool startup_control_allowed_;
        bool error_reported_;
        bool finish_wave_seen_;

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

        inline void startClock(uint32_t frequency_Hz) {
            clock_.setFrequency(frequency_Hz);
            __HAL_TIM_SET_COUNTER(htim_clk_, 0);
            __HAL_TIM_CLEAR_FLAG(htim_clk_, TIM_FLAG_UPDATE);
            __HAL_TIM_ENABLE_IT(htim_clk_, TIM_IT_UPDATE);
        }

        inline void stopClock(void) {
            __HAL_TIM_DISABLE_IT(htim_clk_, TIM_IT_UPDATE);
        }

        inline void startTimeout(uint32_t frequency_Hz) {
            timeout_.setFrequency(frequency_Hz);
            __HAL_TIM_SET_COUNTER(htim_timeout_, 0);
            __HAL_TIM_CLEAR_FLAG(htim_timeout_, TIM_FLAG_UPDATE);
            __HAL_TIM_ENABLE_IT(htim_timeout_, TIM_IT_UPDATE);
        }

        inline void resetTimeout(void) {
            __HAL_TIM_SET_COUNTER(htim_timeout_, 0);
        }

        inline void stopTimeout(void) {
            __HAL_TIM_DISABLE_IT(htim_timeout_, TIM_IT_UPDATE);
            timeout_purpose_ = TimeoutPurpose::TIMEOUT_NONE;
        }

        inline void startSignalTimeout(void) {
            timeout_purpose_ = TimeoutPurpose::TIMEOUT_SIGNAL;
            startTimeout(TRANSMISSION_TIMEOUT_HZ);
        }

        inline void startTransmissionWatchdog(void) {
            timeout_purpose_ = TimeoutPurpose::TIMEOUT_TRANSMISSION_WATCHDOG;
            startTimeout(TRANSMISSION_WATCHDOG_HZ);
        }

        inline void startSimulationTimeout(void) {
            timeout_purpose_ = TimeoutPurpose::TIMEOUT_SIMULATION;
            startTimeout(SIMULATION_TIMEOUT_HZ);
        }

        inline void startSimulationFinishDelay(void) {
            timeout_purpose_ = TimeoutPurpose::TIMEOUT_SIMULATION_FINISH_DELAY;
            startTimeout(SIMULATION_FINISH_DELAY_HZ);
        }

        void distinguishSignal(void);

        void sendSignal(IncomingSignalType signal);

        void resetReception(IncomingSignalType signal = IncomingSignalType::SIGNAL_NONE);

        void enterMode(CouplingMode mode);

        bool applySimulationStep(void);

        void handleRxSignal(void);

        void handleLow(void);

        void handleHigh(void);
    
        void handleWave(void);

        void handleRun(void);

        void handleNone(void);

        bool requestControlStart(bool master);

        bool checkControlStart(bool master);

        bool startControl(bool master);

        inline void errorHandler(void) {
            sendSignal(IncomingSignalType::SIGNAL_LOW);
            if (simulation_active_ && callbacks_.stop != nullptr && !error_reported_) {
                error_reported_ = true;
                callbacks_.stop(true);
                simulation_active_ = false;
            }
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

        inline void setRunningCallback(void (*callback)(float speed, float torque, bool error)) {
            (void)callback;
        }

        inline void setSimulationCallbacks(CoupledSimulationCallbacks callbacks) {
            callbacks_ = callbacks;
        }

        bool startSimulation(void);

        void reset(void);
};

static constexpr CoupledSimulationStep_t coupled_simulation_table[] = {
    {  500.0f, 0.0f, 0.0f, 0.10f },
    { 1000.0f, 0.0f, 0.0f, 0.20f },
    { 1500.0f, 0.0f, 0.0f, 0.30f },
    { 1000.0f, 0.0f, 0.0f, 0.20f },
    {  500.0f, 0.0f, 0.0f, 0.10f },
    {    0.0f, 0.0f, 0.0f, 0.00f }
};
