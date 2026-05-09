#include "coupled_comm.h"
//#include <cmath>
#include <cstdint>

CoupledCommunication* CoupledCommunication::instance_ = nullptr;

CoupledCommunication::CoupledCommunication(GPIO_TypeDef* tx_port,
                                           uint16_t tx_pin,
                                           GPIO_TypeDef* rx_port,
                                           uint16_t rx_pin,
                                           TIM_HandleTypeDef* htim_clk,
                                           TIM_HandleTypeDef* htim_timeout):
    tx_(tx_port, tx_pin),
    rx_(rx_port, rx_pin),
    htim_clk_(htim_clk),
    htim_timeout_(htim_timeout),
    clock_(htim_clk),
    timeout_(htim_timeout) {
        instance_ = this;
}

void CoupledCommunication::handleClock(void) {

}

void CoupledCommunication::handleTimeout(void) {
    if (coupling_mode_ == CouplingMode::COUPLING_SLAVE_RUN) {

    }
    else {
        receive_event_counter_ = 0;
        receiving_signal_ = rx_state_ == GPIO_PIN_SET ? IncomingSignalType::SIGNAL_HIGH : IncomingSignalType::SIGNAL_LOW;
        handleRxSignal();
    }
    stopTimeout();
}

void CoupledCommunication::distinguishSignal(void) {
    if (coupling_mode_ == CouplingMode::COUPLING_SLAVE_RUN) {

        startTimeout(SIMULATION_TIMEOUT_MS);
    }
    else {
        receive_event_counter_++;
        if (receive_event_counter_ >= WAVE_CNT_THRESHOLD && receiving_signal_ != IncomingSignalType::SIGNAL_WAVE) {
            receiving_signal_ = IncomingSignalType::SIGNAL_WAVE;
            handleRxSignal();
        }
        startTimeout(TRANSMISSION_TIMEOUT_MS);
    }
}

void CoupledCommunication::handleRxSignal(void) {
    switch (receiving_signal_) {
        case IncomingSignalType::SIGNAL_LOW:
            handleLow();
            break;

        case IncomingSignalType::SIGNAL_HIGH:
            handleHigh();
            break;

        case IncomingSignalType::SIGNAL_WAVE:
            handleWave();
            break;

        case IncomingSignalType::SIGNAL_RUN:
            handleRun();
            break;

        case IncomingSignalType::SIGNAL_NONE: default:
            handleNone();
    }
}

void CoupledCommunication::handleLow(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
            coupling_mode_ = CouplingMode::COUPLING_DEFAULT;
            break;

        case CouplingMode::COUPLING_MASTER:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_LOST;
            break;
        case CouplingMode::COUPLING_MASTER_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_LOST;
            break;
        case CouplingMode::COUPLING_MASTER_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_LOST;
            break;
        case CouplingMode::COUPLING_MASTER_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_LOST;
            break;
        case CouplingMode::COUPLING_MASTER_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_LOST;
            break;
        case CouplingMode::COUPLING_MASTER_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_LOST;
            break;
        case CouplingMode::COUPLING_MASTER_LOST:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_LOST;
            break;
            

        case CouplingMode::COUPLING_SLAVE:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_STARTUP;
            break;
        case CouplingMode::COUPLING_SLAVE_PARING:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_LOST;
            break;
        case CouplingMode::COUPLING_SLAVE_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_LOST;
            break;
        case CouplingMode::COUPLING_SLAVE_READY:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_LOST;
            break;
        case CouplingMode::COUPLING_SLAVE_RUN:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_LOST;
            break;
        case CouplingMode::COUPLING_SLAVE_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_LOST;
            break;
        case CouplingMode::COUPLING_SLAVE_LOST:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_LOST;
            break;

        default:
            break;
    }
}

void CoupledCommunication::handleHigh(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_PARING;
            break;

        case CouplingMode::COUPLING_MASTER:
            coupling_mode_ = CouplingMode::COUPLING_MASTER;
            break;
        case CouplingMode::COUPLING_MASTER_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER;
            break;
        case CouplingMode::COUPLING_MASTER_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_STARTUP;
            break;
        case CouplingMode::COUPLING_MASTER_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_RUN;
            break;
        case CouplingMode::COUPLING_MASTER_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_FINISH;
            break;
        case CouplingMode::COUPLING_MASTER_LOST:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_PARING;
            break;
            

        case CouplingMode::COUPLING_SLAVE:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE;
            break;
        case CouplingMode::COUPLING_SLAVE_PARING:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_PARING;
            break;
        case CouplingMode::COUPLING_SLAVE_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_READY:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_RUN:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_LOST;
            break;
        case CouplingMode::COUPLING_SLAVE_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE;
            break;
        case CouplingMode::COUPLING_SLAVE_LOST:
            coupling_mode_ = CouplingMode::COUPLING_SLAVE_PARING;
            break;

        default:
            break;
    }
}

void CoupledCommunication::handleWave(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_PARING;
            break;

        case CouplingMode::COUPLING_MASTER:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_LOST:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
            

        case CouplingMode::COUPLING_SLAVE:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_LOST:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;

        default:
            break;
    }
}

void CoupledCommunication::handleRun(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_PARING;
            break;

        case CouplingMode::COUPLING_MASTER:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_LOST:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
            

        case CouplingMode::COUPLING_SLAVE:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_LOST:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;

        default:
            break;
    }
}

void CoupledCommunication::handleNone(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_PARING;
            break;

        case CouplingMode::COUPLING_MASTER:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_MASTER_LOST:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
            

        case CouplingMode::COUPLING_SLAVE:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_PARING:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_STARTUP:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_READY:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_RUN:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_FINISH:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;
        case CouplingMode::COUPLING_SLAVE_LOST:
            coupling_mode_ = CouplingMode::COUPLING_MASTER_READY;
            break;

        default:
            break;
    }
}

void CoupledCommunication::irqHandlerRxRising(void) {
    if (instance_ != nullptr) {
        instance_->rxRising();
    }
}

void CoupledCommunication::irqHandlerRxFalling(void) {
    if (instance_ != nullptr) {
        instance_->rxFalling();
    }
}

void CoupledCommunication::irqHandlerClock(void) {
    if (instance_ != nullptr) {
        instance_->handleClock();
    }
}

void CoupledCommunication::irqHandlerTimeout(void) {
    if (instance_ != nullptr) {
        instance_->handleTimeout();
    }
}

HAL_StatusTypeDef CoupledCommunication::init(void) {
    rx_state_ = GPIO_PIN_RESET;
    coupling_mode_ = CouplingMode::COUPLING_DEFAULT;

    tx_.write(1);
    clock_.setFrequency(TRANSMISSION_FREQ_HZ);
    timeout_.setFrequency(TRANSMISSION_TIMEOUT_HZ);

    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_TIM_Base_Start(htim_clk_) == HAL_OK ? status : HAL_ERROR;
    status = HAL_TIM_Base_Start(htim_timeout_) == HAL_OK ? status : HAL_ERROR;
    return status;
}