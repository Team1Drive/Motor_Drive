#include "coupled_comm.h"
#include <cstdint>

CoupledCommunication* CoupledCommunication::instance_ = nullptr;

namespace {
    constexpr uint32_t simulationTableLength(void) {
        return sizeof(coupled_simulation_table) / sizeof(coupled_simulation_table[0]);
    }

    bool isMasterState(CouplingMode mode) {
        switch (mode) {
            case CouplingMode::COUPLING_MASTER:
            case CouplingMode::COUPLING_MASTER_PARING:
            case CouplingMode::COUPLING_MASTER_STARTUP:
            case CouplingMode::COUPLING_MASTER_READY:
            case CouplingMode::COUPLING_MASTER_RUN:
            case CouplingMode::COUPLING_MASTER_FINISH:
            case CouplingMode::COUPLING_MASTER_LOST:
                return true;

            default:
                return false;
        }
    }

    bool isErrorState(CouplingMode mode) {
        return mode == CouplingMode::COUPLING_ERROR_CALLBACK
            || mode == CouplingMode::COUPLING_ERROR_ARRAY
            || mode == CouplingMode::COUPLING_ERROR;
    }

    bool shouldPreserveSignalTimeout(CouplingMode mode) {
        return mode == CouplingMode::COUPLING_MASTER_PARING
            || mode == CouplingMode::COUPLING_MASTER_READY;
    }
}

void CoupledCommunication::resetReception(IncomingSignalType signal) {
    receive_event_counter_ = 0;
    receiving_signal_ = signal;
}

void CoupledCommunication::sendSignal(IncomingSignalType signal) {
    outgoing_signal_ = signal;
    switch (signal) {
        case IncomingSignalType::SIGNAL_LOW:
            stopClock();
            tx_state_ = GPIO_PIN_RESET;
            tx_.write(GPIO_PIN_RESET);
            break;

        case IncomingSignalType::SIGNAL_HIGH:
            stopClock();
            tx_state_ = GPIO_PIN_SET;
            tx_.write(GPIO_PIN_SET);
            break;

        case IncomingSignalType::SIGNAL_WAVE:
            startClock(TRANSMISSION_FREQ_HZ);
            break;

        case IncomingSignalType::SIGNAL_RUN:
            startClock(SIMULATION_FREQ_HZ);
            break;

        case IncomingSignalType::SIGNAL_NONE: default:
            break;
    }
}

void CoupledCommunication::enterMode(CouplingMode mode) {
    bool preserve_signal_timeout = timeout_purpose_ == TimeoutPurpose::TIMEOUT_SIGNAL
                                && shouldPreserveSignalTimeout(mode);

    coupling_mode_ = mode;
    resetReception();
    if (!preserve_signal_timeout) {
        stopTimeout();
    }
    startup_control_allowed_ = false;
    finish_wave_seen_ = false;

    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
        case CouplingMode::COUPLING_MASTER:
        case CouplingMode::COUPLING_SLAVE:
            sendSignal(IncomingSignalType::SIGNAL_HIGH);
            break;

        case CouplingMode::COUPLING_MASTER_READY:
            sendSignal(IncomingSignalType::SIGNAL_HIGH);
            break;

        case CouplingMode::COUPLING_SLAVE_READY:
            sendSignal(IncomingSignalType::SIGNAL_HIGH);
            break;

        case CouplingMode::COUPLING_MASTER_PARING:
            sendSignal(IncomingSignalType::SIGNAL_WAVE);
            break;

        case CouplingMode::COUPLING_SLAVE_PARING:
            sendSignal(IncomingSignalType::SIGNAL_WAVE);
            break;

        case CouplingMode::COUPLING_SLAVE_STARTUP:
            sendSignal(IncomingSignalType::SIGNAL_WAVE);
            startStartupWatchdog();
            break;

        case CouplingMode::COUPLING_MASTER_STARTUP:
            sendSignal(IncomingSignalType::SIGNAL_LOW);
            break;

        case CouplingMode::COUPLING_MASTER_RUN:
            running_step_ = 0;
            error_reported_ = false;
            sendSignal(IncomingSignalType::SIGNAL_RUN);
            break;

        case CouplingMode::COUPLING_SLAVE_RUN:
            running_step_ = 0;
            error_reported_ = false;
            sendSignal(IncomingSignalType::SIGNAL_HIGH);
            startSimulationTimeout();
            break;

        case CouplingMode::COUPLING_MASTER_FINISH:
            sendSignal(IncomingSignalType::SIGNAL_WAVE);
            break;

        case CouplingMode::COUPLING_SLAVE_FINISH:
            sendSignal(IncomingSignalType::SIGNAL_HIGH);
            break;

        case CouplingMode::COUPLING_MASTER_LOST:
        case CouplingMode::COUPLING_SLAVE_LOST:
            errorHandler();
            break;

        case CouplingMode::COUPLING_ERROR_CALLBACK:
        case CouplingMode::COUPLING_ERROR_ARRAY:
        case CouplingMode::COUPLING_ERROR:
            stopClock();
            sendSignal(IncomingSignalType::SIGNAL_LOW);
            break;

        default:
            break;
    }
}

bool CoupledCommunication::applySimulationStep(void) {
    if (callbacks_.applyStep == nullptr) {
        enterMode(CouplingMode::COUPLING_ERROR_CALLBACK);
        return false;
    }

    if (simulationTableLength() == 0U) {
        enterMode(CouplingMode::COUPLING_ERROR_ARRAY);
        return false;
    }

    if (running_step_ >= simulationTableLength()) return false;

    const CoupledSimulationStep_t& step = coupled_simulation_table[running_step_];
    if (coupling_mode_ == CouplingMode::COUPLING_MASTER_RUN) {
        callbacks_.applyStep(step.master_speed, step.master_torque);
        return true;
    }
    else if (coupling_mode_ == CouplingMode::COUPLING_SLAVE_RUN) {
        callbacks_.applyStep(step.slave_speed, step.slave_torque);
        return true;
    }

    return false;
}

bool CoupledCommunication::checkControlStart(bool master) {
    startup_control_allowed_ = false;

    if (callbacks_.canStart == nullptr
     || callbacks_.start == nullptr
     || callbacks_.applyStep == nullptr
     || callbacks_.stop == nullptr) {
        enterMode(CouplingMode::COUPLING_ERROR_CALLBACK);
        return false;
    }

    if (callbacks_.canStart() != CoupledControlResult::OK) {
        if (master) {
            enterMode(CouplingMode::COUPLING_MASTER);
        }
        else {
            enterMode(CouplingMode::COUPLING_SLAVE_LOST);
        }
        return false;
    }

    startup_control_allowed_ = true;
    return true;
}

bool CoupledCommunication::startControl(bool master) {
    if (!startup_control_allowed_ || callbacks_.start == nullptr) {
        enterMode(CouplingMode::COUPLING_ERROR_CALLBACK);
        return false;
    }

    if (!callbacks_.start(master)) {
        if (master) {
            enterMode(CouplingMode::COUPLING_MASTER);
        }
        else {
            enterMode(CouplingMode::COUPLING_SLAVE_LOST);
        }
        return false;
    }

    simulation_active_ = true;
    startup_control_allowed_ = false;
    return true;
}

bool CoupledCommunication::requestControlStart(bool master) {
    if (!checkControlStart(master)) return false;
    return startControl(master);
}

void CoupledCommunication::handleClock(void) {
    tx_state_ = tx_state_ == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET;
    tx_.write(tx_state_);

    if (coupling_mode_ != CouplingMode::COUPLING_MASTER_RUN) return;

    if (rx_state_ != GPIO_PIN_SET) {
        enterMode(CouplingMode::COUPLING_MASTER_LOST);
        return;
    }

    if (running_step_ < simulationTableLength()) {
        if (!applySimulationStep()) return;
        running_step_++;
        if (running_step_ >= simulationTableLength()) {
            stopClock();
            startSimulationFinishDelay();
        }
    }
    else {
        stopClock();
        startSimulationFinishDelay();
    }
}

void CoupledCommunication::handleTimeout(void) {
    TimeoutPurpose expired_timeout = timeout_purpose_;
    stopTimeout();

    switch (expired_timeout) {
        case TimeoutPurpose::TIMEOUT_SIGNAL: {
            IncomingSignalType stable_signal =
                rx_state_ == GPIO_PIN_SET ? IncomingSignalType::SIGNAL_HIGH : IncomingSignalType::SIGNAL_LOW;
            resetReception(stable_signal);
            handleRxSignal();
            break;
        }

        case TimeoutPurpose::TIMEOUT_STARTUP_WATCHDOG:
            if (isMasterState(coupling_mode_)) {
                enterMode(CouplingMode::COUPLING_MASTER_LOST);
            }
            else {
                enterMode(CouplingMode::COUPLING_SLAVE_LOST);
            }
            break;

        case TimeoutPurpose::TIMEOUT_SIMULATION:
            if (coupling_mode_ == CouplingMode::COUPLING_SLAVE_RUN) {
                if (running_step_ >= simulationTableLength()) {
                    enterMode(CouplingMode::COUPLING_SLAVE_FINISH);
                }
                else {
                    enterMode(CouplingMode::COUPLING_SLAVE_LOST);
                }
            }
            break;

        case TimeoutPurpose::TIMEOUT_SIMULATION_FINISH_DELAY:
            if (coupling_mode_ == CouplingMode::COUPLING_MASTER_RUN
             && running_step_ >= simulationTableLength()) {
                enterMode(CouplingMode::COUPLING_MASTER_FINISH);
            }
            else {
                enterMode(CouplingMode::COUPLING_MASTER_LOST);
            }
            break;

        case TimeoutPurpose::TIMEOUT_NONE: default:
            break;
        }
}

void CoupledCommunication::distinguishSignal(void) {
    if (coupling_mode_ == CouplingMode::COUPLING_SLAVE_RUN) {
        if (!applySimulationStep()) enterMode(CouplingMode::COUPLING_SLAVE_LOST);
        return;
    }

    receive_event_counter_++;

    if (receive_event_counter_ >= WAVE_CNT_THRESHOLD
     && receiving_signal_ != IncomingSignalType::SIGNAL_WAVE) {
        receiving_signal_ = IncomingSignalType::SIGNAL_WAVE;
        CouplingMode previous_mode = coupling_mode_;
        handleRxSignal();

        if (coupling_mode_ != previous_mode) {
            return;
        }
    }

    startSignalTimeout();
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
            break;
    }
}

void CoupledCommunication::handleLow(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_SLAVE:
            enterMode(CouplingMode::COUPLING_SLAVE_STARTUP);
            break;

        case CouplingMode::COUPLING_DEFAULT:
            enterMode(CouplingMode::COUPLING_DEFAULT);
            break;

        case CouplingMode::COUPLING_MASTER_LOST:
        case CouplingMode::COUPLING_SLAVE_LOST:
            break;

        default:
            if (coupling_mode_ < CouplingMode::COUPLING_MASTER_LOST) {
                enterMode(CouplingMode::COUPLING_MASTER_LOST);
            }
            else {
                enterMode(CouplingMode::COUPLING_SLAVE_LOST);
            }
            break;
    }
}

void CoupledCommunication::handleHigh(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
            enterMode(CouplingMode::COUPLING_SLAVE_PARING);
            break;

        case CouplingMode::COUPLING_MASTER_PARING:
            enterMode(CouplingMode::COUPLING_MASTER);
            break;

        case CouplingMode::COUPLING_MASTER_READY:
            enterMode(CouplingMode::COUPLING_MASTER_RUN);
            break;

        case CouplingMode::COUPLING_MASTER_STARTUP:
            enterMode(CouplingMode::COUPLING_MASTER_LOST);
            break;

        case CouplingMode::COUPLING_MASTER_FINISH:
            if (!finish_wave_seen_) {
                enterMode(CouplingMode::COUPLING_MASTER_LOST);
                break;
            }
            if (simulation_active_ && callbacks_.stop != nullptr) {
                callbacks_.stop(false);
                simulation_active_ = false;
            }
            enterMode(CouplingMode::COUPLING_MASTER);
            break;

        case CouplingMode::COUPLING_SLAVE_PARING:
            enterMode(CouplingMode::COUPLING_SLAVE);
            break;

        case CouplingMode::COUPLING_SLAVE_STARTUP:
            if (!requestControlStart(false)) {
                enterMode(CouplingMode::COUPLING_SLAVE_LOST);
            }
            else {
                enterMode(CouplingMode::COUPLING_SLAVE_READY);
            }
            break;

        case CouplingMode::COUPLING_SLAVE_FINISH:
            if (!finish_wave_seen_) {
                enterMode(CouplingMode::COUPLING_SLAVE_LOST);
                break;
            }
            if (simulation_active_ && callbacks_.stop != nullptr) {
                callbacks_.stop(false);
                simulation_active_ = false;
            }
            enterMode(CouplingMode::COUPLING_SLAVE);
            break;

        case CouplingMode::COUPLING_MASTER_LOST:
        case CouplingMode::COUPLING_SLAVE_LOST:
            break;

        default:
            break;
    }
}

void CoupledCommunication::handleWave(void) {
    switch (coupling_mode_) {
        case CouplingMode::COUPLING_DEFAULT:
        case CouplingMode::COUPLING_MASTER:
            enterMode(CouplingMode::COUPLING_MASTER_PARING);
            break;

        case CouplingMode::COUPLING_MASTER_PARING:
            break;

        case CouplingMode::COUPLING_MASTER_STARTUP:
            enterMode(CouplingMode::COUPLING_MASTER_READY);
            break;

        case CouplingMode::COUPLING_MASTER_FINISH:
            finish_wave_seen_ = true;
            sendSignal(IncomingSignalType::SIGNAL_HIGH);
            break;

        case CouplingMode::COUPLING_SLAVE_PARING:
            sendSignal(IncomingSignalType::SIGNAL_HIGH);
            break;

        case CouplingMode::COUPLING_SLAVE_FINISH:
            finish_wave_seen_ = true;
            sendSignal(IncomingSignalType::SIGNAL_WAVE);
            break;

        default:
            break;
    }
}

void CoupledCommunication::handleRun(void) {
    if (coupling_mode_ != CouplingMode::COUPLING_SLAVE_RUN) return;

    if (simulationTableLength() == 0U) {
        enterMode(CouplingMode::COUPLING_ERROR_ARRAY);
        return;
    }

    if (running_step_ < simulationTableLength()) {
        if (!applySimulationStep()) return;
        running_step_++;
    }
    else {
        enterMode(CouplingMode::COUPLING_SLAVE_LOST);
    }
}

void CoupledCommunication::handleNone(void) {
    if (coupling_mode_ == CouplingMode::COUPLING_DEFAULT) {
        enterMode(CouplingMode::COUPLING_DEFAULT);
    }
}

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
    timeout_(htim_timeout),
    rx_state_(GPIO_PIN_RESET),
    tx_state_(GPIO_PIN_SET),
    coupling_mode_(CouplingMode::COUPLING_DEFAULT),
    receiving_signal_(IncomingSignalType::SIGNAL_NONE),
    timeout_purpose_(TimeoutPurpose::TIMEOUT_NONE),
    receive_event_counter_(0),
    running_step_(0),
    simulation_active_(false),
    startup_control_allowed_(false),
    error_reported_(false),
    finish_wave_seen_(false) {
        instance_ = this;
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
    tx_state_ = GPIO_PIN_SET;
    tx_.write(tx_state_);
    rx_state_ = rx_.read() ? GPIO_PIN_SET : GPIO_PIN_RESET;
    clock_.setFrequency(TRANSMISSION_FREQ_HZ);
    timeout_.setFrequency(TRANSMISSION_TIMEOUT_HZ);

    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_TIM_Base_Start(htim_clk_) == HAL_OK ? status : HAL_ERROR;
    status = HAL_TIM_Base_Start(htim_timeout_) == HAL_OK ? status : HAL_ERROR;
    stopClock();
    stopTimeout();
    return status;
}

bool CoupledCommunication::startSimulation(void) {
    if (coupling_mode_ != CouplingMode::COUPLING_MASTER) return false;
    if (simulationTableLength() == 0U) {
        enterMode(CouplingMode::COUPLING_ERROR_ARRAY);
        return false;
    }
    if (!requestControlStart(true)) return false;

    enterMode(CouplingMode::COUPLING_MASTER_STARTUP);
    return true;
}

void CoupledCommunication::reset(void) {
    if (isErrorState(coupling_mode_)) return;

    running_step_ = 0;
    error_reported_ = false;
    finish_wave_seen_ = false;
    tx_state_ = GPIO_PIN_SET;
    tx_.write(tx_state_);
    clock_.setFrequency(TRANSMISSION_FREQ_HZ);
    timeout_.setFrequency(TRANSMISSION_TIMEOUT_HZ);
    stopClock();
    stopTimeout();
    enterMode(CouplingMode::COUPLING_DEFAULT);
}
