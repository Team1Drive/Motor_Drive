#include "protection.h"

/* ----------------------------- DAC ----------------------------- */
DAC::DAC(DAC_HandleTypeDef* hdac, uint32_t channel) : hdac_(hdac), channel_(channel) {}

HAL_StatusTypeDef DAC::start(void) {
    return HAL_DAC_Start(hdac_, channel_);
}

HAL_StatusTypeDef DAC::stop(void) {
    return HAL_DAC_Stop(hdac_, channel_);
}

HAL_StatusTypeDef DAC::setOutput(uint32_t output) {
    return HAL_DAC_SetValue(hdac_, channel_, DAC_ALIGN_12B_R, output);
}

uint32_t DAC::getOutput(void) const {
    return LL_DAC_RetrieveOutputData(hdac_->Instance, channel_);
}

/* ----------------------------- COMP ----------------------------- */
COMP::COMP(COMP_HandleTypeDef* hcomp) : hcomp_(hcomp) {}

HAL_StatusTypeDef COMP::start(void) {
    return HAL_COMP_Start(hcomp_);
}

HAL_StatusTypeDef COMP::stop(void) {
    return HAL_COMP_Stop(hcomp_);
}

void COMP::setPolarity(uint32_t polarity) {
    LL_COMP_SetOutputPolarity(hcomp_->Instance, polarity);
}

uint32_t COMP::getPolarity(void) const {
    return LL_COMP_GetOutputPolarity(hcomp_->Instance);
}

/* ----------------------------- COMP_Protection ----------------------------- */
COMP_Protection::COMP_Protection(COMP_HandleTypeDef* hcomp, DAC_HandleTypeDef* hdac, uint32_t dac_channel):
    comp(hcomp),
    dac(hdac, dac_channel) {
        comp.setPolarity(LL_COMP_OUTPUTPOL_NONINVERTED);
    }

HAL_StatusTypeDef COMP_Protection::start(void) {
    HAL_StatusTypeDef status = HAL_OK;
    if (dac.start() != HAL_OK) status = HAL_ERROR;
    if (comp.start() != HAL_OK) status = HAL_ERROR;
    return status;
}

HAL_StatusTypeDef COMP_Protection::stop(void) {
    HAL_StatusTypeDef status = HAL_OK;
    if (comp.stop() != HAL_OK) status = HAL_ERROR;
    if (dac.stop() != HAL_OK) status = HAL_ERROR;
    return status;
}

HAL_StatusTypeDef COMP_Protection::setThreshold(uint32_t threshold) {
    return dac.setOutput(threshold);
}

void COMP_Protection::setPolarity(uint32_t polarity) {
    comp.setPolarity(polarity);
}

uint32_t COMP_Protection::getThreshold(void) const {
    return dac.getOutput();
}

uint32_t COMP_Protection::getPolarity(void) const {
    return comp.getPolarity();
}

/* ----------------------------- AWD_Protection ----------------------------- */
AWD_Protection::AWD_Protection(ADC_HandleTypeDef* hadc, uint32_t watchdog) : hadc_(hadc), watchdog_(watchdog) {
    resolution_ = LL_ADC_GetResolution(hadc_->Instance);
}

void AWD_Protection::start(void) {
    switch (watchdog_) {
        case ADC_ANALOGWATCHDOG_1:
            __HAL_ADC_ENABLE_IT(hadc_, ADC_IT_AWD1);
            break;
        case ADC_ANALOGWATCHDOG_2:
            __HAL_ADC_ENABLE_IT(hadc_, ADC_IT_AWD2);
            break;
        case ADC_ANALOGWATCHDOG_3:
            __HAL_ADC_ENABLE_IT(hadc_, ADC_IT_AWD3);
            break;
        default:
            break;
    }
}

void AWD_Protection::stop(void) {
    switch (watchdog_) {
        case ADC_ANALOGWATCHDOG_1:
            __HAL_ADC_DISABLE_IT(hadc_, ADC_IT_AWD1);
            break;
        case ADC_ANALOGWATCHDOG_2:
            __HAL_ADC_DISABLE_IT(hadc_, ADC_IT_AWD2);
            break;
        case ADC_ANALOGWATCHDOG_3:
            __HAL_ADC_DISABLE_IT(hadc_, ADC_IT_AWD3);
            break;
        default:
            break;
    }
}

void AWD_Protection::setUpperThreshold(uint32_t threshold) {
    switch (watchdog_) {
        case ADC_ANALOGWATCHDOG_1:
            LL_ADC_SetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_HIGH, __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(resolution_, threshold));
            break;
        case ADC_ANALOGWATCHDOG_2:
            LL_ADC_SetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD2, LL_ADC_AWD_THRESHOLD_HIGH, __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(resolution_, threshold));
            break;
        case ADC_ANALOGWATCHDOG_3:
            LL_ADC_SetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD3, LL_ADC_AWD_THRESHOLD_HIGH, __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(resolution_, threshold));
            break;
        default:
            break;
    }
}

void AWD_Protection::setLowerThreshold(uint32_t threshold) {
    switch (watchdog_) {
        case ADC_ANALOGWATCHDOG_1:
            LL_ADC_SetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_LOW, __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(resolution_, threshold));
            break;
        case ADC_ANALOGWATCHDOG_2:
            LL_ADC_SetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD2, LL_ADC_AWD_THRESHOLD_LOW, __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(resolution_, threshold));
            break;
        case ADC_ANALOGWATCHDOG_3:
            LL_ADC_SetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD3, LL_ADC_AWD_THRESHOLD_LOW, __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(resolution_, threshold));
            break;
        default:
            break;
    }
}

uint32_t AWD_Protection::getUpperThreshold(void) const {
    switch (watchdog_) {
        case ADC_ANALOGWATCHDOG_1:
            return __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION(resolution_, LL_ADC_GetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_HIGH));
        case ADC_ANALOGWATCHDOG_2:
            return __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION(resolution_, LL_ADC_GetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD2, LL_ADC_AWD_THRESHOLD_HIGH));
        case ADC_ANALOGWATCHDOG_3:
            return __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION(resolution_, LL_ADC_GetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD3, LL_ADC_AWD_THRESHOLD_HIGH));
        default:
            return 0;
    }
}

uint32_t AWD_Protection::getLowerThreshold(void) const {
    switch (watchdog_) {
        case ADC_ANALOGWATCHDOG_1:
            return __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION(resolution_, LL_ADC_GetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD1, LL_ADC_AWD_THRESHOLD_LOW));
        case ADC_ANALOGWATCHDOG_2:
            return __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION(resolution_, LL_ADC_GetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD2, LL_ADC_AWD_THRESHOLD_LOW));
        case ADC_ANALOGWATCHDOG_3:
            return __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION(resolution_, LL_ADC_GetAnalogWDThresholds(hadc_->Instance, LL_ADC_AWD3, LL_ADC_AWD_THRESHOLD_LOW));
        default:
            return 0;
    }
}