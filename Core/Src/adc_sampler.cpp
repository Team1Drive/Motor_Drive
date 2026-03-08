#include "adc_sampler.h"
//#include <cmath>
#include <cstdint>

ADCSampler* ADCSampler::instance_[3] = {nullptr, nullptr, nullptr};

uint32_t ADCSampler::getInstanceIndex(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) return 0;
    if (hadc->Instance == ADC2) return 1;
    if (hadc->Instance == ADC3) return 2;
    return -1; // Invalid case
}

void ADCSampler::processBuffer(void) {
    // Process the full buffer (second half)
    if (use_proc_buffer_) {
        // Invalidate data cache to ensure latest date is read
        SCB_InvalidateDCache_by_Addr((uint32_t *)(buffer_ + half_len_), half_len_ * sizeof(uint16_t));
        // Copy the second half of the buffer to the processing buffer
        memcpy(proc_buffer_ + half_len_, buffer_ + half_len_, half_len_ * sizeof(uint16_t));
    }
    full_ready_ = true;
}

void ADCSampler::processHalfBuffer(void) {
    // Process the half buffer (first half)
    if (use_proc_buffer_) {
        // Invalidate data cache to ensure latest date is read
        SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_, half_len_ * sizeof(uint16_t));
        // Copy the first half of the buffer to the processing buffer
        memcpy(proc_buffer_, buffer_, half_len_ * sizeof(uint16_t));
    }
    half_ready_ = true;
}

ADCSampler::ADCSampler(ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma, uint16_t* buffer, uint32_t length):
    hadc_(hadc),
    hdma_(hdma),
    buffer_(buffer),
    length_(length),
    half_len_(length / 2),
    latest_group_(0),
    half_ready_(false),
    full_ready_(false) {
        num_channels_ = hadc_->Init.NbrOfConversion;
        if (num_channels_ == 0) num_channels_ = 1;
        uint32_t i = getInstanceIndex(hadc_);
        instance_[i] = this;
    }

void ADCSampler::setProcessingBuffer(uint16_t* proc_buf, uint32_t proc_len) {
    proc_buffer_ = proc_buf;
    proc_len_ = proc_len;
    use_proc_buffer_ = true;
}

void ADCSampler::irqConvCplt(ADC_HandleTypeDef* hadc) {
    uint32_t i = getInstanceIndex(hadc);
    if (instance_[i] != nullptr) {
        instance_[i]->processBuffer();
    }
}

void ADCSampler::irqConvHalfCplt(ADC_HandleTypeDef* hadc) {
    uint32_t i = getInstanceIndex(hadc);
    if (instance_[i] != nullptr) {
        instance_[i]->processHalfBuffer();
    }
}

HAL_StatusTypeDef ADCSampler::startADC(void) {
    return HAL_ADC_Start(hadc_);
}

HAL_StatusTypeDef ADCSampler::startDMA(void) {
    return HAL_ADC_Start_DMA(hadc_, (uint32_t*)buffer_, length_);
}

void ADCSampler::getLatestData(uint16_t* channel_data) {
    // Calculate how many samples have been written by checking the DMA NDTR register
    __disable_irq();
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(hdma_);
    __enable_irq();

    // Calculate the index of the latest complete group of samples
    uint32_t written = length_ - ndtr;
    if (written == 0) written = length_;
    uint32_t latest_index = written - 1;
    uint32_t group_start = (latest_index / num_channels_) * num_channels_;

    // Copy the latest group of samples for each channel
    for (uint32_t i = 0; i < num_channels_; i++) {
        channel_data[i] = buffer_[group_start + i];
    }
}