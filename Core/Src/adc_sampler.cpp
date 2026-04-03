#include "adc_sampler.h"
//#include <cmath>
#include <cstdint>

extern uint16_t fastAverage(uint16_t* data_ptr, uint16_t size);

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
        memcpy(proc_buffer_ + half_len_, (uint16_t*)buffer_ + half_len_, half_len_ * sizeof(uint16_t));
    }
    full_ready_ = true;
}

void ADCSampler::processHalfBuffer(void) {
    // Process the half buffer (first half)
    if (use_proc_buffer_) {
        // Invalidate data cache to ensure latest date is read
        SCB_InvalidateDCache_by_Addr((uint32_t *)buffer_, half_len_ * sizeof(uint16_t));
        // Copy the first half of the buffer to the processing buffer
        memcpy(proc_buffer_, (uint16_t*)buffer_, half_len_ * sizeof(uint16_t));
    }
    half_ready_ = true;

    if (!data_ready_) {
        data_ready_ = true;
    }
}

ADCSampler::ADCSampler(ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma, volatile uint16_t* buffer, uint32_t length):
    hadc_(hadc),
    hdma_(hdma),
    buffer_(buffer),
    length_(length),
    half_len_(length / 2),
    latest_group_(0),
    half_ready_(false),
    full_ready_(false) {
        data_ready_ = false;
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
    uint32_t i = getInstanceIndex(hadc_);
    if (i != (uint32_t)-1) instance_[i] = this;
    if (HAL_ADC_Start_IT(hadc_) != HAL_OK) return HAL_ERROR;
    num_channels_ = hadc_->Init.NbrOfConversion;
    if (num_channels_ == 0) num_channels_ = 1;
    return HAL_OK;
}

HAL_StatusTypeDef ADCSampler::startDMA(void) {
    uint32_t i = getInstanceIndex(hadc_);
    if (i != (uint32_t)-1) instance_[i] = this;
    if (HAL_ADC_Start_DMA(hadc_, (uint32_t*)buffer_, length_) != HAL_OK) return HAL_ERROR;
    num_channels_ = hadc_->Init.NbrOfConversion;
    if (num_channels_ == 0) num_channels_ = 1;
    return HAL_OK;
}

void ADCSampler::getLatestData(uint16_t* data_ptr) {
    // Return zeros before the first half full DMA interrupt to prevent processing invalid data
    if (!data_ready_) {
        for (uint32_t i = 0; i < num_channels_; i++) {
            data_ptr[i] = 0;
        }
        return;
    }

    // Calculate how many samples have been written by checking the DMA NDTR register
    __disable_irq();
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(hdma_);
    __enable_irq();

    // Calculate the index of the latest complete group of samples
    uint32_t written = length_ - ndtr;
    uint32_t written_groups = written / num_channels_;
    if (written_groups == 0) written_groups = length_ / num_channels_;
    written_groups--;
    uint32_t group_start = written_groups * num_channels_;

    // Copy the latest group of samples for each channel
    for (uint32_t i = 0; i < num_channels_; i++) {
        data_ptr[i] = buffer_[group_start + i];
    }
}

void ADCSampler::getLatestData(uint16_t* data_ptr, uint32_t set_length) {
    // Return zeros before the first half full DMA interrupt to prevent processing invalid data
    if (!data_ready_) {
        for (uint32_t i = 0; i < set_length * num_channels_; i++) {
            data_ptr[i] = 0;
        }
        return;
    }

    // Calculate how many samples have been written by checking the DMA NDTR register
    __disable_irq();
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(hdma_);
    __enable_irq();

    // Calculate the index of the latest complete group of samples
    uint32_t written = length_ - ndtr;
    uint32_t written_groups = written / num_channels_;
    if (written_groups == 0) written_groups = length_ / num_channels_;
    written_groups--;
    uint32_t group_start = written_groups * num_channels_;

    // Copy the latest group of samples for each channel
    int32_t buffer_index = group_start;
    for (uint32_t i = 0; i < set_length; i++) {
        for (uint32_t j = 0; j < num_channels_; j++) {
                data_ptr[i * num_channels_ + j] = buffer_[buffer_index + j];
        }
        buffer_index -= num_channels_;
        if (buffer_index < 0) buffer_index += length_;
    }
}

void ADCSampler::getLatestDataMean(uint16_t* data_ptr, uint32_t set_length) {
    uint16_t data[set_length * num_channels_];
    getLatestData(data, set_length);
    for (uint32_t i = 0; i < num_channels_; i++) {
        uint16_t channel_data[set_length];
        for (uint32_t j = 0; j < set_length; j++) {
            channel_data[j] = data[i + j * num_channels_];
        }
        data_ptr[i] = fastAverage(channel_data, set_length);
    }
}

uint16_t ADCSampler::getLatestChannel(uint8_t channel) {
    if (channel >= num_channels_) return 0; // Invalid channel index
    uint16_t data[num_channels_];
    getLatestData(data);
    return data[channel];
}

void ADCSampler::getLatestChannel(uint8_t channel, uint16_t* data_ptr, uint32_t set_length) {
    if (channel >= num_channels_) return; // Invalid channel index
    // Return zeros before the first half full DMA interrupt to prevent processing invalid data
    if (!data_ready_) {
        for (uint32_t i = 0; i < set_length; i++) {
            data_ptr[i] = 0;
        }
        return;
    }

    // Calculate how many samples have been written by checking the DMA NDTR register
    __disable_irq();
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(hdma_);
    __enable_irq();

    // Calculate the index of the latest complete group of samples
    uint32_t written = length_ - ndtr;
    uint32_t written_groups = written / num_channels_;
    if (written_groups == 0) written_groups = length_ / num_channels_;
    written_groups--;
    uint32_t group_start = written_groups * num_channels_;

    // Copy the latest group of samples for each channel
    int32_t buffer_index = group_start + channel;
    for (uint32_t i = 0; i < set_length; i++) {
        data_ptr[i] = buffer_[buffer_index];
        buffer_index -= num_channels_;
        if (buffer_index < 0) buffer_index += length_;
    }
}

uint16_t ADCSampler::getLatestChannelMean(uint8_t channel, uint32_t set_length) {
    if (channel >= num_channels_) return 0; // Invalid channel index
    uint16_t data[set_length];
    getLatestChannel(channel, data, set_length);
    return fastAverage(data, set_length);
}