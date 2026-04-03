#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include <cstring>

class ADCSampler {
    private:
        ADC_HandleTypeDef* hadc_;
        DMA_HandleTypeDef* hdma_;
        volatile uint16_t* buffer_;
        uint32_t length_;
        uint32_t half_len_;
        uint32_t num_channels_;
        static ADCSampler* instance_[3];
        volatile uint32_t latest_group_;

        uint16_t* proc_buffer_;
        uint32_t  proc_len_;
        bool      use_proc_buffer_;
        bool      data_ready_;

        /**
         * Helper function to determine the instance index based on the ADC handle. This is used to route the correct ADC handle to the corresponding ADCSampler instance in the static callback functions.
         * @note This function is for mapping ADC1, ADC2, and ADC3 to indices 0, 1, and 2 respectively. It returns -1 for invalid handles. Internal use only.
         */
        static uint32_t getInstanceIndex(ADC_HandleTypeDef* hadc);

        /**
         * Process the 2nd half buffer when the conversion complete callback is triggered. This function should be called from the ADC conversion complete interrupt handler. It copies the data from the DMA buffer to the processing buffer if one is set, and sets the full_ready_ flag to indicate that new data is available for processing.
         */
        void processBuffer(void);

        /**
         * Process the half buffer when the conversion half complete callback is triggered. This function should be called from the ADC conversion half complete interrupt handler. It copies the first half of the data from the DMA buffer to the processing buffer if one is set, and sets the half_ready_ flag to indicate that new data is available for processing.
         */
        void processHalfBuffer(void);

    public:
        volatile bool half_ready_;
        volatile bool full_ready_;

        ADCSampler(ADC_HandleTypeDef* hadc, DMA_HandleTypeDef* hdma, volatile uint16_t* buffer, uint32_t length);

        /**
         * Static callback function to be called from the ADC conversion complete interrupt handler. This function identifies which ADC instance triggered the interrupt and calls the corresponding instance's processBuffer method to handle the new data.
          * @param hadc Pointer to the ADC handle that triggered the conversion complete interrupt.
         */
        static void irqConvCplt(ADC_HandleTypeDef* hadc);

        /**
         * Static callback function to be called from the ADC conversion half complete interrupt handler. This function identifies which ADC instance triggered the interrupt and calls the corresponding instance's processHalfBuffer method to handle the new data.
         * @param hadc Pointer to the ADC handle that triggered the conversion half complete interrupt.
         */
        static void irqConvHalfCplt(ADC_HandleTypeDef* hadc);

        /**
         * Start ADC without DMA.
         * @return HAL status code indicating success or failure of starting the ADC.
         * @note Not to be called when using DMA, use startDMA() instead.
         */
        HAL_StatusTypeDef startADC(void);

        /**
         * Start ADC with DMA.
         * @return HAL status code indicating success or failure of starting the ADC with DMA.
         */
        HAL_StatusTypeDef startDMA(void);

        /**
         * Set an optional processing buffer where the ADC data will be copied before being marked as ready. This allows for double buffering, where the DMA can continue filling the main buffer while the processing buffer is being used for computations. The processing buffer should have enough space to hold at least half of the DMA buffer length to accommodate the data copied during the half complete callback.
         * @param proc_buf Pointer to the processing buffer where ADC data will be copied.
         * @param proc_len Length of the processing buffer. It should be same as or larger than the DMA buffer length to ensure it can hold the data copied.
         */
        void setProcessingBuffer(uint16_t* proc_buf, uint32_t proc_len);

        /**
         * Get the latest ADC data for each channel. The channel_data array should have enough space to hold the number of channels configured in the ADC. The function copies the latest ADC values for each channel into the provided array.
         * @param channel_data Pointer to an array where the latest ADC values for each channel will be copied. The caller is responsible for ensuring that this array has enough space to hold the number of channels configured in the ADC.
         */
        void getLatestData(uint16_t* data_ptr);

        /**
         * Get the latest ADC value for a specific channel. The channel index should be zero-based and less than the number of channels configured in the ADC. The function returns the latest ADC value for the specified channel.
         * @param channel The zero-based index of the channel for which to retrieve the latest ADC value. This index should be less than the number of channels configured in the ADC.
         * @return The latest ADC value for the specified channel.
         */
        uint16_t getLatestChannel(uint8_t channel);

        /**
         * Get the latest ADC values for a specific channel over multiple samples. The channel index should be zero-based and less than the number of channels configured in the ADC. The function copies the latest ADC values for the specified channel into the provided array. The set_length parameter specifies how many of the most recent samples to copy, and should not exceed the length of the DMA buffer divided by the number of channels.
         * @param channel The zero-based index of the channel for which to retrieve the latest ADC values. This index should be less than the number of channels configured in the ADC.
         * @param data_ptr Pointer to an array where the latest ADC values for the specified channel will be copied. The caller is responsible for ensuring that this array has enough space to hold the number of samples specified by set_length.
         * @param set_length The number of the most recent samples to copy for the specified channel.
         * @attention The caller is responsible for ensuring that `data_ptr` has enough space to hold the number of samples specified by `set_length`.
         */
        void getLatestChannel(uint8_t channel, uint16_t* data_ptr, uint32_t length);

        /**
         * Get the average of the latest ADC values for a specific channel over multiple samples. The channel index should be zero-based and less than the number of channels configured in the ADC. The function retrieves the latest ADC values for the specified channel and computes their average using a fast method that avoids overflow. The length parameter specifies how many of the most recent samples to include in the average, and should be a power of 2 for the averaging method to work correctly.
         * @param channel The zero-based index of the channel for which to retrieve the average ADC value. This index should be less than the number of channels configured in the ADC.
         * @param length The number of the most recent samples to include in the average.
         * @return The average of the latest ADC values for the specified channel over the specified number of samples.
         * @attention The `length` parameter should be a power of 2 for the averaging method to work correctly. The function will return 0 if `length` is not a power of 2 or if the channel index is invalid.
         */
        uint16_t getLatestChannel(uint8_t channel, uint32_t length);
};