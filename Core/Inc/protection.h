#pragma once

#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_dac.h"
#include "stm32h7xx_ll_comp.h"
#include "parameters.h"
#include <cstdint>

class DAC {
    private:
        DAC_HandleTypeDef* hdac_;
        uint32_t channel_;
    public:
        DAC(DAC_HandleTypeDef* hdac, uint32_t channel);

        /**
         * @brief Start the DAC channel
         * @return HAL status code indicating success or failure
         */
        HAL_StatusTypeDef start(void);

        /**
         * @brief Stop the DAC channel
         * @return HAL status code indicating success or failure
         */
        HAL_StatusTypeDef stop(void);

        /**
         * @brief Set the DAC output value
         * @param output 12-bit value to set the DAC output to (e.g., 0-4095 for 12-bit resolution)
         */
        HAL_StatusTypeDef setOutput(uint32_t output);

        /**
         * @brief Get the current DAC output value
         * @return The current 12-bit DAC output value
         */
        uint32_t getOutput(void) const;
};

class COMP {
    private:
        COMP_HandleTypeDef* hcomp_;
    public:
        COMP(COMP_HandleTypeDef* hcomp);

        /**
         * @brief Start the comparator
         * @return HAL status code indicating success or failure
         */
        HAL_StatusTypeDef start(void);

        /**
         * @brief Stop the comparator
         * @return HAL status code indicating success or failure
         */
        HAL_StatusTypeDef stop(void);

        /**
         * @brief Set the comparator output polarity
         * @param polarity This parameter can be one of the following values: LL_COMP_OUTPUTPOL_NONINVERTED LL_COMP_OUTPUTPOL_INVERTED
         */
        void setPolarity(uint32_t polarity);

        /**
         * @brief Get the current comparator output polarity
         * @return The current polarity setting (e.g., LL_COMP_OUTPUTPOL_NONINVERTED or LL_COMP_OUTPUTPOL_INVERTED)
         */
        uint32_t getPolarity(void) const;
};

class COMP_Protection {
    private:
        COMP comp;
        DAC dac;

    public:
        COMP_Protection(COMP_HandleTypeDef* hcomp, DAC_HandleTypeDef* hdac, uint32_t dac_channel);
        
        /**
         * @brief Start the comparator-based protection mechanism
         * @return HAL status code indicating success or failure
         */
        HAL_StatusTypeDef start(void);

        /**
         * @brief Stop the comparator-based protection mechanism
         * @return HAL status code indicating success or failure
         */
        HAL_StatusTypeDef stop(void);

        /**
         * @brief Set the protection threshold
         * @param threshold The threshold value for the protection mechanism
         * @return HAL status code indicating success or failure
         */
        HAL_StatusTypeDef setThreshold(uint32_t threshold);

        /**
         * @brief Set the comparator output polarity
         * @param polarity This parameter can be one of the following values: LL_COMP_OUTPUTPOL_NONINVERTED LL_COMP_OUTPUTPOL_INVERTED
         */
        void setPolarity(uint32_t polarity);

        /**
         * @brief Get the current protection threshold
         * @return The current threshold value for the protection mechanism
         */
        uint32_t getThreshold(void) const;

        /**
         * @brief Get the current comparator output polarity
         * @return The current polarity setting (e.g., LL_COMP_OUTPUTPOL_NONINVERTED or LL_COMP_OUTPUTPOL_INVERTED)
         */
        uint32_t getPolarity(void) const;
};

class AWD_Protection {
    private:
        ADC_HandleTypeDef* hadc_;
        uint32_t watchdog_;
        uint32_t threshold_;
        uint32_t resolution_;

    public:
        AWD_Protection(ADC_HandleTypeDef* hadc, uint32_t watchdog);

        /**
         * @brief Start the ADC-based protection mechanism
         * @return HAL status code indicating success or failure
         */
        void start(void);

        /**
         * @brief Stop the ADC-based protection mechanism
         * @return HAL status code indicating success or failure
         */
        void stop(void);

        /**
         * @brief Set the upper protection threshold
         * @param threshold The threshold value for the protection mechanism
         */
        void setUpperThreshold(uint32_t threshold);

        /**
         * @brief Set the lower protection threshold
         * @param threshold The threshold value for the protection mechanism
         */
        void setLowerThreshold(uint32_t threshold);

        /**
         * @brief Get the current upper protection threshold
         * @return The current upper threshold value for the protection mechanism
         */
        uint32_t getUpperThreshold(void) const;

        /**
         * @brief Get the current lower protection threshold
         * @return The current lower threshold value for the protection mechanism
         */
        uint32_t getLowerThreshold(void) const;
};