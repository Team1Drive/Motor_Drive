#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include "parameters.h"


class RollingMax {
    private:
        float buffer[LOG_MAX_VALUE_WINDOW_SIZE];
        float max_value;
        uint16_t index;
        
    public:
        RollingMax();

        void newValue(float value);

        float getMax() const;
};

/**
 * @brief Converts a raw ADC value to a voltage in volts, applying the reference voltage, resolution, gain, and offset for calibration.
 * @param raw The raw ADC value to be converted.
 * @param vref The reference voltage for the ADC.
 * @param resolution The resolution of the ADC (e.g., 4096 for 12-bit, 65536 for 16-bit).
 * @param gain The gain factor for the ADC.
 * @param offset The offset factor for the ADC.
 * @return The converted voltage in volts.
 */
float adcToVoltage(uint32_t raw, float vref, uint32_t resolution, float gain, float offset);

/**
 * @brief Converts a raw ADC value to a current in amps, using the voltage conversion and applying the shunt resistance for current measurement.
 * @param raw The raw ADC value to be converted.
 * @param vref The reference voltage for the ADC.
 * @param resolution The resolution of the ADC (e.g., 4096 for 12-bit, 65536 for 16-bit).
 * @param gain The gain factor for the ADC.
 * @param offset The offset factor for the ADC.
 * @param shunt The shunt resistance in ohms used for current measurement.
 * @return The converted current in amps.
 */
float adcToCurrent(uint32_t raw, float vref, uint32_t resolution, float gain, float offset, float shunt);

/**
 * @brief Computes the average of an array of uint16_t values using a fast method that avoids overflow. The function sums all values in a uint32_t variable and then right shifts by the number of bits corresponding to the size of the array (assuming size is a power of 2) to get the average.
 * @param data_ptr Pointer to the array of uint16_t values.
 * @param size The number of elements in the array (must be a power of 2).
 * @return The average value as a uint16_t.
 * @attention `size` must be a power of 2, otherwise 0 will be returned.
 */
uint16_t fastAverage(uint16_t* data_ptr, uint16_t size);

/**
 * @brief Checks if a given uint16_t value is a power of 2.
 * @param x The value to check.
 * @return true if x is a power of 2, false otherwise.
 */
bool isPowerOfTwo(uint16_t x);