#include "math_helpers.h"
//#include <cmath>
#include <cstdint>


RollingMax::RollingMax() : max_value(0.0f), index(0) {
    for (int i = 0; i < LOG_MAX_VALUE_WINDOW_SIZE; i++) {
        buffer[i] = 0.0f;
    }
}

void RollingMax::newValue(float value) {
    float p_value = buffer[index];
    buffer[index] = value;
    index++;
    if (index >= LOG_MAX_VALUE_WINDOW_SIZE) index = 0;

    if (value >= max_value) max_value = value;
    else if (p_value == max_value) {
        max_value = buffer[0];
        for (uint16_t i = 1; i < LOG_MAX_VALUE_WINDOW_SIZE; i++) {
            if (buffer[i] > max_value) max_value = buffer[i];
        }
    }
}

float RollingMax::getMax() const {
    return max_value;
}


float adcToVoltage(uint32_t raw, float vref, uint32_t resolution, float gain, float offset) {
  return (((float)raw / (float)resolution) * vref - offset) / gain;
}

float adcToCurrent(uint32_t raw, float vref, uint32_t resolution, float gain, float offset, float shunt) {
  float voltage = adcToVoltage(raw, vref, resolution, gain, offset);
  return voltage / shunt;
}

uint16_t fastAverage(uint16_t* data_ptr, uint16_t size) {
  if (size == 0) return 0; // Avoid division by zero
  if (!isPowerOfTwo(size)) return 0; // Size must be a power of 2 for this method to work correctly

  uint32_t sum = 0;
  for (uint16_t i = 0; i < size; i++) {
    sum += data_ptr[i];
  }

  uint32_t shift = 31 - __builtin_clz(size);

  return (uint16_t)(sum >> shift);
}

bool isPowerOfTwo(uint16_t x) {
  return (x != 0) && ((x & (x - 1)) == 0);
}