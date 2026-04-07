#include "main.h"
#include "parameters.h"
#include "pwm3phase_timer.h"
#include "digitalio.h"
#include "encoder.h"
#include "hallsensor.h"
#include "modulation.h"
#include "ustimer.h"
#include "adc_sampler.h"
#include "timer.h"
#include "foc.h"
#include "cmd.h"
#include "cordic_math.h"
#include <cstdint>

#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>

#define ADC1_BUF_LEN  6144
#define ADC2_BUF_LEN  4096
#define ADC3_BUF_LEN  4096

void MPU_Config(void);

void timer2IRQ(void);
void timer3IRQ(void);
void timer6IRQ(void);

void printTelemetryUTF8(void);
void printTelemetryBinary(void);

void speedControl(void);

void startUpSequence(void);
void alignRotor(void);
void vvvfRampUp(void);
void sixStepCommutation(void);

/* Forward declaration for FOC ISR helper */
static void foc_isr_tick(void);

void test_PWM(void);
void test_PWM_sweep(void);

float adcToVoltage(uint32_t raw, float vref, uint32_t resolution, float gain, float offset);
float adcToCurrent(uint32_t raw, float vref, uint32_t resolution, float gain, float offset, float shunt);
uint16_t fastAverage(uint16_t* data_ptr, uint16_t size);
bool isPowerOfTwo(uint16_t x);

/* Declare ADC buffers */
alignas(32) volatile uint16_t adc1_buffer[ADC1_BUF_LEN] __attribute__((section(".sram_d1")));
alignas(32) volatile uint16_t adc2_buffer[ADC2_BUF_LEN] __attribute__((section(".sram_d1")));
alignas(32) volatile uint16_t adc3_buffer[ADC3_BUF_LEN] __attribute__((section(".sram_d1")));

/* Declare timer handles */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

/* Declare ADC & DMA handles */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

/* Custom Class Objects */
ThreePhasePWMOut motorPWM(&htim8);

MicrosecondTimer usTimer(&htim7);

ADCSampler adc1(&hadc1, &hdma_adc1, adc1_buffer, ADC1_BUF_LEN);
ADCSampler adc2(&hadc2, &hdma_adc2, adc2_buffer, ADC2_BUF_LEN);
ADCSampler adc3(&hadc3, &hdma_adc3, adc3_buffer, ADC3_BUF_LEN);

Timer adcTimer(&htim1), printTimer(&htim2), ledTimer(&htim3), encoderTimer(&htim4), speedControlTimer(&htim6);

DigitalOut pwm_ch1_dis(GPIOA, GPIO_PIN_2), pwm_ch2_dis(GPIOB, GPIO_PIN_2), pwm_ch3_dis(GPIOB, GPIO_PIN_13);
DigitalOut led_red(GPIOC, GPIO_PIN_9), led_green(GPIOA, GPIO_PIN_8), led_yellow_1(GPIOA, GPIO_PIN_9), led_yellow_2(GPIOA, GPIO_PIN_10);
DigitalOut relay(GPIOD, GPIO_PIN_8);
Encoder encoder(&htim4, usTimer, GPIO_PIN_9, ENCODER_PPR, TIM6_FREQ_HZ, ENCODER_STALL_THRESHOLD);
HallSensor hallsensor(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_8, GPIOE, GPIO_PIN_4);

alignas(32) uint16_t adc1_proc_buffer[ADC1_BUF_LEN];
alignas(32) uint16_t adc2_proc_buffer[ADC2_BUF_LEN];
alignas(32) uint16_t adc3_proc_buffer[ADC3_BUF_LEN];

volatile MotorControlMode control_mode = MotorControlMode::MOTOR_STOP;

volatile uint32_t system_flag = 0;
volatile uint32_t error_flag = 0;

volatile ADCGain_t adc_gain = {
    .ia_shunt = ADC_IA_SHUNT,
    .ib_shunt = ADC_IB_SHUNT,
    .ic_shunt = ADC_IC_SHUNT,
    .ia_offset = ADC_IA_OFFSET,
    .ib_offset = ADC_IB_OFFSET,
    .ic_offset = ADC_IC_OFFSET,
    .va_gain = ADC_VA_GAIN,
    .vb_gain = ADC_VB_GAIN,
    .va_offset = ADC_VA_OFFSET,
    .vb_offset = ADC_VB_OFFSET,
    .ibatt_shunt = ADC_IBATT_SHUNT,
    .ibatt_offset = ADC_IBATT_OFFSET,
    .vbatt_gain = ADC_VBATT_GAIN,
    .vbatt_offset = ADC_VBATT_OFFSET
};
volatile Target_t target = { .speed = 0.0f, .torque = 0.0f };

volatile uint32_t print_mask = 0;
volatile PrintFormat print_format = PrintFormat::PRINT_UTF8;

ring_buffer_t rx_ring = { .head = 0, .tail = 0 };

/* FOC state — single global instance */
FOC_State_t foc_state;




// ─────────────────────────────────────────────────────────────────────────────
//  Main Function
// ─────────────────────────────────────────────────────────────────────────────

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_CORDIC_Init();

  /* USER CODE BEGIN 2 */
  led_red.write(1);

  /* Enable USB regulator */
  HAL_PWREx_EnableUSBReg();
  
  /* Start ADC */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) error_flag |= ERROR_ADC_CONFIG;

  if (adc1.startDMA() != HAL_OK) error_flag |= ERROR_DMA_CONFIG;
  if (adc2.startDMA() != HAL_OK) error_flag |= ERROR_DMA_CONFIG;
  if (adc3.startDMA() != HAL_OK) error_flag |= ERROR_DMA_CONFIG;
  
  //while (adc1.startADC() != HAL_OK) usb_printf("Failed to start ADC1 Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc1));
  //while (adc2.startADC() != HAL_OK) usb_printf("Failed to start ADC2 Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc2));
  //while (adc3.startADC() != HAL_OK) usb_printf("Failed to start ADC3 Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc3));
  
  /* Start timers */
  if (adcTimer.start() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;
  if (printTimer.startIT() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;
  if (ledTimer.startIT() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;
  if (encoderTimer.startIT() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;
  if (speedControlTimer.startIT() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;

  if (usTimer.init() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;

  if (encoder.start() != HAL_OK) error_flag |= ERROR_ENCODER_CONFIG;

  /* Start PWM */
  if (motorPWM.init() != HAL_OK) error_flag |= ERROR_PWM_CONFIG;

  /* Enable Caches */
  SCB_EnableICache();
  SCB_EnableDCache();

  usb_printf("HAL Initialized\n");
  
  if (motorPWM.setFrequency(20000) != HAL_OK) error_flag |= ERROR_PWM_CONFIG;
  if (motorPWM.setDeadTime(1000) != HAL_OK) error_flag |= ERROR_PWM_CONFIG;
  
  /* Initialise FOC controller */
  foc_init(&foc_state);
  foc_state.target_rpm = FOC_INITIAL_RPM;
  
  adc1.setProcessingBuffer(adc1_proc_buffer, ADC1_BUF_LEN);
  adc2.setProcessingBuffer(adc2_proc_buffer, ADC2_BUF_LEN);
  adc3.setProcessingBuffer(adc3_proc_buffer, ADC3_BUF_LEN);

  if (!error_flag) led_red.write(0);
  led_green.write(0);
  led_yellow_1.write(0);
  led_yellow_2.write(0);
  
  control_mode = MotorControlMode::MOTOR_STOP;
  
  usb_printf("System Initialized\n");
  /* USER CODE END 2 */
  
  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    char cmd_line[CMD_MAX_LEN];
    if (read_line_from_ring(&rx_ring, cmd_line, CMD_MAX_LEN)) process_command(cmd_line);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}


// ─────────────────────────────────────────────────────────────────────────────
//  System Interrupt Handlers and Callbacks
// ─────────────────────────────────────────────────────────────────────────────

/* GPIO EXTI interrupt callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
    // Handle Hall Channel C (PE4)
    case GPIO_PIN_4:
      if (HAL_GPIO_ReadPin(GPIOE, GPIO_Pin) == GPIO_PIN_SET) {
        HallSensor::irqHandlerRising(GPIO_Pin);
        if (control_mode == MotorControlMode::MOTOR_SIX_STEP) sixStepCommutation();
      } else {
        HallSensor::irqHandlerFalling(GPIO_Pin);
        if (control_mode == MotorControlMode::MOTOR_SIX_STEP) sixStepCommutation();
      }
      break;
    // Hall Channel A (PB5), Hall Channel B (PB8)
    case GPIO_PIN_5: case GPIO_PIN_8:
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_SET) {
        HallSensor::irqHandlerRising(GPIO_Pin);
        if (control_mode == MotorControlMode::MOTOR_SIX_STEP) sixStepCommutation();
      } else {
        HallSensor::irqHandlerFalling(GPIO_Pin);
        if (control_mode == MotorControlMode::MOTOR_SIX_STEP) sixStepCommutation();
      }
      break;
    // Handle Encoder Index (PB9)
    case GPIO_PIN_9:
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_SET) {
        Encoder::irqHandlerIndex(GPIO_Pin);
      }
      break;
    default:
      // Unhandled pin interrupt
      break;
  }
}

/* TIM Period Elapsed callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8) {
    // PWM update interrupt
    if (control_mode == MotorControlMode::MOTOR_FOC_LINEAR ||
        control_mode == MotorControlMode::MOTOR_FOC_DPWM) {
      foc_isr_tick();
    }
    else if (control_mode == MotorControlMode::MOTOR_VVVF) {
      vvvfRampUp();
    }
    else if (control_mode == MotorControlMode::MOTOR_ALIGN) {
      alignRotor();
    }
  }
  else if (htim->Instance == TIM6) {
    // 1 kHz control loop interrupt
    Encoder::irqHandlerSpeed();
    speedControl();
    //timer6IRQ();
    printTelemetryBinary();
  }
  else if (htim->Instance == TIM7) {
    // 1 MHz timer interrupt (Interrupt every 65.536 ms)
    MicrosecondTimer::irqHandler(htim);
  }
  else if (htim->Instance == TIM2) {
    // 10 Hz timer interrupt
    //timer2IRQ();
    printTelemetryUTF8();
  }
  else if (htim->Instance == TIM4) {
    Encoder::irqHandlerOverflow();
  }
  else if (htim->Instance == TIM3) {
    // 2 Hz timer interrupt
    timer3IRQ();
  }
}

/* ADC Conversion HalfComplete callback */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  ADCSampler::irqConvHalfCplt(hadc);
}

/* ADC Conversion Complete callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  ADCSampler::irqConvCplt(hadc);
}

/* USB CDC Receive Handler */
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len) {
  for (uint32_t i = 0; i < Len; i++) {
    ring_buffer_write(&rx_ring, Buf[i]);
  }
}


// ─────────────────────────────────────────────────────────────────────────────
//  Interrupt driven functions (e.g. control loops, status indicators, etc.)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Timer 2 interrupt running at 10 Hz.
 * @note Currently not used.
 */
void timer2IRQ(void) {
  
}

/**
 * @brief Timer 3 interrupt running at 4 Hz.
 * @note Mainly used for status indicators under different control modes.
 */
void timer3IRQ(void) {
  static uint8_t led_increment_counter = 0;
  switch (control_mode) {
    case MotorControlMode::MOTOR_STOP:
      led_green.write(0);
      led_yellow_1.write(0);
      led_yellow_2.write(0);
      break;
    case MotorControlMode::MOTOR_STARTUP:
      led_yellow_1.write(1);
      led_yellow_2.write(1);
      break;
    case MotorControlMode::MOTOR_ALIGN:
       if ((led_increment_counter & 0b010) == 0) {
        led_green.write(1);
        led_yellow_1.write(0);
        led_yellow_2.write(0);
      }
      else {
        led_green.write(0);
        if (led_increment_counter & 0b001) {led_yellow_1.write(0); led_yellow_2.write(1);}
        else {led_yellow_1.write(1); led_yellow_2.write(0);}
      }
      break;
    case MotorControlMode::MOTOR_VVVF:
      if (led_increment_counter >> 1 & 1) {
        led_green.write(1);
      }
      else {
        led_green.write(0);
      }
      break;
    case MotorControlMode::MOTOR_SIX_STEP:
      led_green.toggle();
      if (led_increment_counter >> 1 & 1) {
        led_yellow_1.write(1);
        led_yellow_2.write(0);
      }
      else {
        led_yellow_1.write(0);
        led_yellow_2.write(1);
      }
      break;
    case MotorControlMode::MOTOR_FOC_LINEAR:
      led_green.write(1);
      break;
    case MotorControlMode::MOTOR_FOC_DPWM:
      led_green.write(1);
      led_yellow_2.write(0);
      if (led_increment_counter >> 2 & 1) {
        led_yellow_1.write(1);
      }
      else {
        led_yellow_1.write(0);
      }
      break;
    case MotorControlMode::MOTOR_PROTECTION:
        led_green.write(0);
        led_yellow_1.write(0);
        led_yellow_2.write(0);
        if (error_flag & ERROR_OVERCURRENT) {
          led_red.toggle();
        }
       break;
    default:
      break;
  }

  if ((error_flag & ERROR_OVERCURRENT) == 0) {
    led_red.write(0);
  }
  
  if (led_increment_counter++ >= 7) {
    led_increment_counter = 0;
  }
}

/**
 * @brief Function to be called with TIM6 interrupt.
 * @note Currently not used.
 */
void timer6IRQ(void) {

}

/**
 * @brief Function to print telemetry data in UTF-8 format over USB.
 * @note The data fields to be printed are determined by the print_mask variable, and the format is determined by the print_format variable. The function reads the latest ADC data, processes it into physical units, and constructs a formatted string to be sent over USB.
 */
void printTelemetryUTF8(void) {
  if (print_mask == 0 || (print_format != PrintFormat::PRINT_UTF8)) return;
  
  uint16_t adc1_raw[3];
  uint16_t adc2_raw[2];
  uint16_t adc3_raw[2];
  float ia, ib, ic, va, vb, vbatt, ibatt;
  if ((print_mask & (PRINT_IA
                   | PRINT_VB
                   | PRINT_VBATT
                   | PRINT_IA_RAW
                   | PRINT_VB_RAW
                   | PRINT_VBATT_RAW)) != 0) {
    adc1.getLatestDataMean(adc1_raw, FOC_OVERSAMPLING_SIZE);

    ia = adcToCurrent(adc1_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ia_offset, adc_gain.ia_shunt);
    vb = adcToVoltage(adc1_raw[1], 3.3f, 65536, adc_gain.vb_gain, 1.65f + adc_gain.vb_offset);
    vbatt = adcToVoltage(adc1_raw[2], 3.3f, 65536, adc_gain.vbatt_gain, 0.0f + adc_gain.vbatt_offset);
  }
  if ((print_mask & (PRINT_IB
                   | PRINT_VA
                   | PRINT_IB_RAW
                   | PRINT_VA_RAW)) != 0) {
    adc2.getLatestDataMean(adc2_raw, FOC_OVERSAMPLING_SIZE);

    ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ib_offset, adc_gain.ib_shunt);
    va = adcToVoltage(adc2_raw[1], 3.3f, 65536, adc_gain.va_gain, 1.65f + adc_gain.va_offset);
  }
  if ((print_mask & (PRINT_IC
                   | PRINT_IBATT
                   | PRINT_IC_RAW
                   | PRINT_IBATT_RAW)) != 0) {
    adc3.getLatestDataMean(adc3_raw, FOC_OVERSAMPLING_SIZE);

    ic = adcToCurrent(adc3_raw[0], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ic_offset, adc_gain.ic_shunt);
    ibatt = adcToCurrent(adc3_raw[1], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ibatt_offset, adc_gain.ibatt_shunt);
  }

  // Construct a UTF-8 string, e.g. "rpm 123.45 pos 67.89\r\n"
  char buffer[128];
  int pos = 0;
  if (print_mask & PRINT_HALLBIN) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "hall %u%u%u ", hallsensor.getState() >> 2 & 1, hallsensor.getState() >> 1 & 1, hallsensor.getState() & 1);
  }
  if (print_mask & PRINT_HALLDEC) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "hall_ %u ", hallsensor.getState());
  }
  if (print_mask & PRINT_RPM) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "rpm %.2f ", encoder.getRPM());
  }
  if (print_mask & PRINT_POS) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "pos %u ", encoder.getPos());
  }
  if (print_mask & PRINT_DUTY_A) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "duty_a %.2f ", motorPWM.getDuty(0));
  }
  if (print_mask & PRINT_DUTY_B) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "duty_b %.2f ", motorPWM.getDuty(1));
  }
  if (print_mask & PRINT_DUTY_C) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "duty_c %.2f ", motorPWM.getDuty(2));
  }
  if (print_mask & PRINT_IA) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ia %.5f ", ia);
  }
  if (print_mask & PRINT_IB) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ib %.5f ", ib);
  }
  if (print_mask & PRINT_IC) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ic %.5f ", ic);
  }
  if (print_mask & PRINT_VA) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "va %.5f ", va);
  }
  if (print_mask & PRINT_VB) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "vb %.5f ", vb);
  }
  if (print_mask & PRINT_VBATT) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "vbatt %.5f ", vbatt);
  }
  if (print_mask & PRINT_IBATT) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ibatt %.5f ", ibatt);
  }
  if (print_mask & PRINT_IA_RAW) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ia_ %u ", adc1_raw[0]);
  }
  if (print_mask & PRINT_IB_RAW) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ib_ %u ", adc2_raw[0]);
  }
  if (print_mask & PRINT_IC_RAW) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ic_ %u ", adc3_raw[0]);
  }
  if (print_mask & PRINT_VA_RAW) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "va_ %u ", adc2_raw[1]);
  }
  if (print_mask & PRINT_VB_RAW) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "vb_ %u ", adc1_raw[1]);
  }
  if (print_mask & PRINT_VBATT_RAW) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "vbatt_ %u ", adc1_raw[2]);
  }
  if (print_mask & PRINT_IBATT_RAW) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ibatt_ %u ", adc3_raw[1]);
  }
  if (pos > 0) {
    buffer[pos - 1] = '\n';
    buffer[pos] = '\0';
    CDC_Transmit_HS((uint8_t*)buffer, pos);
  }
}

/**
 * @brief Function to print telemetry data in binary format over USB.
 * @note The data fields to be printed are determined by the print_mask variable,with variable data packet length. The packet starts with a header (0xAA 0x55), followed by a 4-byte mask indicating which fields are included, and then the data fields in the order defined by the print_mask.
 * @note Binary data sent is parsed automatically by a script on the host computer.
 */
void printTelemetryBinary(void) {
  if (print_mask == 0 || (print_format != PrintFormat::PRINT_BINARY)) return;

  uint16_t adc1_raw[3];
  uint16_t adc2_raw[2];
  uint16_t adc3_raw[2];
  float ia, ib, ic, va, vb, vbatt, ibatt;
  if ((print_mask & (PRINT_IA
                   | PRINT_VB
                   | PRINT_VBATT
                   | PRINT_IA_RAW
                   | PRINT_VB_RAW
                   | PRINT_VBATT_RAW)) != 0) {
    adc1.getLatestDataMean(adc1_raw, FOC_OVERSAMPLING_SIZE);

    ia = adcToCurrent(adc1_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ia_offset, adc_gain.ia_shunt);
    vb = adcToVoltage(adc1_raw[1], 3.3f, 65536, adc_gain.vb_gain, 1.65f + adc_gain.vb_offset);
    vbatt = adcToVoltage(adc1_raw[2], 3.3f, 65536, adc_gain.vbatt_gain, 0.0f + adc_gain.vbatt_offset);
  }
  if ((print_mask & (PRINT_IB
                   | PRINT_VA
                   | PRINT_IB_RAW
                   | PRINT_VA_RAW)) != 0) {
    adc2.getLatestDataMean(adc2_raw, FOC_OVERSAMPLING_SIZE);

    ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ib_offset, adc_gain.ib_shunt);
    va = adcToVoltage(adc2_raw[1], 3.3f, 65536, adc_gain.va_gain, 1.65f + adc_gain.va_offset);
  }
  if ((print_mask & (PRINT_IC
                   | PRINT_IBATT
                   | PRINT_IC_RAW
                   | PRINT_IBATT_RAW)) != 0) {
    adc3.getLatestDataMean(adc3_raw, FOC_OVERSAMPLING_SIZE);

    ic = adcToCurrent(adc3_raw[0], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ic_offset, adc_gain.ic_shunt);
    ibatt = adcToCurrent(adc3_raw[1], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ibatt_offset, adc_gain.ibatt_shunt);
  }

  // Construct a binary packet
  uint8_t buffer[128];
  uint8_t* ptr = buffer;
  uint32_t mask = print_mask;

  *ptr++ = 0xAA;
  *ptr++ = 0x55;
  
  memcpy(ptr, &mask, 4);
  ptr += 4;

  if (print_mask & PRINT_HALLBIN) {
    uint8_t val = hallsensor.getState() & 0x07;
    memcpy(ptr, &val, 1);
    ptr += 1;
  }
  if (print_mask & PRINT_HALLDEC) {
    uint8_t val = hallsensor.getState() & 0x07;
    memcpy(ptr, &val, 1);
    ptr += 1;
  }
  if (print_mask & PRINT_RPM) {
    float val = encoder.getRPM();
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_POS) {
    uint16_t val = encoder.getPos();
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_DUTY_A) {
    float val = motorPWM.getDuty(0);
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_DUTY_B) {
    float val = motorPWM.getDuty(1);
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_DUTY_C) {
    float val = motorPWM.getDuty(2);
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IA) {
    float val = ia;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IB) {
    float val = ib;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IC) {
    float val = ic;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_VA) {
    float val = va;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_VB) {
    float val = vb;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_VBATT) {
    float val = vbatt;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IBATT) {
    float val = ibatt;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IA_RAW) {
    uint16_t val = adc1_raw[0];
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_IB_RAW) {
    uint16_t val = adc2_raw[0];
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_IC_RAW) {
    uint16_t val = adc3_raw[0];
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_VA_RAW) {
    uint16_t val = adc2_raw[1];
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_VB_RAW) {
    uint16_t val = adc1_raw[1];
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_VBATT_RAW) {
    uint16_t val = adc1_raw[2];
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_IBATT_RAW) {
    uint16_t val = adc3_raw[1];
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (ptr != buffer) {
      CDC_Transmit_HS(buffer, ptr - buffer);
  }
}

void speedControl(void) {
  uint32_t rpm = encoder.getRPM();
}

void alignRotor(void) {
  if (control_mode != MotorControlMode::MOTOR_ALIGN) return;

  static uint16_t p_pos = 0;
  static uint16_t settlement_counter = 0;

  if ((system_flag & FLAG_ROTOR_ALIGNING) == 0) {
    uint16_t vdc_sample[FOC_OVERSAMPLING_SIZE];
    adc1.getLatestChannel(2, vdc_sample, FOC_OVERSAMPLING_SIZE);
    float Vdc = adcToVoltage(fastAverage(vdc_sample, FOC_OVERSAMPLING_SIZE), 3.3f, 65536,
                             adc_gain.vbatt_gain, adc_gain.vbatt_offset);

    float dutyA, dutyB, dutyC;
    foc_state.ts = 1.0f / (float)motorPWM.getFrequency();
    foc_align_zero(&foc_state, MOTOR_ALIGNMENT_VOLTAGE, Vdc, &dutyA, &dutyB, &dutyC);
    motorPWM.setDuty(dutyA, dutyB, dutyC);

    system_flag |= FLAG_ROTOR_ALIGNING;
  }

  uint16_t pos = encoder.getPosBypass();
  
  int16_t delta_pos = (int16_t)(pos - p_pos);
  p_pos = pos;
  if (abs(delta_pos) <= MOTOR_ALIGNMENT_THRESHOLD) {
    settlement_counter++;
  }
  else {
    settlement_counter = 0;
  }

  //char buffer[64];
  //snprintf(buffer, sizeof(buffer), "Aligning... Pos: %u, Delta: %d, Counter: %d\n", pos, delta_pos, settlement_counter);
  //CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

  if (settlement_counter > MOTOR_ALIGNMENT_POS_WINDOW) {
    encoder.elecZeroAlign();

    control_mode = MotorControlMode::MOTOR_STOP;
    motorPWM.stop();
    relay.write(0);
    system_flag &= ~FLAG_ROTOR_ALIGNING;
    system_flag |= FLAG_ELEC_ZERO_ALIGNED;

    CDC_Transmit_HS((uint8_t*)"Electrical Zero Angle Aligned\n", 33);
  }
}

/**
 * @brief Simple startup sequence — resets FOC state and goes directly into
 *        FOC_LINEAR mode at the configured initial RPM target.
 *
 * VVVF ramp-up has been removed. The motor starts under closed-loop FOC
 * from rest. Ensure the encoder is homed / synchronised before calling.
 *
 * @note The original VVVF ramp-up is preserved in vvvfRampUp() below but is
 *       no longer called from here.
 */
void startUpSequence(void) {
  if (control_mode != MotorControlMode::MOTOR_STARTUP) return;

  if ((system_flag & FLAG_ELEC_ZERO_ALIGNED) == 0) {
    control_mode = MotorControlMode::MOTOR_ALIGN;
    alignRotor();
    return;
  }

  foc_reset(&foc_state);
  hallsensor.read();
  system_flag |= FLAG_VVVF_RAMP_UP;
  control_mode = MotorControlMode::MOTOR_VVVF;
  vvvfRampUp();

  //usb_printf("Startup: FOC enabled, target=%u RPM\r\n", (unsigned)FOC_INITIAL_RPM);
}

/**
 * @brief Implements a VVVF ramp-up sequence for a BLDC motor. Gradually increases the frequency and amplitude of the PWM signals to smoothly accelerate the motor from standstill to a target speed defined by VVVF_THRESHOLD_RPM.
 * @note To be called in the TIM8 update interrupt when running in MOTOR_STARTUP mode.
 */
void vvvfRampUp(void) {
  if (control_mode != MotorControlMode::MOTOR_VVVF) return;
  const uint32_t ramp_up = VVVF_RAMP_UP_SPEED; // RPM/s
  static float rpm;  
  static float angle;
  static bool accelerating;

  // Initialize on zero speed starting
  if ((system_flag & FLAG_VVVF_RUNNING) == 0) {
    rpm = 0.0f;
    angle = 0.0f;
    accelerating = true;
    system_flag |= FLAG_VVVF_RUNNING;
  }

  // Audible frequency adjustment
  if (system_flag & FLAG_AUDIBLE) {
    if (rpm < 500.0f) {
      motorPWM.setFrequency(450);
    }
    else if (rpm < 1000.0f) {
      motorPWM.setFrequency(900);
    }
    else {
      motorPWM.setFrequency((uint32_t)rpm * 2);
    }
  }
  else {
    if (motorPWM.getFrequency() != 20000) motorPWM.setFrequency(20000);
  }

  // Increment aligned with interrupt frequency
  uint32_t frequency = motorPWM.getFrequency();
  float step_increment = (float)ramp_up / frequency;

  // Ramp up or down the speed
  if (system_flag & FLAG_VVVF_RAMP_UP) {
    if (accelerating) {
      rpm += step_increment;
      if (rpm >= VVVF_MAX_RPM >> 1) {
        rpm = VVVF_MAX_RPM >> 1;
        accelerating = false;
      }
    }
  }
  else {
    if (rpm > 0.0f) {
      rpm -= 2 * step_increment;
      if (rpm <= 0.0f) {
        rpm = 0.0f;
        system_flag &= ~FLAG_VVVF_RUNNING;
        control_mode = MotorControlMode::MOTOR_STOP;
        motorPWM.setDuty(-1.0f, -1.0f, -1.0f);
        relay.write(0);
        return;
      }
    }
  }

  // Calculate electrical angle
  float electrical_freq = rpm / 60.0f * MOTOR_POLE_PAIRS;

  float delta_angle = 2.0f * M_PI * electrical_freq / frequency;
  angle += delta_angle * MOTOR_ROTATION_DIRECTION * -1.0f;
  if (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;

  // Calculate voltage amplitude
  float amplitude;
  const float MIN_VOLTAGE = 0.15f;      // Boost start voltage
  const float KNEE_RPM = 1000.0f;        // Knee point (RPM)
  if (rpm < KNEE_RPM) {
      // Increase amplitude from MIN_VOLTAGE until knee point
      amplitude = MIN_VOLTAGE + (1.0f - MIN_VOLTAGE) * (rpm / KNEE_RPM);
  } else {
      // Remains at maximum amplitude after knee point
      amplitude = 1.0f;
  }

  // Limit output range
  if (amplitude > 1.0f) amplitude = 1.0f;

  float dutyA = 0.5f + amplitude * 0.5f * sinf(angle);
  float dutyB = 0.5f + amplitude * 0.5f * sinf(angle + 2.0f * M_PI / 3.0f);
  float dutyC = 0.5f + amplitude * 0.5f * sinf(angle + 4.0f * M_PI / 3.0f);

  motorPWM.setDuty(dutyA, dutyB, dutyC);

  if (FOC_ALLOWED && encoder.is_synchronized_ && rpm >= VVVF_THRESHOLD_RPM >> 1) {
    system_flag &= ~FLAG_VVVF_RUNNING;
    system_flag |= FLAG_FOC_RUNNING;
    foc_state.target_rpm = FOC_INITIAL_RPM;
    control_mode = MotorControlMode::MOTOR_FOC_LINEAR;
    CDC_Transmit_HS((uint8_t*)"Entering FOC mode\r\n", 19);
  }
}

/**
 * @brief Implements six-step commutation control for a BLDC motor using Hall sensor feedback. Include 2 commutation tables for clockwise and anti-clockwise.
 * @note  To be called at every Hall sensor interrupt when running in six-step mode.
 */
void sixStepCommutation(void) {
// Clockwise commutation table
const int8_t commutation_cw[8][3] = {
    { -1, -1, -1 }, // 0 (Invalid)
    { 1, 0, -1 },   // 1 (001): A+, B-
    { -1, 1, 0 },   // 2 (010): B+, C-
    { 1, -1, 0 },   // 3 (011): A+, C-
    { 0, -1, 1 },   // 4 (100): C+, A-
    { -1, 0, 1 },   // 5 (101): C+, B-
    { 0, 1, -1 },   // 6 (110): B+, A-
    { -1, -1, -1 }  // 7 (Invalid)
};

// Anti-clockwise commutation table
const int8_t commutation_acw[8][3] = {
    { -1, -1, -1 }, // 0 (Invalid)
    { 0, -1, 1 },   // 1 (001)(4): C+, A-
    { 1, 0, -1 },   // 2 (010)(1): A+, B-
    { -1, 0, 1 },   // 3 (011)(5): C+, B-
    { -1, 1, 0 },   // 4 (100)(2): B+, C-
    { 0, 1, -1 },   // 5 (101)(6): B+, A-
    { 1, -1, 0 },   // 6 (110)(3): A+, C-
    { -1, -1, -1 }  // 7 (Invalid)
};

  if ((system_flag & FLAG_SIXSTEP_RUNNING) == 0) {
    hallsensor.read();
    system_flag |= FLAG_SIXSTEP_RUNNING;
  }

  uint8_t hall_state = hallsensor.getState();

  if (hall_state < 1 || hall_state > 6) return;

  int8_t a_state, b_state, c_state;

  // Determine the desired state of each phase based on the current Hall sensor state and rotation direction
  if (MOTOR_ROTATION_DIRECTION == -1) {
      a_state = commutation_cw[hall_state][0];
      b_state = commutation_cw[hall_state][1];
      c_state = commutation_cw[hall_state][2];
  } else if (MOTOR_ROTATION_DIRECTION == 1) {
      a_state = commutation_acw[hall_state][0];
      b_state = commutation_acw[hall_state][1];
      c_state = commutation_acw[hall_state][2];
  }

  float dutyB = (b_state == 1) ? SIXSTEP_DUTYCYCLE : (b_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
  float dutyC = (c_state == 1) ? SIXSTEP_DUTYCYCLE : (c_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
  float dutyA = (a_state == 1) ? SIXSTEP_DUTYCYCLE : (a_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;

  motorPWM.setDuty(dutyA, dutyB, dutyC);
}

/**
 * @brief Converts a raw ADC value to a voltage in volts, applying the reference voltage, resolution, gain, and offset for calibration.
 * @param raw The raw ADC value to be converted.
 * @param vref The reference voltage for the ADC.
 * @param resolution The resolution of the ADC (e.g., 4096 for 12-bit, 65536 for 16-bit).
 * @param gain The gain factor for the ADC.
 * @param offset The offset factor for the ADC.
 * @return The converted voltage in volts.
 */
float adcToVoltage(uint32_t raw, float vref, uint32_t resolution, float gain, float offset) {
  return (((float)raw / (float)resolution) * vref - offset) / gain;
}

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
float adcToCurrent(uint32_t raw, float vref, uint32_t resolution, float gain, float offset, float shunt) {
  float voltage = adcToVoltage(raw, vref, resolution, gain, offset);
  return voltage / shunt;
}

/**
 * @brief Computes the average of an array of uint16_t values using a fast method that avoids overflow. The function sums all values in a uint32_t variable and then right shifts by the number of bits corresponding to the size of the array (assuming size is a power of 2) to get the average.
 * @param data_ptr Pointer to the array of uint16_t values.
 * @param size The number of elements in the array (must be a power of 2).
 * @return The average value as a uint16_t.
 * @attention `size` must be a power of 2, otherwise 0 will be returned.
 */
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

/**
 * @brief Checks if a given uint16_t value is a power of 2.
 * @param x The value to check.
 * @return true if x is a power of 2, false otherwise.
 */
bool isPowerOfTwo(uint16_t x) {
  return (x != 0) && ((x & (x - 1)) == 0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  USB Command Handling
//  The following functions facilitate handling of USB commands received from host computer
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief Command handler for "start" command.
 * @note Initializes the motor control system and starts the VVVF ramp-up sequence.
 */
void cmd_start(int argc, char** argv) {
    if (control_mode == MotorControlMode::MOTOR_PROTECTION) {protectionModePrint(); return;}
    control_mode = MotorControlMode::MOTOR_STARTUP;
    system_flag &= ~FLAG_VVVF_RUNNING;
    system_flag &= ~FLAG_SIXSTEP_RUNNING;
    system_flag &= ~FLAG_FOC_RUNNING;
    relay.write(1);
    startUpSequence();
    CDC_Transmit_HS((uint8_t*)"Starting\r\n", 10);
}

/**
 * @brief Command handler for "stop" command.
 * @note Stops the motor immediately and resets the control state.
 * @note If currently in VVVF ramp-up, it will start ramping down instead of an immediate stop.
 */
void cmd_stop(int argc, char** argv) {
    if (control_mode == MotorControlMode::MOTOR_VVVF && system_flag & FLAG_VVVF_RAMP_UP) {
        system_flag &= ~FLAG_VVVF_RAMP_UP; // Start ramp down
        CDC_Transmit_HS((uint8_t*)"VVVF ramping down\r\n", 21);
    } else {
        control_mode = MotorControlMode::MOTOR_STOP;
        system_flag &= ~FLAG_ROTOR_ALIGNING;
        system_flag &= ~FLAG_VVVF_RUNNING;
        system_flag &= ~FLAG_SIXSTEP_RUNNING;
        system_flag &= ~FLAG_FOC_RUNNING;
        motorPWM.stop();
        foc_reset(&foc_state);
        relay.write(0);
        CDC_Transmit_HS((uint8_t*)"Stopping\r\n", 10);
    }
}

/**
 * @brief Command handler for "reset" command.
 * @note Resets the entire system to a known safe state, stopping the motor and clearing any active control modes or flags.
 */
void cmd_reset(int argc, char** argv) {
    control_mode = MotorControlMode::MOTOR_STOP;
    system_flag &= ~FLAG_VVVF_RUNNING;
    system_flag &= ~FLAG_SIXSTEP_RUNNING;
    system_flag &= ~FLAG_FOC_RUNNING;
    error_flag &= ~ERROR_OVERCURRENT;
    motorPWM.stop();
    led_red.write(0);
    foc_reset(&foc_state);
    relay.write(0);
    CDC_Transmit_HS((uint8_t*)"Resetting\r\n", 11);
}

/**
 * @brief Command handler for "foc" command.
 * @note If the argument is "status" or "stat", it prints the current FOC state including RPM, currents, voltages, and fault status.
 * @note Otherwise, it treats the argument as a target RPM value, resets the FOC state, and starts FOC control to reach the target RPM.
 */
void cmd_foc(int argc, char** argv) {
    if (strcmp(argv[1], "status") == 0 || strcmp(argv[1], "stat") == 0) {
        usb_printf("RPM=%.1f  Id=%.3fA  Iq=%.3fA  Vd=%.2fV  Vq=%.2fV  Vdc=%.2fV  |u|=%.2fV  fault=%d\r\n",
                  foc_state.rpm,
                  foc_state.Id, foc_state.Iq,
                  foc_state.Vd_cmd, foc_state.Vq_cmd,
                  foc_state.Vdc, foc_state.u_mag,
                  (int)foc_state.fault);
    }
    // argv[1] contains the string of the target RPM
    else {
        if (control_mode == MotorControlMode::MOTOR_PROTECTION) {protectionModePrint(); return;}
        int rpm_cmd = atoi(argv[1]);
        
        foc_reset(&foc_state);
        foc_state.target_rpm = (float)rpm_cmd;
        system_flag &= ~FLAG_VVVF_RUNNING;
        system_flag &= ~FLAG_SIXSTEP_RUNNING;
        system_flag |= FLAG_FOC_RUNNING;
        relay.write(1);
        motorPWM.start();
        control_mode = MotorControlMode::MOTOR_FOC_LINEAR;
        usb_printf("FOC started  target=%d RPM\r\n", rpm_cmd);
    }
}

/**
 * @brief Command handler for "rpm" command.
 * @note Sets the target RPM for the motor
 */
void cmd_rpm(int argc, char** argv) {
    int rpm_cmd = atoi(argv[1]);
    foc_state.target_rpm = (float)rpm_cmd;
    usb_printf("Target RPM set to %d\r\n", rpm_cmd);
}

/**
 * @brief Command handler for "sixstep" command.
 * @note Starts six-step commutation control mode.
 */
void cmd_sixstep(int argc, char** argv) {
    if (control_mode == MotorControlMode::MOTOR_PROTECTION) {protectionModePrint(); return;}
    control_mode = MotorControlMode::MOTOR_SIX_STEP;
    system_flag &= ~FLAG_VVVF_RUNNING;
    system_flag &= ~FLAG_FOC_RUNNING;
    relay.write(1);
    sixStepCommutation();
    CDC_Transmit_HS((uint8_t*)"Six-step running\r\n", 31);
}

/**
 * @brief Command handler for "vvvf" command.
 * @note Set the speed setpoint for FOC speed loop.
 */
void cmd_speed(int argc, char** argv) {
    float speed = atof(argv[1]);
    if (speed >= -5000.0f && speed <= 5000.0f) {
        relay.write(1);
        target.speed = speed;
        foc_state.target_rpm = speed;
        CDC_Transmit_HS((uint8_t*)"Speed set\r\n", 11);
    } else {
        const char* err = "Invalid speed\r\n";
        CDC_Transmit_HS((uint8_t*)err, strlen(err));
    }
}

/**
 * @brief Command handler for "duty" command.
 * @note Sets the duty cycle for each phase directly.
 * @attention ONLY use when supplied with a CURRENT LIMITED power source.
 * @attention This command overrides all control modes and should be used with caution.
 */
void cmd_duty(int argc, char** argv) {
  if (BATTERY_PROTECTION) {batteryProtectionPrint(); return;}
    if (control_mode == MotorControlMode::MOTOR_PROTECTION) {protectionModePrint(); return;}
    // argv[1] lands as "0.3,0.3,0.3" from the universal space tokenizer.
    // We split it via commas using strtok in-place.
    float values[3] = {0};
    int count = 0;
    
    char* token = strtok(argv[1], ",");
    while (token != NULL && count < 3) {
        while (*token == ' ') token++; // Remove leading spaces if any
        values[count++] = atof(token);
        token = strtok(NULL, ",");
    }

    if (count == 3) {
        if (values[0] >= -1.0f && values[0] <= 1.0f &&
            values[1] >= -1.0f && values[1] <= 1.0f &&
            values[2] >= -1.0f && values[2] <= 1.0f) {
            
            relay.write(1);
            motorPWM.setDuty(values[0], values[1], values[2]);
            control_mode = MotorControlMode::MOTOR_MANUAL;
            system_flag &= ~FLAG_VVVF_RUNNING;
            system_flag &= ~FLAG_SIXSTEP_RUNNING;
            system_flag &= ~FLAG_FOC_RUNNING;
            CDC_Transmit_HS((uint8_t*)"Duty set\r\n", 10);
        } else {
            CDC_Transmit_HS((uint8_t*)"Duty values out of range [-1,1]\r\n", 34);
        }
    } else {
        CDC_Transmit_HS((uint8_t*)"Invalid duty format. Usage: duty 0.3,0.3,0.3\r\n", 48);
    }
}

/**
 * @brief Command handler for "vec" command.
 * @note Applies a predefined six-step commutation vector based on the input number (0-5).
 * @attention ONLY use when supplied with a CURRENT LIMITED power source.
 * @attention This command overrides all control modes and should be used with caution.
 */
void cmd_vec(int argc, char** argv) {
    if (BATTERY_PROTECTION) {batteryProtectionPrint(); return;}
    if (control_mode == MotorControlMode::MOTOR_PROTECTION) {protectionModePrint(); return;}
    relay.write(1);
    int vec_num = atoi(argv[1]);
    
    if (vec_num >= 0 && vec_num <= 5) {
        const int8_t vector_states[6][3] = {
            { 1,  0, -1},  // 0: A+B-
            { 1, -1,  0},  // 1: A+C-
            {-1,  1,  0},  // 2: B+C-
            { 0,  1, -1},  // 3: B+A-
            { 0, -1,  1},  // 4: C+A-
            {-1,  0,  1}   // 5: C+B-
        };
      
        int8_t a_state = vector_states[vec_num][0];
        int8_t b_state = vector_states[vec_num][1];
        int8_t c_state = vector_states[vec_num][2];
      
        float dutyA = (a_state == 1) ? SIXSTEP_DUTYCYCLE : (a_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
        float dutyB = (b_state == 1) ? SIXSTEP_DUTYCYCLE : (b_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
        float dutyC = (c_state == 1) ? SIXSTEP_DUTYCYCLE : (c_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
      
        motorPWM.setDuty(dutyA, dutyB, dutyC);
        control_mode = MotorControlMode::MOTOR_MANUAL;
        system_flag &= ~FLAG_VVVF_RUNNING;
        system_flag &= ~FLAG_SIXSTEP_RUNNING;
        system_flag &= ~FLAG_FOC_RUNNING;

        uint8_t hall_state = hallsensor.getState();
        char resp[64];
        int len = snprintf(resp, sizeof(resp), "Vector %d applied, Hall=%d\r\n", vec_num, hall_state);
        CDC_Transmit_HS((uint8_t*)resp, len);
    } else {
        CDC_Transmit_HS((uint8_t*)"Invalid vector. Use 0-5.\r\n", 26);
    }
}

/**
 * @brief Command handler for "tune" command.
 * @note Allows tuning of various parameters such as PID gains and ADC calibration values via USB commands.
 */
void cmd_tune(int argc, char** argv) {
    // "tune speed p 0.1" -> argv[1]="speed", argv[2]="p", argv[3]="0.1"
    char* subsys = argv[1];
    char* param = argv[2];
    float value = atof(argv[3]);

    bool success = false;
    char resp[64];
    float original = 0.0f;

    if (strcmp(subsys, "speed") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_speed.kp;
            foc_state.pi_speed.kp = value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_speed.ki;
            foc_state.pi_speed.ki = value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "id") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_d.kp;
            foc_state.pi_d.kp = value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_d.ki;
            foc_state.pi_d.ki = value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "iq") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_q.kp;
            foc_state.pi_q.kp = value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_q.ki;
            foc_state.pi_q.ki = value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "fw") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_fw.kp;
            foc_state.pi_fw.kp = value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_fw.ki;
            foc_state.pi_fw.ki = value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "flux") == 0) {
        if (strcmp(param, "p") == 0) { 
            snprintf(resp, sizeof(resp), "Flux gain set to %.3f\r\n", value); 
            CDC_Transmit_HS((uint8_t*)resp, strlen(resp));
            return; 
        }
    } 
    else if (strcmp(subsys, "gain") == 0) {
        if (strcmp(param, "ia") == 0) {
            original = adc_gain.ia_shunt;
            adc_gain.ia_shunt = value;
            success = true;
        } else if (strcmp(param, "ib") == 0) {
            original = adc_gain.ib_shunt;
            adc_gain.ib_shunt = value;
            success = true;
        } else if (strcmp(param, "ic") == 0) {
            original = adc_gain.ic_shunt;
            adc_gain.ic_shunt = value;
            success = true;
        } else if (strcmp(param, "va") == 0) {
            original = adc_gain.va_gain;
            adc_gain.va_gain = value;
            success = true;
        } else if (strcmp(param, "vb") == 0) {
            original = adc_gain.vb_gain;
            adc_gain.vb_gain = value;
            success = true;
        } else if (strcmp(param, "ibatt") == 0) {
            original = adc_gain.ibatt_shunt;
            adc_gain.ibatt_shunt = value;
            success = true;
        } else if (strcmp(param, "vbatt") == 0) {
            original = adc_gain.vbatt_gain;
            adc_gain.vbatt_gain = value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "offset") == 0) {
        if (strcmp(param, "ia") == 0) {
            original = adc_gain.ia_offset;
            adc_gain.ia_offset = value;
            success = true;
        } else if (strcmp(param, "ib") == 0) {
            original = adc_gain.ib_offset;
            adc_gain.ib_offset = value;
            success = true;
        } else if (strcmp(param, "ic") == 0) {
            original = adc_gain.ic_offset;
            adc_gain.ic_offset = value;
            success = true;
        } else if (strcmp(param, "va") == 0) {
            original = adc_gain.va_offset;
            adc_gain.va_offset = value;
            success = true;
        } else if (strcmp(param, "vb") == 0) {
            original = adc_gain.vb_offset;
            adc_gain.vb_offset = value;
            success = true;
        } else if (strcmp(param, "ibatt") == 0) {
            original = adc_gain.ibatt_offset;
            adc_gain.ibatt_offset = value;
            success = true;
        } else if (strcmp(param, "vbatt") == 0) {
            original = adc_gain.vbatt_offset;
            adc_gain.vbatt_offset = value;
            success = true;
        }
    }

    if (success) {
        int len = snprintf(resp, sizeof(resp), "%s %s set to %.4f (was %.4f)\r\n", subsys, param, value, original);
        CDC_Transmit_HS((uint8_t*)resp, len);
    } else {
        int len = snprintf(resp, sizeof(resp), "Unknown parameter '%s' or subsystem '%s'\r\n", param, subsys);
        CDC_Transmit_HS((uint8_t*)resp, len);
    }
}

/**
 * @brief Command handler for "log" command.
 * @note Allows dynamic configuration of which variables to include in the data log output andoutput encoding.
 */
void cmd_log(int argc, char** argv) {
    char* action = argv[1];

    if (strcmp(action, "preset") == 0) {
        if (argc < 3) {
            CDC_Transmit_HS((uint8_t*)"Missing preset number\r\n", 23);
            return;
        }
        
        int preset_id = atoi(argv[2]); // Safe conversion of "1", "2", etc.
        
        switch (preset_id) {
            case 1: // Preset 1: FOC Tuning
                print_mask = PRINT_RPM | PRINT_IA | PRINT_IB | PRINT_IC | PRINT_VBATT;
                CDC_Transmit_HS((uint8_t*)"Preset 1 active\r\n", 37);
                break;
                
            case 2: // Preset 2: Raw Sensor Calibration
                print_mask = PRINT_IA | PRINT_IB | PRINT_IC | PRINT_IA_RAW | PRINT_IB_RAW | PRINT_IC_RAW;
                CDC_Transmit_HS((uint8_t*)"Preset 2 active\r\n", 45);
                break;
                
            case 3: // Preset 3: Power Monitoring
                print_mask = PRINT_VBATT | PRINT_IBATT | PRINT_DUTY_A | PRINT_DUTY_B | PRINT_DUTY_C;
                CDC_Transmit_HS((uint8_t*)"Preset 3 active\r\n", 40);
                break;
                
            default:
                CDC_Transmit_HS((uint8_t*)"Unknown preset. Try 1, 2, or 3\r\n", 32);
                break;
        }
        return; // Exit here
    }

    else if (strcmp(action, "add") == 0 || strcmp(action, "rm") == 0) {
        if (argc < 3) {
            CDC_Transmit_HS((uint8_t*)"Missing variable name\r\n", 23);
            return;
        }
        char* token = argv[2];
        uint32_t flag = 0;
        
        if (strcmp(token, "hall") == 0) flag = PRINT_HALLBIN;
        else if (strcmp(token, "hall_dec") == 0) flag = PRINT_HALLDEC;
        else if (strcmp(token, "rpm") == 0) flag = PRINT_RPM;
        else if (strcmp(token, "pos") == 0) flag = PRINT_POS;
        else if (strcmp(token, "duty_a") == 0) flag = PRINT_DUTY_A;
        else if (strcmp(token, "duty_b") == 0) flag = PRINT_DUTY_B;
        else if (strcmp(token, "duty_c") == 0) flag = PRINT_DUTY_C;
        else if (strcmp(token, "ia") == 0) flag = PRINT_IA;
        else if (strcmp(token, "ib") == 0) flag = PRINT_IB;
        else if (strcmp(token, "ic") == 0) flag = PRINT_IC;
        else if (strcmp(token, "va") == 0) flag = PRINT_VA;
        else if (strcmp(token, "vb") == 0) flag = PRINT_VB;
        else if (strcmp(token, "vbatt") == 0) flag = PRINT_VBATT;
        else if (strcmp(token, "ibatt") == 0) flag = PRINT_IBATT;
        else if (strcmp(token, "ia_raw") == 0) flag = PRINT_IA_RAW;
        else if (strcmp(token, "ib_raw") == 0) flag = PRINT_IB_RAW;
        else if (strcmp(token, "ic_raw") == 0) flag = PRINT_IC_RAW;
        else if (strcmp(token, "va_raw") == 0) flag = PRINT_VA_RAW;
        else if (strcmp(token, "vb_raw") == 0) flag = PRINT_VB_RAW;
        else if (strcmp(token, "vbatt_raw") == 0) flag = PRINT_VBATT_RAW;
        else if (strcmp(token, "ibatt_raw") == 0) flag = PRINT_IBATT_RAW;
        else if (strcmp(token, "all") == 0 && strcmp(action, "rm") == 0) {
            print_mask = 0;
            CDC_Transmit_HS((uint8_t*)"All variables removed\r\n", 23);
            return;
        } else {
            CDC_Transmit_HS((uint8_t*)"Unknown variable\r\n", 18);
            return;
        }

        if (strcmp(action, "add") == 0) {
            print_mask |= flag;
            CDC_Transmit_HS((uint8_t*)"Variable added\r\n", 16);
        } else {
            print_mask &= ~flag;
            CDC_Transmit_HS((uint8_t*)"Variable removed\r\n", 18);
        }
    } 
    else if (strcmp(action, "utf8") == 0) {
        print_format = PrintFormat::PRINT_UTF8;
        CDC_Transmit_HS((uint8_t*)"Print format set to UTF8\r\n", 27);
    } 
    else if (strcmp(action, "bin") == 0) {
        print_format = PrintFormat::PRINT_BINARY;
        CDC_Transmit_HS((uint8_t*)"Print format set to BINARY\r\n", 28);
    } 
    else {
        CDC_Transmit_HS((uint8_t*)"Invalid log action\r\n", 20);
    }
}

void cmd_audible(int argc, char** argv) {
    if (system_flag & FLAG_AUDIBLE) {
        system_flag &= ~FLAG_AUDIBLE;
        CDC_Transmit_HS((uint8_t*)"Audible frequency disabled\r\n", 28);
    } else {
        system_flag |= FLAG_AUDIBLE;
        CDC_Transmit_HS((uint8_t*)"Audible frequency enabled\r\n", 27);
    }
}

void cmd_sin(int argc, char** argv) {
    float value = atof(argv[1]);
    char resp[64];
    int len = snprintf(resp, sizeof(resp), "sinf(%.2f) = %.5f, cordic::sinf(%.2f) = %.5f\r\n", value, sinf(value), value, cordic::sinf(value));
    CDC_Transmit_HS((uint8_t*)resp, len);
}

void cmd_cos(int argc, char** argv) {
    float value = atof(argv[1]);
    char resp[64];
    int len = snprintf(resp, sizeof(resp), "cosf(%.2f) = %.5f, cordic::cosf(%.2f) = %.5f\r\n", value, cosf(value), value, cordic::cosf(value));
    CDC_Transmit_HS((uint8_t*)resp, len);
}

void cmd_arctan(int argc, char** argv) {
    float y = atof(argv[1]);
    float x = atof(argv[2]);
    if (x <= 0.0f || y <= 0.0f) {
        CDC_Transmit_HS((uint8_t*)"x and y must be positive for arctan test\r\n", 43);
        return;
    }
    char resp[64];
    int len = snprintf(resp, sizeof(resp), "atan2f(%.2f, %.2f) = %.5f, cordic::atan2f(%.2f, %.2f) = %.5f\r\n", y, x, atan2f(y, x), y, x, cordic::atan2f(y, x));
    CDC_Transmit_HS((uint8_t*)resp, len);
}

void cmd_hypot(int argc, char** argv) {
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    if (x <= 0.0f || y <= 0.0f) {
        CDC_Transmit_HS((uint8_t*)"x and y must be positive for hypot test\r\n", 42);
        return;
    }
    char resp[64];
    int len = snprintf(resp, sizeof(resp), "hypotf(%.2f, %.2f) = %.5f, cordic::hypotf(%.2f, %.2f) = %.5f\r\n", x, y, hypotf(x, y), x, y, cordic::hypotf(x, y));
    CDC_Transmit_HS((uint8_t*)resp, len);
}

// ─────────────────────────────────────────────────────────────────────────────
//  FOC ISR tick — called from TIM8 update interrupt at 20 kHz
// ─────────────────────────────────────────────────────────────────────────────

/*
 * foc_isr_tick
 *
 * Reads ADC and encoder, converts to SI units using the same formula and
 * adc_gain struct as the rest of the codebase, then calls foc_run() and
 * applies the resulting duty cycles to the inverter.
 *
 * ADC formula (from main.cpp adcToVoltage):
 *   voltage = ((raw / resolution) * vref - offset) / gain
 *   current = voltage / shunt
 *
 * Current offsets: 1.65f (Vref/2 midpoint) + adc_gain.i*_offset (per-board trim)
 *
 * Encoder: getPos_rad() reads TIM4 hardware counter (updated continuously).
 *          getRPM() is updated by TIM6 at 1 kHz — safe to read any time.
 *
 * Must complete within one PWM period (50 µs at 20 kHz).
 */
static void foc_isr_tick(void)
{
    /* Guard: fault latched — stop inverter and return to STOP mode */
    static uint32_t tick_counter = 0;
    if (foc_state.fault) {
      // Prints out the current FOC state for debugging before stopping
      usb_printf("\r\n------------------------\r\nFOC fault at tick %u\r\nRPM=%.1f  Ia=%.3fA  Ib=%.3fA  Ic=%.3fA\r\nId=%.3fA  Iq=%.3fA  Vd=%.2fV  Vq=%.2fV  Vdc=%.2fV  |u|=%.2fV  fault=%d\r\n------------------------\r\n",
              tick_counter,
              foc_state.rpm,
              foc_state.Ia,  foc_state.Ib,  foc_state.Ic,
              foc_state.Id,  foc_state.Iq,
              foc_state.Vd_cmd, foc_state.Vq_cmd,
              foc_state.Vdc, foc_state.u_mag,
              (int)foc_state.fault);
      // Stop all 3 phases PWM output
      motorPWM.stop();
      // De-energize relay
      relay.write(0);
      // Set fault flag
      error_flag |= ERROR_OVERCURRENT;
      // Set control mode to PROTECTION and clear FOC running flag
      control_mode = MotorControlMode::MOTOR_PROTECTION;
      system_flag &= ~FLAG_FOC_RUNNING;
      tick_counter = 0;
      return;
    }

    /* -----------------------------------------------------------------------
     * 1. Read ADC — latest DMA sample (interrupt-safe via NDTR)
     *
     * Channel mapping (from parameters.h comment block):
     *   ADC1[0] = I_A   (PA7,  INP7,  16-bit)
     *   ADC1[1] = V_B+  (PA0,  INP16, 16-bit)  — not used in FOC loop
     *   ADC1[2] = V_BATT(PB0,  INP9,  16-bit)
     *   ADC2[0] = I_B   (PB1,  INP5,  16-bit)
     *   ADC2[1] = V_A+  (PC4,  INP4,  16-bit)  — not used in FOC loop
     *   ADC3[0] = I_C   (PC1,  INP11, 12-bit → 4096 counts)
     *   ADC3[1] = I_BATT(PC0,  INP10, 12-bit)  — not used in FOC loop
     * --------------------------------------------------------------------- */
    uint16_t adc1_raw[3];
    uint16_t adc2_raw[2];
    uint16_t adc3_raw[2];

    adc1.getLatestDataMean(adc1_raw, FOC_OVERSAMPLING_SIZE);
    adc2.getLatestDataMean(adc2_raw, FOC_OVERSAMPLING_SIZE);
    adc3.getLatestDataMean(adc3_raw, FOC_OVERSAMPLING_SIZE);

    /*
     * Convert raw ADC to amps using the real formula:
     *   adcToVoltage: ((raw/resolution)*vref - offset) / gain
     *   adcToCurrent: adcToVoltage(..., gain=1) / shunt
     *
     * offset = 1.65f (op-amp Vref/2) + adc_gain.i*_offset (per-board trim)
     * shunt  = adc_gain.i*_shunt  (from parameters.h, e.g. ADC_IA_SHUNT)
     *
     * Note: gain parameter passed as 1.0f to adcToVoltage for current
     *       channels — the op-amp gain of 50 is already embedded in the
     *       shunt value (effective sensitivity = 50 × shunt V/A).
     */
    float Ia = adcToCurrent(adc1_raw[0], 3.3f, 65536, 50.0f,
                            1.65f + adc_gain.ia_offset, adc_gain.ia_shunt);
    float Ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f,
                            1.65f + adc_gain.ib_offset, adc_gain.ib_shunt);
    float Ic = adcToCurrent(adc3_raw[0], 3.3f,  4096, 50.0f,
                            1.65f + adc_gain.ic_offset, adc_gain.ic_shunt);

    /* DC bus voltage */
    float Vdc = adcToVoltage(adc1_raw[2], 3.3f, 65536,
                             adc_gain.vbatt_gain, adc_gain.vbatt_offset);

    /* Guard: Vdc too low — SVPWM would divide by zero */
    if (Vdc < 1.0f) return;

    /* -----------------------------------------------------------------------
     * 2. Read encoder
     *    getPos_rad() — reads TIM4 CNT register directly (hardware quadrature)
     *    getRPM()     — filtered, updated by TIM6 at 1 kHz
     *
     *    theta_e = theta_mech × pole_pairs, wrapped to [0, 2π)
     *    omega_m = mechanical angular velocity (rad/s)
     * --------------------------------------------------------------------- */
    //float theta_mech = encoder.getPos_rad();
    //float theta_e    = fmodf(theta_mech * (float)MOTOR_POLE_PAIRS, 2.0f * M_PI);
    //if (theta_e < 0.0f) theta_e += 2.0f * M_PI;
    float theta_e    = encoder.getElecPos_rad();

    float omega_m = encoder.getRPM() * (2.0f * M_PI / 60.0f);

    /* -----------------------------------------------------------------------
     * 3. Run one FOC tick
     * --------------------------------------------------------------------- */
    float dutyA, dutyB, dutyC;
    foc_state.ts = 1.0f / (float)motorPWM.getFrequency();  // Update FOC state with actual PWM period
    foc_run(&foc_state,
            Ia, Ib, Ic,
            Vdc,
            theta_e, omega_m,
            &dutyA, &dutyB, &dutyC);

    /* -----------------------------------------------------------------------
     * 4. Apply to inverter
     * --------------------------------------------------------------------- */
    motorPWM.setDuty(dutyA, dutyB, dutyC);

    tick_counter++;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Memory Protection Unit (MPU) Configuration
// ─────────────────────────────────────────────────────────────────────────────

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

// ─────────────────────────────────────────────────────────────────────────────
//  Test Functions - No longer used
// ─────────────────────────────────────────────────────────────────────────────

// SVPWM test: Generates a rotating space vector by calculating the appropriate duty cycles for a 3-phase inverter using the SVPWM algorithm. The test runs in a timer interrupt to continuously update the PWM outputs and create a smooth rotation of the vector. This verifies that the PWM generation and modulation are working correctly, and that the motor responds as expected to changes in the duty cycle.
void test_PWM(void) {
    static uint32_t timeElapsed = 0U;

    float angle = (float) 2U * M_PI * timeElapsed / 20U;

    if (angle >= (M_PI)) angle -= 2U * M_PI;

    float dutyA;
    float dutyB;
    float dutyC;

    float alpha = cosf(angle);
    float beta = sinf(angle);

    const float Ts = 1.0f / 20000.0f;
    Modulate(ModulationType::SVPWM, alpha, beta, 2.0f, Ts, &dutyA, &dutyB, &dutyC);

    motorPWM.setDuty(dutyA, dutyB, dutyC);

    //usb.printf("Duty Cycle: %.2f %.2f %.2f %.2f %.2f %.2f\n", dutyA, dutyB, dutyC, alpha, beta, angle);

    if (timeElapsed++ >= 20U) timeElapsed = 0;
    //if (counter++ >= 50000) {
    //    counter = 0;
    //    usb.printf("One Second\n");
    //}

}

// Frequency sweep test: Sweeps the PWM frequency between 5 kHz and 10 kHz while keeping the duty cycle constant at 50%. This tests the timer's ability to adjust the PWM frequency on the fly and ensures that the motor responds correctly to frequency changes.
void test_PWM_sweep(void) {
    static float frequency = 5000.0f;
    static bool increasing = true;

    motorPWM.setDuty(0.5f, 0.5f, 0.5f);

    if (increasing) {
        frequency += 0.1f; // Increase frequency
        if (frequency >= 10000.0f) {
            frequency = 10000.0f;
            increasing = false; // Start decreasing after reaching 100%
        }
    } else {
        frequency -= 0.1f; // Decrease frequency
        if (frequency <= 5000.0f) {
            frequency = 5000.0f;
            increasing = true; // Start increasing after reaching 0%
        }
    }

    motorPWM.setFrequency((uint32_t)frequency);
}