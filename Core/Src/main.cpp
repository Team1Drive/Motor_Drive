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
#include "math_helpers.h"
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

void startUpSequence(MotorControlMode mode);
void alignRotor(void);
void vvvfRampUp(void);
void sixStepCommutation(void);

void clearRunningFlags(void);
void loadAdcCalibration(ADCGain_t* adc_gain, uint8_t preset_num);

/* Forward declaration for FOC ISR helper */
static void foc_isr_tick(void);
void focTick(void);

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
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

/* Declare ADC & DMA handles */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

/* Custom Class Objects */
ThreePhasePWMOut motorPWM(&htim8);

ADCSampler adc1(&hadc1, &hdma_adc1, adc1_buffer, ADC1_BUF_LEN);
ADCSampler adc2(&hadc2, &hdma_adc2, adc2_buffer, ADC2_BUF_LEN);
ADCSampler adc3(&hadc3, &hdma_adc3, adc3_buffer, ADC3_BUF_LEN);

Timer adcTimer(&htim1), printTimer(&htim2), ledTimer(&htim3), encoderTimer(&htim4), binaryLogTimer(&htim6), speedControlTimer(&htim16);

DigitalOut pwm_ch1_dis(GPIOA, GPIO_PIN_2), pwm_ch2_dis(GPIOB, GPIO_PIN_2), pwm_ch3_dis(GPIOB, GPIO_PIN_13);
DigitalOut led_red(GPIOC, GPIO_PIN_9), led_green(GPIOA, GPIO_PIN_8), led_yellow_1(GPIOA, GPIO_PIN_9), led_yellow_2(GPIOA, GPIO_PIN_10);
DigitalOut relay(GPIOD, GPIO_PIN_8);
Encoder encoder(&htim4, &htim15, GPIO_PIN_9, ENCODER_PPR, TIM6_FREQ_HZ, ENCODER_STALL_THRESHOLD);
HallSensor hallsensor(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_8, GPIOE, GPIO_PIN_4);

alignas(32) uint16_t adc1_proc_buffer[ADC1_BUF_LEN];
alignas(32) uint16_t adc2_proc_buffer[ADC2_BUF_LEN];
alignas(32) uint16_t adc3_proc_buffer[ADC3_BUF_LEN];

volatile MotorControlMode control_mode = MotorControlMode::MOTOR_STOP;

volatile uint32_t system_flag = 0;
volatile uint32_t error_flag = 0;

ADCGain_t adc_gain;

volatile Target_t target = { .speed = 0.0f, .torque = 0.0f, .time = 0.0f };

volatile uint32_t print_mask = 0;
volatile PrintFormat print_format = PrintFormat::PRINT_UTF8;

ring_buffer_t rx_ring = { .head = 0, .tail = 0 };

/* FOC state — single global instance */
FOC_State_t foc_state;

RollingMax ia_max, ib_max, ic_max, ibatt_max;




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
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();

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
  if (binaryLogTimer.startIT() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;
  if (speedControlTimer.startIT() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;

  if (HighResTimer::start() != HAL_OK) error_flag |= ERROR_TIM_CONFIG;

  if (encoder.start() != HAL_OK) error_flag |= ERROR_ENCODER_CONFIG;

  /* Start PWM */
  if (motorPWM.init() != HAL_OK) error_flag |= ERROR_PWM_CONFIG;

  /* Enable Caches */
  SCB_EnableICache();
  SCB_EnableDCache();

  usb_printf("HAL Initialized\n");

  loadAdcCalibration(&adc_gain, 1);
  
  if (motorPWM.setFrequency(PWM_FREQ_DEFAULT_HZ) != HAL_OK) error_flag |= ERROR_PWM_CONFIG;
  if (motorPWM.setDeadTime(PWM_DEADTIME_DEFAULT_NS) != HAL_OK) error_flag |= ERROR_PWM_CONFIG;
  
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
  uint32_t tick_count = 0;
  /* USER CODE END 2 */
  
  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    char cmd_line[CMD_MAX_LEN];
    if (read_line_from_ring(&rx_ring, cmd_line, CMD_MAX_LEN)) process_command(cmd_line);
    //if (control_mode != MotorControlMode::MOTOR_STOP && tick_count & 16384) usb_printf("Main Loop Tick: %u\r\n", tick_count);
    tick_count++;
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
    switch (control_mode) {
      case MotorControlMode::MOTOR_FOC_LINEAR:
      case MotorControlMode::MOTOR_FOC_DPWM:
        focTick();
        break;

      case MotorControlMode::MOTOR_FOC_MANUAL:
        focTick();
        break;

      case MotorControlMode::MOTOR_VVVF:
        vvvfRampUp();
        break;

      case MotorControlMode::MOTOR_ALIGN:
        alignRotor();
        break;
        
      default:
        break;
    }
  }
  else if (htim->Instance == TIM16) {
    Encoder::irqHandlerSpeed();
    if (control_mode == MotorControlMode::MOTOR_FOC_LINEAR
     || control_mode == MotorControlMode::MOTOR_FOC_DPWM
     || control_mode == MotorControlMode::MOTOR_FOC_MANUAL) {
      speedControl();
    }

  }
  else if (htim->Instance == TIM6) {
    // 1 kHz control loop interrupt
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
    Encoder::irqHandlerEncoderOverflow();
  }
  else if (htim->Instance == TIM15) {
    Encoder::irqHandlerTimerOverflow();
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

  if (error_flag == 0) {
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

  // Construct ADC data in physical units only if at least one of the relevant print_mask bits is set
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

    ia_max.newValue(fabsf(ia));
  }
  if ((print_mask & (PRINT_IB
                   | PRINT_VA
                   | PRINT_IB_RAW
                   | PRINT_VA_RAW)) != 0) {
    adc2.getLatestDataMean(adc2_raw, FOC_OVERSAMPLING_SIZE);

    ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ib_offset, adc_gain.ib_shunt);
    va = adcToVoltage(adc2_raw[1], 3.3f, 65536, adc_gain.va_gain, 1.65f + adc_gain.va_offset);

    ib_max.newValue(fabsf(ib));
  }
  if ((print_mask & (PRINT_IC
                   | PRINT_IBATT
                   | PRINT_IC_RAW
                   | PRINT_IBATT_RAW)) != 0) {
    adc3.getLatestDataMean(adc3_raw, FOC_OVERSAMPLING_SIZE);

    ic = adcToCurrent(adc3_raw[0], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ic_offset, adc_gain.ic_shunt);
    //ic = -ia - ib;
    ibatt = adcToCurrent(adc3_raw[1], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ibatt_offset, adc_gain.ibatt_shunt);

    ic_max.newValue(fabsf(ic));
    ibatt_max.newValue(fabsf(ibatt));
  }

  // Construct a UTF-8 string, e.g. "rpm 123.45 pos 67.89\r\n"
  char buffer[128];
  int pos = 0;
  if (print_mask & PRINT_RPM) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "rpm %.2f ", encoder.getRPM());
  }
  if (print_mask & PRINT_RPMSP) {
    float val;
    if (control_mode == MotorControlMode::MOTOR_FOC_LINEAR || control_mode == MotorControlMode::MOTOR_FOC_DPWM) val = foc_state.target_rpm;
    else val = target.speed;
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "rpmsp %.2f ", val);
  }
  if (print_mask & PRINT_POS) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "pos %u ", encoder.getPos());
  }
  if (print_mask & PRINT_ELPOS) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "elpos %u ", encoder.getElecPos());
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
  if (print_mask & PRINT_IA_MAX) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ia_max %.2f ", ia_max.getMax());
  }
  if (print_mask & PRINT_IB_MAX) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ib_max %.2f ", ib_max.getMax());
  }
  if (print_mask & PRINT_IC_MAX) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ic_max %.2f ", ic_max.getMax());
  }
  if (print_mask & PRINT_IBATT_MAX) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "ibatt_max %.2f ", ibatt_max.getMax());
  }
  if (print_mask & PRINT_FOC_ID) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "foc_id %.2f ", foc_state.Id);
  }
  if (print_mask & PRINT_FOC_IQ) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "foc_iq %.2f ", foc_state.Iq);
  }
  if (print_mask & PRINT_FOC_IDSP) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "foc_idsp %.2f ", foc_state.Id_ref);
  }
  if (print_mask & PRINT_FOC_IQSP) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "foc_iqsp %.2f ", foc_state.Iq_ref);
  }
  if (print_mask & PRINT_FOC_VD) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "foc_vd %.2f ", foc_state.Vd_cmd);
  }
  if (print_mask & PRINT_FOC_VQ) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "foc_vq %.2f ", foc_state.Vq_cmd);
  }
  if (pos > 0) {
    buffer[pos - 1] = '\n';
    buffer[pos] = '\0';
    CDC_Transmit_HS((uint8_t*)buffer, pos);
  }
}

/**
 * Print telemetry data in binary format over USB.
 * Data structure:
 * [Header: 0xAA 0x55][4-byte print_mask][Data fields...]
 * The data fields are included based on the print_mask bits, and are in the same order as defined in the printTelemetryUTF8 function. Each field is represented in its raw binary format (e.g., float as 4 bytes, uint16_t as 2 bytes).
 * 
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

    ia_max.newValue(fabsf(ia));
  }
  if ((print_mask & (PRINT_IB
                   | PRINT_VA
                   | PRINT_IB_RAW
                   | PRINT_VA_RAW)) != 0) {
    adc2.getLatestDataMean(adc2_raw, FOC_OVERSAMPLING_SIZE);

    ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ib_offset, adc_gain.ib_shunt);
    va = adcToVoltage(adc2_raw[1], 3.3f, 65536, adc_gain.va_gain, 1.65f + adc_gain.va_offset);

    ib_max.newValue(fabsf(ib));
  }
  if ((print_mask & (PRINT_IC
                   | PRINT_IBATT
                   | PRINT_IC_RAW
                   | PRINT_IBATT_RAW)) != 0) {
    adc3.getLatestDataMean(adc3_raw, FOC_OVERSAMPLING_SIZE);

    ic = adcToCurrent(adc3_raw[0], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ic_offset, adc_gain.ic_shunt);
    //ic = -ia - ib;
    ibatt = adcToCurrent(adc3_raw[1], 3.3f, 4096, 50.0f, 1.65f + adc_gain.ibatt_offset, adc_gain.ibatt_shunt);

    ic_max.newValue(fabsf(ic));
    ibatt_max.newValue(fabsf(ibatt));
  }

  // Construct a binary packet
  uint8_t buffer[128];
  uint8_t* ptr = buffer;
  uint32_t mask = print_mask;

  *ptr++ = 0xAA;
  *ptr++ = 0x55;
  
  memcpy(ptr, &mask, 4);
  ptr += 4;

  if (print_mask & PRINT_RPM) {
    float val = encoder.getRPM();
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_RPMSP) {
    float val;
    if (control_mode == MotorControlMode::MOTOR_FOC_LINEAR || control_mode == MotorControlMode::MOTOR_FOC_DPWM) val = foc_state.target_rpm;
    else val = target.speed;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_POS) {
    uint16_t val = encoder.getPos();
    memcpy(ptr, &val, 2);
    ptr += 2;
  }
  if (print_mask & PRINT_ELPOS) {
    uint16_t val = encoder.getElecPos();
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
  if (print_mask & PRINT_IA_MAX) {
    float val = ia_max.getMax();
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IB_MAX) {
    float val = ib_max.getMax();
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IC_MAX) {
    float val = ic_max.getMax();
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_IBATT_MAX) {
    float val = ibatt_max.getMax();
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_FOC_ID) {
    float val = foc_state.Id;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_FOC_IQ) {
    float val = foc_state.Iq;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_FOC_IDSP) {
    float val = foc_state.Id_ref;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_FOC_IQSP) {
    float val = foc_state.Iq_ref;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_FOC_VD) {
    float val = foc_state.Vd_cmd;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (print_mask & PRINT_FOC_VQ) {
    float val = foc_state.Vq_cmd;
    memcpy(ptr, &val, 4);
    ptr += 4;
  }
  if (ptr != buffer) {
      CDC_Transmit_HS(buffer, ptr - buffer);
  }
}

void speedControl(void) {
  static float ramp_speed_increment = 0.0f;
  static uint32_t ramp_tick = 0;

  foc_state.ts_speed = 1.0f / speedControlTimer.getFrequency();
  
  if ((system_flag & FLAG_FOC_RUNNING) == 0) {
    float ramp_down_step = FOC_RAMP_RATE * foc_state.ts_speed;
    if (target.speed > 0.0f){
      target.speed -= ramp_down_step;
      if (target.speed < 0.0f) {
        target.speed = 0.0f;
        motorPWM.stop();
        control_mode = MotorControlMode::MOTOR_STOP;
        relay.write(0);
      }
    }
    else {
      target.speed += ramp_down_step;
      if (target.speed > 0.0f) {
        target.speed = 0.0f;
        motorPWM.stop();
        control_mode = MotorControlMode::MOTOR_STOP;
        relay.write(0);
      }
    }
  }

  // Check if target speed has changed and update foc_state.target_rpm accordingly, with optional ramping
  if (target.speed != foc_state.target_rpm) {
    // Check if new target contains ramp flag
    if (system_flag & FLAG_TARGET_RAMP) {
      // Set up ramp parameters if this is the first tick of a new speed target
      if (system_flag & FLAG_SPEED_RAMP_INIT) {
        float speed_delta = target.speed - foc_state.target_rpm;
        ramp_tick = (uint32_t)(target.time * (float)SPEEDLOOP_FREQ_HZ + 0.5f) + 1;
        ramp_speed_increment = speed_delta / (float)ramp_tick;
        if (ramp_speed_increment > (FOC_RAMP_RATE / SPEEDLOOP_FREQ_HZ)) {
          ramp_speed_increment = FOC_RAMP_RATE / SPEEDLOOP_FREQ_HZ;
          ramp_tick = (uint32_t)(fabsf(speed_delta / ramp_speed_increment)) + 1;
        }
        if (ramp_speed_increment < -(FOC_RAMP_RATE / SPEEDLOOP_FREQ_HZ)) {
          ramp_speed_increment = -FOC_RAMP_RATE / SPEEDLOOP_FREQ_HZ;
          ramp_tick = (uint32_t)(fabsf(speed_delta / ramp_speed_increment)) + 1;
        }
        system_flag &= ~FLAG_SPEED_RAMP_INIT;
      }
      if (ramp_tick > 0) {
        foc_state.target_rpm += ramp_speed_increment;
        ramp_tick--;
      }
      if (ramp_tick == 0) {
        foc_state.target_rpm = target.speed;
      }
    }
    // If no ramping, update target RPM immediately
    else {
      foc_state.target_rpm = target.speed;
    }
  }
  else if (system_flag & FLAG_TARGET_RAMP) {
    ramp_speed_increment = 0.0f;
    system_flag &= ~FLAG_TARGET_RAMP;
  }

  // Calculate speed delta using a ramp function to limit acceleration
  static float p_rpm = 0.0f;
  foc_state.rpm = encoder.getRPM() * 0.2f + p_rpm * 0.8f;
  p_rpm = foc_state.rpm;
  float omega_m = foc_state.rpm * RPM_TO_RAD_S;
  float omega_target = foc_state.target_rpm * RPM_TO_RAD_S;
  const float max_step = FOC_RAMP_RATE * RPM_TO_RAD_S / (float)SPEEDLOOP_FREQ_HZ;
  float speed_delta = omega_target - foc_state.omega_ref;

  // Limit the speed delta to the maximum step size to ensure smooth acceleration
  if (speed_delta >  max_step) speed_delta =  max_step;
  if (speed_delta < -max_step) speed_delta = -max_step;
  foc_state.omega_ref += speed_delta;

  // Update the speed PI controller to adjust Iq reference based on the speed error
  float err_sp = foc_state.omega_ref - omega_m;
  float new_Iq_ref = PI_update(&foc_state.pi_speed, err_sp, foc_state.ts_speed);

  float U_max_fw   = foc_state.Vdc / SQRT3;
  float u_mag_prev = foc_state.u_mag;
  float new_Id_ref;
  if (fabsf(foc_state.omega_m) > 20.0f) {
      float fw_error = (U_max_fw - u_mag_prev) / fabsf(foc_state.omega_m);
      if (fw_error < 0.0f || foc_state.Id_ref < 0.0f) {
        // Only integrates when FW is requested or is already inside FW
        new_Id_ref = PI_update(&foc_state.pi_fw, fw_error, foc_state.ts_speed);
      }
      else {
        // Zero the integrator if FW is not needed
        // Method 1：Reset PI（Simple, faster）
        new_Id_ref = 0.0f;
        PI_reset(&foc_state.pi_fw);
        // Method 2：Exponential decay（Smoother）
        // foc_state.pi_fw.integrator *= 0.99f;
        // new_Id_ref = foc_state.pi_fw.integrator;
      }
  }else {
      new_Id_ref = 0.0f;
      PI_reset(&foc_state.pi_fw);
  }

  // In manual FOC mode, current setpoints are controlled directly by the user
  if (control_mode != MotorControlMode::MOTOR_FOC_MANUAL) {
    __disable_irq();
    foc_state.Iq_ref = new_Iq_ref;
    //foc_state.Id_ref = new_Id_ref;
    __enable_irq();
  }
}

void alignRotor(void) {
  if (control_mode != MotorControlMode::MOTOR_ALIGN) return;
  
  static uint16_t p_pos = 0;
  static uint16_t settlement_counter = 0;
  static float rpm = 0.0f;  
  static float angle = 0.0f;

  // Step 1: If electrical zero is not aligned, apply alignment voltage and monitor encoder position for settling
  if ((system_flag & FLAG_ELEC_ZERO_ALIGNED) == 0) {

    if ((system_flag & FLAG_ROTOR_ALIGNING) == 0) {
      uint16_t vdc_sample[FOC_OVERSAMPLING_SIZE];
      adc1.getLatestChannel(2, vdc_sample, FOC_OVERSAMPLING_SIZE);
      float Vdc = adcToVoltage(fastAverage(vdc_sample, FOC_OVERSAMPLING_SIZE), 3.3f, 65536,
                              adc_gain.vbatt_gain, adc_gain.vbatt_offset);

      float dutyA, dutyB, dutyC;
      foc_state.ts = 1.0f / (float)motorPWM.getFrequency();
      focAlignZero(&foc_state, MOTOR_ALIGNMENT_VOLTAGE, Vdc, &dutyA, &dutyB, &dutyC);
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

    if (settlement_counter > MOTOR_ALIGNMENT_POS_WINDOW) {
      encoder.elecZeroAlign();

      system_flag &= ~FLAG_ROTOR_ALIGNING;
      system_flag |= FLAG_ELEC_ZERO_ALIGNED;

      p_pos = 0;
      settlement_counter = 0;

      usb_printf("Electrical Zero Angle Aligned\n");
    }
  }

  // Step 2: Once aligned, ramp up speed using VVVF to search for index pulse for absolute position reference
  else if (!encoder.is_synchronized_) {
    const uint32_t ramp_up = VVVF_RAMP_UP_SPEED; // RPM/s
    // Increment aligned with interrupt frequency
    uint32_t frequency = motorPWM.getFrequency();
    float step_increment = (float)ramp_up / frequency;

    // Ramp up speed
    rpm += step_increment;

    // Calculate electrical angle
    float electrical_freq = rpm / 60.0f * MOTOR_POLE_PAIRS;

    float delta_angle = 2.0f * M_PI * electrical_freq / frequency;
    angle += delta_angle * MOTOR_ROTATION_DIRECTION;
    if (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;

    // Calculate voltage amplitude
    float amplitude;
    const float MIN_VOLTAGE = 0.15f;  // Boost start voltage
    const float KNEE_RPM = 1000.0f;   // Knee point (RPM)
    amplitude = MIN_VOLTAGE + (1.0f - MIN_VOLTAGE) * (rpm / KNEE_RPM);

    // Limit output range
    if (amplitude > 1.0f) amplitude = 1.0f;

    float dutyA = 0.5f + amplitude * 0.5f * sinf(angle);
    float dutyB = 0.5f + amplitude * 0.5f * sinf(angle - 2.0f * M_PI / 3.0f);
    float dutyC = 0.5f + amplitude * 0.5f * sinf(angle + 2.0f * M_PI / 3.0f);

    motorPWM.setDuty(dutyA, dutyB, dutyC);
  }

  else {
    control_mode = MotorControlMode::MOTOR_STOP;
    motorPWM.stop();
    relay.write(0);
    rpm = 0.0f;
    angle = 0.0f;

    usb_printf("Mechanical Alignment Complete\n");
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
void startUpSequence(MotorControlMode mode) {
  if (control_mode != MotorControlMode::MOTOR_STARTUP) return;

  if ((system_flag & FLAG_ELEC_ZERO_ALIGNED && encoder.is_synchronized_) == 0) {
    control_mode = MotorControlMode::MOTOR_ALIGN;
    usb_printf("Starting rotor alignment sequence...\r\n");
    alignRotor();
    return;
  }

  switch (mode) {
    case MotorControlMode::MOTOR_FOC_LINEAR:
      foc_reset(&foc_state);
      // Set initial FOC state for linear startup
      if (target.speed == 0.0f) target.speed = FOC_INITIAL_RPM;
      control_mode = MotorControlMode::MOTOR_FOC_LINEAR;
      system_flag |= FLAG_FOC_RUNNING;
      speedControl();
      focTick();
      usb_printf("Starting FOC linear startup sequence...\r\n");
      break;
    case MotorControlMode::MOTOR_VVVF:
      foc_reset(&foc_state);
      if (target.speed == 0.0f) target.speed = FOC_INITIAL_RPM;
      hallsensor.read();
      system_flag |= FLAG_VVVF_RAMP_UP;
      control_mode = MotorControlMode::MOTOR_VVVF;
      vvvfRampUp();
      usb_printf("Starting VVVF ramp-up sequence...\r\n");
      break;
    default:
      usb_printf("Invalid startup mode selected. Defaulting to FOC linear startup.\r\n");
      mode = MotorControlMode::MOTOR_FOC_LINEAR;
      break;
  }

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

  // Initialize on zero speed starting
  if ((system_flag & FLAG_VVVF_RUNNING) == 0) {
    rpm = 0.0f;
    angle = 0.0f;
    system_flag |= FLAG_VVVF_RUNNING;
  }

  // Audible frequency adjustment
  if (system_flag & FLAG_AUDIBLE) {
    if (rpm < 30.0f) {
      motorPWM.setFrequency(293.66f);
    }
    else if (rpm < 60.0f) {
      motorPWM.setFrequency(329.63f);
    }
    else if (rpm < 90.0f) {
      motorPWM.setFrequency(349.23f);
    }
    else if (rpm < 120.0f) {
      motorPWM.setFrequency(392.00f);
    }
    else if (rpm < 150.0f) {
      motorPWM.setFrequency(440.00f);
    }
    else if (rpm < 180.0f) {
      motorPWM.setFrequency(493.88f);
    }
    else if (rpm < 210.0f) {
      motorPWM.setFrequency(523.25f);
    }
    else if (rpm < 240.0f) {
      motorPWM.setFrequency(587.33f);
    }
    else if (rpm < 270.0f) {
      motorPWM.setFrequency(659.26f);
    }
    else if (rpm < 300.0f) {
      motorPWM.setFrequency(698.46f);
    }
    else if (rpm < 330.0f) {
      motorPWM.setFrequency(783.99f);
    }
    else if (rpm < 360.0f) {
      motorPWM.setFrequency(880.00f);
    }
    else if (rpm < 600.0f) {
      motorPWM.setFrequency(900.00f);
    }
    else {
      motorPWM.setFrequency((uint32_t)rpm * 2);
    }
  }
  else {
    if (motorPWM.getFrequency() != PWM_FREQ_DEFAULT_HZ) motorPWM.setFrequency(PWM_FREQ_DEFAULT_HZ);
  }

  // Increment aligned with interrupt frequency
  uint32_t frequency = motorPWM.getFrequency();
  float step_increment = (float)ramp_up / frequency;

  // Ramp up or down the speed
  if (system_flag & FLAG_VVVF_RAMP_UP) {
    if ((rpm < VVVF_MAX_RPM >> 1) && (rpm < target.speed / 2)) {
      rpm += step_increment;
      if (rpm >= VVVF_MAX_RPM >> 1) {
        rpm = VVVF_MAX_RPM >> 1;
      }
      else if (rpm >= (target.speed / 2)) {
        rpm = target.speed / 2;
      }
    }
    else if (rpm > (target.speed / 2)) {
      rpm -= step_increment;
      if (rpm <= (target.speed / 2)) {
        rpm = target.speed / 2;
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
  angle += delta_angle * MOTOR_ROTATION_DIRECTION;
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
  float dutyB = 0.5f + amplitude * 0.5f * sinf(angle - 2.0f * M_PI / 3.0f);
  float dutyC = 0.5f + amplitude * 0.5f * sinf(angle + 2.0f * M_PI / 3.0f);

  motorPWM.setDuty(dutyA, dutyB, dutyC);

  if (FOC_ALLOWED && encoder.is_synchronized_ && encoder.is_zeroed_ && rpm >= VVVF_THRESHOLD_RPM >> 1) {
    system_flag &= ~FLAG_VVVF_RUNNING;
    system_flag |= FLAG_FOC_RUNNING;
    foc_state.target_rpm = FOC_INITIAL_RPM;
    control_mode = MotorControlMode::MOTOR_FOC_LINEAR;
    usb_printf("Entering FOC mode\r\n");
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

  float dutyA = (a_state == 1) ? SIXSTEP_DUTYCYCLE : (a_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
  float dutyB = (b_state == 1) ? SIXSTEP_DUTYCYCLE : (b_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
  float dutyC = (c_state == 1) ? SIXSTEP_DUTYCYCLE : (c_state == 0) ? (1.0f - SIXSTEP_DUTYCYCLE) : -1.0f;
  
  motorPWM.setDuty(dutyA, dutyB, dutyC);
}

void clearRunningFlags(void) {
    system_flag &= ~FLAG_ROTOR_ALIGNING;
    system_flag &= ~FLAG_VVVF_RUNNING;
    system_flag &= ~FLAG_SIXSTEP_RUNNING;
    system_flag &= ~FLAG_FOC_RUNNING;

    foc_reset(&foc_state);
}

void loadAdcCalibration(ADCGain_t* adc_gain, uint8_t preset_num) {
  switch (preset_num) {
    case 1:
      adc_gain->ia_shunt = ADC_IA_SHUNT_1;
      adc_gain->ib_shunt = ADC_IB_SHUNT_1;
      adc_gain->ic_shunt = ADC_IC_SHUNT_1;
      adc_gain->ia_offset = ADC_IA_OFFSET_1;
      adc_gain->ib_offset = ADC_IB_OFFSET_1;
      adc_gain->ic_offset = ADC_IC_OFFSET_1;
      adc_gain->va_gain = ADC_VA_GAIN_1;
      adc_gain->vb_gain = ADC_VB_GAIN_1;
      adc_gain->va_offset = ADC_VA_OFFSET_1;
      adc_gain->vb_offset = ADC_VB_OFFSET_1;
      adc_gain->ibatt_shunt = ADC_IBATT_SHUNT_1;
      adc_gain->ibatt_offset = ADC_IBATT_OFFSET_1;
      adc_gain->vbatt_gain = ADC_VBATT_GAIN_1;
      adc_gain->vbatt_offset = ADC_VBATT_OFFSET_1;
      adc_gain->preset = 1;
      break;
    case 2:
      adc_gain->ia_shunt = ADC_IA_SHUNT_2;
      adc_gain->ib_shunt = ADC_IB_SHUNT_2;
      adc_gain->ic_shunt = ADC_IC_SHUNT_2;
      adc_gain->ia_offset = ADC_IA_OFFSET_2;
      adc_gain->ib_offset = ADC_IB_OFFSET_2;
      adc_gain->ic_offset = ADC_IC_OFFSET_2;
      adc_gain->va_gain = ADC_VA_GAIN_2;
      adc_gain->vb_gain = ADC_VB_GAIN_2;
      adc_gain->va_offset = ADC_VA_OFFSET_2;
      adc_gain->vb_offset = ADC_VB_OFFSET_2;
      adc_gain->ibatt_shunt = ADC_IBATT_SHUNT_2;
      adc_gain->ibatt_offset = ADC_IBATT_OFFSET_2;
      adc_gain->vbatt_gain = ADC_VBATT_GAIN_2;
      adc_gain->vbatt_offset = ADC_VBATT_OFFSET_2;
      adc_gain->preset = 2;
      break;
    default:
      adc_gain->ia_shunt = 0.0f;
      adc_gain->ib_shunt = 0.0f;
      adc_gain->ic_shunt = 0.0f;
      adc_gain->ia_offset = 0.0f;
      adc_gain->ib_offset = 0.0f;
      adc_gain->ic_offset = 0.0f;
      adc_gain->va_gain = 1.0f;
      adc_gain->vb_gain = 1.0f;
      adc_gain->va_offset = 0.0f;
      adc_gain->vb_offset = 0.0f;
      adc_gain->ibatt_shunt = 0.0f;
      adc_gain->ibatt_offset = 0.0f;
      adc_gain->vbatt_gain = 1.0f;
      adc_gain->vbatt_offset = 0.0f;
      adc_gain->preset = 0;
      break;
  }
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
    clearRunningFlags();
    relay.write(1);
    if (strcmp(argv[1], "foc") == 0) startUpSequence(MotorControlMode::MOTOR_FOC_LINEAR);
    else if (strcmp(argv[1], "vvvf") == 0) startUpSequence(MotorControlMode::MOTOR_VVVF);
    else startUpSequence(MotorControlMode::MOTOR_FOC_LINEAR);
    usb_printf("Starting\r\n");
}

/**
 * @brief Command handler for "stop" command.
 * @note Stops the motor immediately and resets the control state.
 * @note If currently in VVVF ramp-up, it will start ramping down instead of an immediate stop.
 */
void cmd_stop(int argc, char** argv) {
    if ((control_mode == MotorControlMode::MOTOR_FOC_DPWM || control_mode == MotorControlMode::MOTOR_FOC_LINEAR) && (system_flag & FLAG_FOC_RUNNING)) {
        system_flag &= ~FLAG_FOC_RUNNING; // Start FOC ramp down
        usb_printf("FOC ramping down\r\n");
    }
    else if (control_mode == MotorControlMode::MOTOR_VVVF && system_flag & FLAG_VVVF_RAMP_UP) {
        system_flag &= ~FLAG_VVVF_RAMP_UP; // Start ramp down
        usb_printf("VVVF ramping down\r\n");
    }
    else {
        control_mode = MotorControlMode::MOTOR_STOP;
        clearRunningFlags();
        motorPWM.stop();
        foc_reset(&foc_state);
        relay.write(0);
        usb_printf("Stopping\r\n");
    }
}

void cmd_align(int argc, char** argv) {
    if (strcmp(argv[1], "reset") == 0) {
        system_flag &= ~FLAG_ELEC_ZERO_ALIGNED;
        encoder.is_synchronized_ = false;
        encoder.is_zeroed_ = false;
        usb_printf("Alignment resetted\r\n");
        return;
    }
    if (control_mode == MotorControlMode::MOTOR_PROTECTION) {protectionModePrint(); return;}
    if (system_flag & FLAG_ELEC_ZERO_ALIGNED) {usb_printf("Electrical zero already aligned\r\n"); return;}
    if (control_mode == MotorControlMode::MOTOR_ALIGN) {usb_printf("Already aligning, please wait\r\n"); return;}
    control_mode = MotorControlMode::MOTOR_ALIGN;
    clearRunningFlags();
    relay.write(1);
    alignRotor();
    usb_printf("Starting\r\n");
}

/**
 * @brief Command handler for "reset" command.
 * @note Resets the entire system to a known safe state, stopping the motor and clearing any active control modes or flags.
 */
void cmd_reset(int argc, char** argv) {
    control_mode = MotorControlMode::MOTOR_STOP;
    clearRunningFlags();
    error_flag &= ~ERROR_OVERCURRENT;
    motorPWM.stop();
    led_red.write(0);
    foc_reset(&foc_state);
    relay.write(0);
    usb_printf("Resetting\r\n");
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
    else if (strcmp(argv[1], "manual") == 0) {
        control_mode = MotorControlMode::MOTOR_FOC_MANUAL;
        clearRunningFlags();
        usb_printf("FOC in manual mode, use with care\r\n");
    }
    else if (strcmp(argv[1], "vd") == 0) {
        if (control_mode != MotorControlMode::MOTOR_FOC_MANUAL) {
            usb_printf("Command only valid in FOC manual mode\r\n");
            return;
        }
        foc_state.Vd_cmd = atof(argv[2]);
        if ((system_flag & FLAG_FOC_RUNNING) == 0) {
            system_flag |= FLAG_FOC_RUNNING;
            relay.write(1);
            focTick();
        }
        focResetPI(&foc_state);
        usb_printf("FOC Vd set to %.2f V\r\n", foc_state.Vd_cmd);
    }
    else if (strcmp(argv[1], "vq") == 0) {
        if (control_mode != MotorControlMode::MOTOR_FOC_MANUAL) {
            usb_printf("Command only valid in FOC manual mode\r\n");
            return;
        }
        foc_state.Vq_cmd = atof(argv[2]);
        if ((system_flag & FLAG_FOC_RUNNING) == 0) {
            system_flag |= FLAG_FOC_RUNNING;
            relay.write(1);
            focTick();
        }
        focResetPI(&foc_state);
        usb_printf("FOC Vq set to %.2f V\r\n", foc_state.Vq_cmd);
    }
    else if (strcmp(argv[1], "id") == 0) {
        if (control_mode != MotorControlMode::MOTOR_FOC_MANUAL) {
            usb_printf("Command only valid in FOC manual mode\r\n");
            return;
        }
        foc_state.Id_ref = atof(argv[2]);
        if ((system_flag & FLAG_FOC_RUNNING) == 0) {
            system_flag |= FLAG_FOC_RUNNING;
            relay.write(1);
            focTick();
        }
        focResetPI(&foc_state);
        usb_printf("FOC Id set to %.3f A\r\n", foc_state.Id_ref);
    }
    else if (strcmp(argv[1], "iq") == 0) {
        if (control_mode != MotorControlMode::MOTOR_FOC_MANUAL) {
            usb_printf("Command only valid in FOC manual mode\r\n");
            return;
        }
        foc_state.Iq_ref = atof(argv[2]);
        if ((system_flag & FLAG_FOC_RUNNING) == 0) {
            system_flag |= FLAG_FOC_RUNNING;
            relay.write(1);
            focTick();
        }
        focResetPI(&foc_state);
        usb_printf("FOC Iq set to %.3f A\r\n", foc_state.Iq_ref);
    }
    else {
        usb_printf("Unknown FOC command\r\n"); 
    }
}

/**
 * @brief Command handler for "sixstep" command.
 * @note Starts six-step commutation control mode.
 */
void cmd_sixstep(int argc, char** argv) {
    if (control_mode == MotorControlMode::MOTOR_PROTECTION) {protectionModePrint(); return;}
    control_mode = MotorControlMode::MOTOR_SIX_STEP;
    clearRunningFlags();
    relay.write(1);
    sixStepCommutation();
    usb_printf("Six-step running\r\n");
}

/**
 * @brief Command handler for "speed" command.
 * @note Set the speed setpoint for FOC speed loop.
 */
void cmd_speed(int argc, char** argv) {
    float speed = atof(argv[1]);
    if (speed < -5000.0f || speed > 5000.0f) {
        usb_printf("Invalid speed value: %s\r\n", argv[1]);
        return;
    }

    // Handle option time parameter
    float time = 0.0f;
    if (argc >= 3) {
        time = atof(argv[2]);
        if (time <= 0.0f) {
            usb_printf("Invalid time value (must be positive): %s\r\n", argv[2]);
            return;
        }
    }
    target.speed = speed;

    // Setting system flag
    if (argc >= 3) {
        system_flag |= FLAG_TARGET_RAMP;
        system_flag |= FLAG_SPEED_RAMP_INIT;
        target.time = time;
        usb_printf("Speed set to %.2f, reaching in %.3f seconds\r\n", speed, time);
    } else {
        // Disable ramping if no time parameter is provided, making the speed change instantaneous
        system_flag &= ~FLAG_TARGET_RAMP;
        system_flag &= ~FLAG_SPEED_RAMP_INIT;
        target.time = 0.0f;
        usb_printf("Speed set to %.2f\r\n", speed);
    }
}


/**
 * @brief Command handler for "torque" command.
 * @note Set the torque setpoint for FOC control.
 */
void cmd_torque(int argc, char** argv) {
    float torque = atof(argv[1]);
    if (torque < -10.0f || torque > 10.0f) {
        usb_printf("Invalid torque value: %s\r\n", argv[1]);
        return;
    }

    // Handle option time parameter
    float time = 0.0f;
    if (argc >= 3) {
        time = atof(argv[2]);
        if (time <= 0.0f) {
            usb_printf("Invalid time value (must be positive): %s\r\n", argv[2]);
            return;
        }
    }
    target.torque = torque;

    // Setting system flag
    if (argc >= 3) {
        system_flag |= FLAG_TARGET_RAMP;
        system_flag |= FLAG_SPEED_RAMP_INIT;
        target.time = time;
        usb_printf("Torque set to %.2f, reaching in %.3f seconds\r\n", torque, time);
    } else {
        // Disable ramping if no time parameter is provided, making the torque change instantaneous
        system_flag &= ~FLAG_TARGET_RAMP;
        system_flag &= ~FLAG_SPEED_RAMP_INIT;
        target.time = 0.0f;
        usb_printf("Torque set to %.2f\r\n", torque);
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
            
            control_mode = MotorControlMode::MOTOR_MANUAL;
            clearRunningFlags();
            relay.write(1);
            motorPWM.setDuty(values[0], values[1], values[2]);

            usb_printf("Duty set to A=%.2f B=%.2f C=%.2f\r\n", values[0], values[1], values[2]);
        } else {
            usb_printf("Duty values out of range [-1,1]: A=%.2f B=%.2f C=%.2f\r\n", values[0], values[1], values[2]);
        }
    } else {
        usb_printf("Invalid duty format. Usage: duty 0.3,0.3,0.3\r\n");
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
        clearRunningFlags();

        uint8_t hall_state = hallsensor.getState();
        usb_printf("Vector %d applied, Hall=%d\r\n", vec_num, hall_state);
    } else {
        usb_printf("Invalid vector. Use 0-5.\r\n");
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
    bool is_query = (strcmp(argv[3], "?") == 0);
    float value = is_query ? 0.0f : atof(argv[3]);

    bool success = false;
    float original = 0.0f;

    float* target = nullptr;

    if (strcmp(subsys, "speed") == 0) {
        if (strcmp(param, "p") == 0) {
            target = &foc_state.pi_speed.kp;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            target = &foc_state.pi_speed.ki;
            success = true;
        }
    } 
    else if (strcmp(subsys, "id") == 0) {
        if (strcmp(param, "p") == 0) {
            target = &foc_state.pi_d.kp;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            target = &foc_state.pi_d.ki;
            success = true;
        }
    } 
    else if (strcmp(subsys, "iq") == 0) {
        if (strcmp(param, "p") == 0) {
            target = &foc_state.pi_q.kp;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            target = &foc_state.pi_q.ki;
            success = true;
        }
    } 
    else if (strcmp(subsys, "fw") == 0) {
        if (strcmp(param, "p") == 0) {
            target = &foc_state.pi_fw.kp;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            target = &foc_state.pi_fw.ki;
            success = true;
        }
    } 
    else if (strcmp(subsys, "gain") == 0) {
        if (strcmp(param, "ia") == 0) {
            target = &adc_gain.ia_shunt;
            success = true;
        } else if (strcmp(param, "ib") == 0) {
            target = &adc_gain.ib_shunt;
            success = true;
        } else if (strcmp(param, "ic") == 0) {
            target = &adc_gain.ic_shunt;
            success = true;
        } else if (strcmp(param, "va") == 0) {
            target = &adc_gain.va_gain;
            success = true;
        } else if (strcmp(param, "vb") == 0) {
            target = &adc_gain.vb_gain;
            success = true;
        } else if (strcmp(param, "ibatt") == 0) {
            target = &adc_gain.ibatt_shunt;
            success = true;
        } else if (strcmp(param, "vbatt") == 0) {
            target = &adc_gain.vbatt_gain;
            success = true;
        }
    } 
    else if (strcmp(subsys, "offset") == 0) {
        if (strcmp(param, "ia") == 0) {
            target = &adc_gain.ia_offset;
            success = true;
        } else if (strcmp(param, "ib") == 0) {
            target = &adc_gain.ib_offset;
            success = true;
        } else if (strcmp(param, "ic") == 0) {
            target = &adc_gain.ic_offset;
            success = true;
        } else if (strcmp(param, "va") == 0) {
            target = &adc_gain.va_offset;
            success = true;
        } else if (strcmp(param, "vb") == 0) {
            target = &adc_gain.vb_offset;
            success = true;
        } else if (strcmp(param, "ibatt") == 0) {
            target = &adc_gain.ibatt_offset;
            success = true;
        } else if (strcmp(param, "vbatt") == 0) {
            target = &adc_gain.vbatt_offset;
            success = true;
        }
    }

    if (success && target != nullptr) {
        if (is_query) {
            usb_printf("%s %s is %.6f\r\n", subsys, param, *target);
        } else {
            original = *target;
            *target = value;
        }
        usb_printf("%s %s set to %.6f (was %.6f)\r\n", subsys, param, value, original);
    } else {
        usb_printf("Unknown parameter '%s' or subsystem '%s'\r\n", param, subsys);
    }
}

/**
 * @brief Command handler for "increment" command.
 * @note Increments the specified parameter by a given value, allowing for fine-tuning without needing to know the current value. Usage: "increment speed p 0.1" to increase the speed loop P gain by 0.1.
 */
void cmd_increment(int argc, char** argv) {
    // "increment speed p 0.1" -> argv[1]="speed", argv[2]="p", argv[3]="0.1"
    char* subsys = argv[1];
    char* param = argv[2];
    float value = atof(argv[3]);

    bool success = false;
    float original = 0.0f;

    if (strcmp(subsys, "speed") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_speed.kp;
            foc_state.pi_speed.kp += value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_speed.ki;
            foc_state.pi_speed.ki += value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "id") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_d.kp;
            foc_state.pi_d.kp += value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_d.ki;
            foc_state.pi_d.ki += value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "iq") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_q.kp;
            foc_state.pi_q.kp += value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_q.ki;
            foc_state.pi_q.ki += value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "fw") == 0) {
        if (strcmp(param, "p") == 0) {
            original = foc_state.pi_fw.kp;
            foc_state.pi_fw.kp += value;
            success = true;
        }
        else if (strcmp(param, "i") == 0) {
            original = foc_state.pi_fw.ki;
            foc_state.pi_fw.ki += value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "flux") == 0) {
        if (strcmp(param, "p") == 0) { 
        }
    } 
    else if (strcmp(subsys, "gain") == 0) {
        if (strcmp(param, "ia") == 0) {
            original = adc_gain.ia_shunt;
            adc_gain.ia_shunt += value;
            success = true;
        } else if (strcmp(param, "ib") == 0) {
            original = adc_gain.ib_shunt;
            adc_gain.ib_shunt += value;
            success = true;
        } else if (strcmp(param, "ic") == 0) {
            original = adc_gain.ic_shunt;
            adc_gain.ic_shunt += value;
            success = true;
        } else if (strcmp(param, "va") == 0) {
            original = adc_gain.va_gain;
            adc_gain.va_gain += value;
            success = true;
        } else if (strcmp(param, "vb") == 0) {
            original = adc_gain.vb_gain;
            adc_gain.vb_gain += value;
            success = true;
        } else if (strcmp(param, "ibatt") == 0) {
            original = adc_gain.ibatt_shunt;
            adc_gain.ibatt_shunt += value;
            success = true;
        } else if (strcmp(param, "vbatt") == 0) {
            original = adc_gain.vbatt_gain;
            adc_gain.vbatt_gain += value;
            success = true;
        }
    } 
    else if (strcmp(subsys, "offset") == 0) {
        if (strcmp(param, "ia") == 0) {
            original = adc_gain.ia_offset;
            adc_gain.ia_offset += value;
            success = true;
        } else if (strcmp(param, "ib") == 0) {
            original = adc_gain.ib_offset;
            adc_gain.ib_offset += value;
            success = true;
        } else if (strcmp(param, "ic") == 0) {
            original = adc_gain.ic_offset;
            adc_gain.ic_offset += value;
            success = true;
        } else if (strcmp(param, "va") == 0) {
            original = adc_gain.va_offset;
            adc_gain.va_offset += value;
            success = true;
        } else if (strcmp(param, "vb") == 0) {
            original = adc_gain.vb_offset;
            adc_gain.vb_offset += value;
            success = true;
        } else if (strcmp(param, "ibatt") == 0) {
            original = adc_gain.ibatt_offset;
            adc_gain.ibatt_offset += value;
            success = true;
        } else if (strcmp(param, "vbatt") == 0) {
            original = adc_gain.vbatt_offset;
            adc_gain.vbatt_offset += value;
            success = true;
        }
    }

    if (success) {
        usb_printf("%s %s set to %.4f (was %.4f)\r\n", subsys, param, (original + value), original);
    } else {
        usb_printf("Unknown parameter '%s' or subsystem '%s'\r\n", param, subsys);
    }
}

/**
 * @brief Command handler for "board" command.
 * @note Loads predefined ADC calibration presets for different motor driver boards. Usage: "board 1" to load preset 1, "board 2" to load preset 2. If no argument is given, it prints the currently loaded preset number.
 */
void cmd_board(int argc, char** argv) {
    if (argc != 2) {
        usb_printf("Current ADC calibration preset: %d\r\n", adc_gain.preset);
        return;
    }
    int id = atoi(argv[1]);
    loadAdcCalibration(&adc_gain, id);
    usb_printf("Loaded ADC calibration preset for board %d\r\n", adc_gain.preset);
}

/**
 * @brief Command handler for "log" command.
 * @note Allows dynamic configuration of which variables to include in the data log output andoutput encoding.
 */
void cmd_log(int argc, char** argv) {
    char* action = argv[1];

    if (strcmp(action, "preset") == 0) {
        if (argc < 3) {
            usb_printf("Missing preset number\r\n");
            return;
        }
        
        int preset_id = atoi(argv[2]); // Safe conversion of "1", "2", etc.
        
        switch (preset_id) {
            case 1: // Preset 1: FOC Tuning
                print_mask = PRINT_RPM | PRINT_IA | PRINT_IB | PRINT_IC | PRINT_VBATT;
                usb_printf("Preset %d active\r\n", preset_id);
                break;
                
            case 2: // Preset 2: Raw Sensor Calibration
                print_mask = PRINT_IA | PRINT_IB | PRINT_IC | PRINT_IA_RAW | PRINT_IB_RAW | PRINT_IC_RAW;
                usb_printf("Preset %d active\r\n", preset_id);
                break;
                
            case 3: // Preset 3: Power Monitoring
                print_mask = PRINT_VBATT | PRINT_IBATT | PRINT_DUTY_A | PRINT_DUTY_B | PRINT_DUTY_C;
                usb_printf("Preset %d active\r\n", preset_id);
                break;

            case 4:
                print_mask = PRINT_RPM | PRINT_IA | PRINT_IB | PRINT_IC | PRINT_FOC_ID | PRINT_FOC_IQ | PRINT_FOC_VD | PRINT_FOC_VQ | PRINT_FOC_IDSP | PRINT_FOC_IQSP;
                usb_printf("Preset %d active\r\n", preset_id);
                break;
                
            default:
                usb_printf("Unknown preset. Try 1, 2, or 3\r\n");
                break;
        }
        return; // Exit here
    }

    else if (strcmp(action, "add") == 0 || strcmp(action, "rm") == 0) {
        if (argc < 3) {
            usb_printf("Missing variable name\r\n");
            return;
        }
        char* token = argv[2];
        uint32_t flag = 0;
        
        if      (strcmp(token, "rpm") == 0) flag = PRINT_RPM;
        else if (strcmp(token, "rpmsp") == 0) flag = PRINT_RPMSP;
        else if (strcmp(token, "pos") == 0) flag = PRINT_POS;
        else if (strcmp(token, "elpos") == 0) flag = PRINT_ELPOS;
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
        else if (strcmp(token, "ia_max") == 0) flag = PRINT_IA_MAX;
        else if (strcmp(token, "ib_max") == 0) flag = PRINT_IB_MAX;
        else if (strcmp(token, "ic_max") == 0) flag = PRINT_IC_MAX;
        else if (strcmp(token, "ibatt_max") == 0) flag = PRINT_IBATT_MAX;
        else if (strcmp(token, "id") == 0) flag = PRINT_FOC_ID;
        else if (strcmp(token, "iq") == 0) flag = PRINT_FOC_IQ;
        else if (strcmp(token, "idsp") == 0) flag = PRINT_FOC_IDSP;
        else if (strcmp(token, "iqsp") == 0) flag = PRINT_FOC_IQSP;
        else if (strcmp(token, "vd") == 0) flag = PRINT_FOC_VD;
        else if (strcmp(token, "vq") == 0) flag = PRINT_FOC_VQ;
        else if (strcmp(token, "all") == 0 && strcmp(action, "rm") == 0) {
            print_mask = 0;
            usb_printf("All variables removed\r\n");
            return;
        } else {
            usb_printf("Unknown variable\r\n");
            return;
        }

        if (strcmp(action, "add") == 0) {
            print_mask |= flag;
            usb_printf("Variable %s added\r\n", token);
        } else {
            print_mask &= ~flag;
            usb_printf("Variable %s removed\r\n", token);
        }
    } 
    else if (strcmp(action, "utf8") == 0) {
        print_format = PrintFormat::PRINT_UTF8;
        usb_printf("Print format set to UTF8\r\n");
    } 
    else if (strcmp(action, "bin") == 0) {
        print_format = PrintFormat::PRINT_BINARY;
        usb_printf("Print format set to BINARY\r\n");
    } 
    else {
        usb_printf("Invalid log action\r\n");
    }
}

void cmd_audible(int argc, char** argv) {
    if (system_flag & FLAG_AUDIBLE) {
        system_flag &= ~FLAG_AUDIBLE;
        usb_printf("Audible frequency disabled\r\n");
        motorPWM.setFrequency(PWM_FREQ_DEFAULT_HZ);
    } else {
        system_flag |= FLAG_AUDIBLE;
        usb_printf("Audible frequency enabled\r\n");
    }
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

void focTick(void) {
    /* uint16_t adc1_raw[3];
    uint16_t adc2_raw[2];
    uint16_t adc3_raw[2];

    adc1.getLatestDataMean(adc1_raw, FOC_OVERSAMPLING_SIZE);
    adc2.getLatestDataMean(adc2_raw, FOC_OVERSAMPLING_SIZE);
    adc3.getLatestDataMean(adc3_raw, FOC_OVERSAMPLING_SIZE);

    float ia = adcToCurrent(adc1_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ia_offset, adc_gain.ia_shunt);
    float ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f, 1.65f + adc_gain.ib_offset, adc_gain.ib_shunt);
    float ic = adcToCurrent(adc3_raw[0], 3.3f,  4096, 50.0f, 1.65f + adc_gain.ic_offset, adc_gain.ic_shunt);
    float vdc = adcToVoltage(adc1_raw[2], 3.3f, 65536, adc_gain.vbatt_gain, adc_gain.vbatt_offset); */

    float ia = adcToCurrent(adc1.getLatestChannelMean(0, FOC_OVERSAMPLING_SIZE), 3.3f, 65536, 50.0f, 1.65f + adc_gain.ia_offset, adc_gain.ia_shunt);
    float ib = adcToCurrent(adc2.getLatestChannelMean(0, FOC_OVERSAMPLING_SIZE), 3.3f, 65536, 50.0f, 1.65f + adc_gain.ib_offset, adc_gain.ib_shunt);
    float ic = adcToCurrent(adc3.getLatestChannelMean(0, FOC_OVERSAMPLING_SIZE), 3.3f, 4096, 50.0f, 1.65f + adc_gain.ic_offset, adc_gain.ic_shunt);
    //float ic = -ia - ib;
    float vdc = adcToVoltage(adc1.getLatestChannelMean(2, FOC_OVERSAMPLING_SIZE), 3.3f, 65536, adc_gain.vbatt_gain, adc_gain.vbatt_offset);

    float theta_e = encoder.getElecPos_rad();
    float omega_m = encoder.getRPM() * RPM_TO_RAD_S;

    foc_state.ts = 1.0f / (float)motorPWM.getFrequency();

    float dutyA, dutyB, dutyC;

    focTest(&foc_state,
            ia, ib, ic,
            vdc,
            theta_e, omega_m,
            &dutyA, &dutyB, &dutyC);

    motorPWM.setDuty(dutyA, dutyB, dutyC);
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
    modulate(ModulationType::SVPWM, alpha, beta, 2.0f, Ts, &dutyA, &dutyB, &dutyC);

    motorPWM.setDuty(dutyA, dutyB, dutyC);

    //usb.printf("Duty Cycle: %.2f %.2f %.2f %.2f %.2f %.2f\n", dutyA, dutyB, dutyC, alpha, beta, angle);

    if (timeElapsed++ >= 20U) timeElapsed = 0;
    //if (counter++ >= 50000) {
    //    counter = 0;
    //    usb.printf("One Second\n");W
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