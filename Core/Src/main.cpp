#include "main.h"
#include "parameters.h"
#include "pwm3phase_timer.h"
#include "digitalio.h"
#include "encoder.h"
#include "hallsensor.h"
#include "modulation.h"
#include "ustimer.h"
#include "adc_sampler.h"
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
void speedControl(void);

static void process_command(const char* cmd);

void startUpSequence(void);
void vvvfRampUp(void);
void sixStepCommutation(void);
void test_PWM(void);
void test_PWM_sweep(void);

void usb_printf(const char *format, ...);

float adcToVoltage(uint32_t raw, float vref, uint32_t resolution, float gain, float offset);
float adcToCurrent(uint32_t raw, float vref, uint32_t resolution, float gain, float offset, float shunt);

static bool ring_buffer_write(uint8_t data);
static bool read_line_from_ring(char* line, int max_len);

/* Declare ADC buffers */
alignas(32) uint16_t adc1_buffer[ADC1_BUF_LEN] __attribute__((section(".sram_d1")));
alignas(32) uint16_t adc2_buffer[ADC2_BUF_LEN] __attribute__((section(".sram_d1")));
alignas(32) uint16_t adc3_buffer[ADC3_BUF_LEN] __attribute__((section(".sram_d1")));

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

DigitalOut pwm_ch1_dis(GPIOA, GPIO_PIN_2), pwm_ch2_dis(GPIOB, GPIO_PIN_2), pwm_ch3_dis(GPIOB, GPIO_PIN_13);
DigitalOut led_red(GPIOC, GPIO_PIN_9), led_green(GPIOA, GPIO_PIN_8), led_yellow_1(GPIOA, GPIO_PIN_9), led_yellow_2(GPIOA, GPIO_PIN_10);
DigitalOut relay(GPIOD, GPIO_PIN_8);
Encoder encoder(&htim4, GPIO_PIN_9, ENCODER_PPR, TIM6_FREQ_HZ, ENCODER_STALL_THRESHOLD);
HallSensor hallsensor(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_8, GPIOE, GPIO_PIN_4);

uint16_t adc1_proc_buffer[ADC1_BUF_LEN];
uint16_t adc2_proc_buffer[ADC2_BUF_LEN];
uint16_t adc3_proc_buffer[ADC3_BUF_LEN];

MotorControlMode control_mode = MotorControlMode::MOTOR_STOP;

static ring_buffer_t rx_ring = { .head = 0, .tail = 0 };



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

  /* USER CODE BEGIN 2 */
  led_red.write(1);
  /* Enable USB regulator */
  HAL_PWREx_EnableUSBReg();
  
  /* Start ADC */
  while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC1 Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc1));
  while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC1 linearity Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc1));
  while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC1 linearity Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc1));
  while (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC2 Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc2));
  while (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC2 linearity Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc2));
  while (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC2 linearity Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc2));
  while (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC3 Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc3));
  while (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC3 linearity Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc3));
  while (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED) != HAL_OK) usb_printf("Failed to calibrate ADC3 linearity Error code: 0x%lx\r\n", HAL_ADC_GetError(&hadc3));

  while (adc1.startDMA() != HAL_OK) usb_printf("Failed to start ADC1 DMA Error code: 0x%lx\r\n", HAL_DMA_GetError(&hdma_adc1));
  while (adc2.startDMA() != HAL_OK) usb_printf("Failed to start ADC2 DMA Error code: 0x%lx\r\n", HAL_DMA_GetError(&hdma_adc2));
  while (adc3.startDMA() != HAL_OK) usb_printf("Failed to start ADC3 DMA Error code: 0x%lx\r\n", HAL_DMA_GetError(&hdma_adc3));
  
  /* Start timers */
  HAL_TIM_Base_Start(&htim1);
  //HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  usTimer.init();

  encoder.start();

  /* Start PWM */
  motorPWM.init();
  //HAL_TIM_Base_Start(&htim8);
  //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  //HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  //HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  //HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

  /* Enable TIM8 update interrupt */
  //__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

  /* Enable Caches */
  SCB_EnableICache();
  SCB_EnableDCache();

  /* USER CODE END 2 */
  usb_printf("HAL Initialized\n");

  motorPWM.setFrequency(20000);
  motorPWM.setDeadTime(1000);

  adc1.setProcessingBuffer(adc1_proc_buffer, ADC1_BUF_LEN);
  adc2.setProcessingBuffer(adc2_proc_buffer, ADC2_BUF_LEN);
  adc3.setProcessingBuffer(adc3_proc_buffer, ADC3_BUF_LEN);

  led_red.write(0);
  led_yellow_1.write(0);
  led_yellow_2.write(1);
  //relay.write(1);

  usb_printf("System Initialized\n");

  control_mode = MotorControlMode::MOTOR_STOP;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //usb_printf("Hello from STM32! Counter: %lu\r\n", counter++);
    /* USER CODE END WHILE */
    /* HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    usb_printf("Polling Val: %lu\r\n", val); */
    char cmd_line[CMD_MAX_LEN];
    if (read_line_from_ring(cmd_line, CMD_MAX_LEN)) {
      process_command(cmd_line);
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
    if (control_mode == MotorControlMode::MOTOR_FOC_LINEAR) {

    }
    else if (control_mode == MotorControlMode::MOTOR_STARTUP) {
      startUpSequence();
    }
  }
  else if (htim->Instance == TIM6) {
    // 1 kHz control loop interrupt
    Encoder::irqHandlerSpeed();
    speedControl();
    timer6IRQ();
  }
  else if (htim->Instance == TIM7) {
    // 1 MHz timer interrupt (Interrupt every 65.536 ms)
    MicrosecondTimer::irqHandler(htim);
  }
  else if (htim->Instance == TIM2) {
    // 10 Hz timer interrupt
    timer2IRQ();
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

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len) {
  for (uint32_t i = 0; i < Len; i++) {
    ring_buffer_write(Buf[i]);
  }
}

void timer2IRQ(void) {
  uint16_t adc1_raw[3];
  uint16_t adc2_raw[2];
  uint16_t adc3_raw[2];

  adc1.getLatestData(adc1_raw);
  adc2.getLatestData(adc2_raw);
  adc3.getLatestData(adc3_raw);

  //usb_printf("RAW: %u\t%u\t%u\t%.1f\n", adc1_raw[0], adc2_raw[0], adc3_raw[0], encoder.getRPM());

  float ia = adcToCurrent(adc1_raw[0], 3.3f, 65536, 50.0f, 0.0f, 0.013f);
  float ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f, 0.0f, 0.013f);
  float ic = adcToCurrent(adc3_raw[0], 3.3f, 4096, 50.0f, 0.0f, 0.013f);

  float vab = adcToVoltage(adc2_raw[1], 3.3f, 65536, 11.0f, 0.0f);
  float vbc = adcToVoltage(adc1_raw[1], 3.3f, 65536, 11.0f, 0.0f);

  float vbatt = adcToVoltage(adc1_raw[2], 3.3f, 65536, 0.130435f, 0.0f);
  float ibatt = adcToCurrent(adc3_raw[1], 3.3f, 4096, 50.0f, 0.0f, 0.013f);

  //usb_printf("i a/b/c (A):\t%.2f\t%.2f\t%.2f\n", ia, ib, ic);
  //usb_printf("v ab/bc (V):\t%.2f\t%.2f\n", vab, vbc);
  //usb_printf("v batt (V): %.2f\t, i batt (A): %.2f\n", vbatt, ibatt);

  //usb_printf("RPM: %.2f\t, Direction: %d\n", encoder.getRPM(), encoder.getDirection());
}

void timer3IRQ(void) {
  //led_red.toggle();
  //led_green.toggle();
  led_yellow_1.toggle();
  led_yellow_2.toggle();
  //relay.toggle();
}

void timer6IRQ(void) {
  uint16_t adc1_raw[3];
  uint16_t adc2_raw[2];
  uint16_t adc3_raw[2];

  adc1.getLatestData(adc1_raw);
  adc2.getLatestData(adc2_raw);
  adc3.getLatestData(adc3_raw);

  float ia = adcToCurrent(adc1_raw[0], 3.3f, 65536, 50.0f, 0.0f, 0.013f);
  float ib = adcToCurrent(adc2_raw[0], 3.3f, 65536, 50.0f, 0.0f, 0.013f);
  float ic = adcToCurrent(adc3_raw[0], 3.3f, 4096, 50.0f, 0.0f, 0.013f);

  float vab = adcToVoltage(adc2_raw[1], 3.3f, 65536, 11.0f, 0.0f);
  float vbc = adcToVoltage(adc1_raw[1], 3.3f, 65536, 11.0f, 0.0f);

  float vbatt = adcToVoltage(adc1_raw[2], 3.3f, 65536, 0.130435f, 0.0f);
  float ibatt = adcToCurrent(adc3_raw[1], 3.3f, 4096, 50.0f, 0.0f, 0.013f);

  LogData_t data;
  data.ia    = adc1_raw[0];
  data.ib    = adc2_raw[0];
  data.ic    = adc3_raw[0];
  data.speed = encoder.getRPM();
  data.pos   = encoder.getPos();

  CDC_Transmit_HS((uint8_t*)&data, sizeof(LogData_t));
}

void speedControl(void) {
  uint32_t rpm = encoder.getRPM();
}

void startUpSequence(void) {
  if (control_mode != MotorControlMode::MOTOR_STARTUP) return;
  hallsensor.read();
  //sixStepCommutation();
  vvvfRampUp();
}

/**
 * @brief Implements a VVVF ramp-up sequence for a BLDC motor. Gradually increases the frequency and amplitude of the PWM signals to smoothly accelerate the motor from standstill to a target speed defined by VVVF_THRESHOLD_RPM.
 * @note To be called in the TIM8 update interrupt when running in MOTOR_STARTUP mode.
 */
void vvvfRampUp(void) {
  const uint32_t frequency = 20000U;
  const uint32_t ramp_up = VVVF_RAMP_UP_SPEED; // RPM/s
  const float step_increment = (float)ramp_up / frequency;
  static float rpm = 0.0f;  
  static float angle = 0.0f;
  static bool accelerating = true;
  static uint32_t last_call_time = 0;

  if (HAL_GetTick() - last_call_time > 500U) {
    rpm = 0.0f;
    angle = 0.0f;
  }
  last_call_time = HAL_GetTick();

  if (accelerating) {
    rpm += step_increment;
    if (rpm >= VVVF_THRESHOLD_RPM) {
      rpm = VVVF_THRESHOLD_RPM;
      accelerating = false;
    }
  }

  float electrical_freq = rpm / 60.0f * MOTOR_POLE_PAIRS;

  float delta_angle = 2.0f * M_PI * electrical_freq / frequency;
  angle += delta_angle;
  if (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;

  // 改进的 V/f 曲线：增加最小电压和转折点
  float amplitude;
  const float MIN_VOLTAGE = 0.15f;      // 最小占空比幅度 15%
  const float KNEE_RPM = 300.0f;        // 转折点转速 (RPM)
  if (rpm < KNEE_RPM) {
      // 低速段：线性从 MIN_VOLTAGE 升到 1.0
      amplitude = MIN_VOLTAGE + (1.0f - MIN_VOLTAGE) * (rpm / KNEE_RPM);
  } else {
      // 高速段：保持 1.0（或继续增加，但一般保持满压）
      amplitude = 1.0f;
  }

  // 限制最大幅度
  if (amplitude > 1.0f) amplitude = 1.0f;

  float dutyA = 0.5f + amplitude * 0.5f * sinf(angle);
  float dutyB = 0.5f + amplitude * 0.5f * sinf(angle + 2.0f * M_PI / 3.0f);
  float dutyC = 0.5f + amplitude * 0.5f * sinf(angle + 4.0f * M_PI / 3.0f);

  motorPWM.setDuty(dutyA, dutyB, dutyC);
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

  uint8_t hall_state = hallsensor.getState();

  if (hall_state < 1 || hall_state > 6) return;

  int8_t a_state, b_state, c_state;

  // Determine the desired state of each phase based on the current Hall sensor state and rotation direction
  if (MOTOR_ROTATION_DIRECTION == 1) {
      a_state = commutation_cw[hall_state][0];
      b_state = commutation_cw[hall_state][1];
      c_state = commutation_cw[hall_state][2];
  } else if (MOTOR_ROTATION_DIRECTION == -1) {
      a_state = commutation_acw[hall_state][0];
      b_state = commutation_acw[hall_state][1];
      c_state = commutation_acw[hall_state][2];
  }

  float dutyB = (b_state == 1) ? MOTOR_SIXSTEP_DUTYCYCLE : (b_state == 0) ? (1.0f - MOTOR_SIXSTEP_DUTYCYCLE) : -1.0f;
  float dutyC = (c_state == 1) ? MOTOR_SIXSTEP_DUTYCYCLE : (c_state == 0) ? (1.0f - MOTOR_SIXSTEP_DUTYCYCLE) : -1.0f;
  float dutyA = (a_state == 1) ? MOTOR_SIXSTEP_DUTYCYCLE : (a_state == 0) ? (1.0f - MOTOR_SIXSTEP_DUTYCYCLE) : -1.0f;

  motorPWM.setDuty(dutyA, dutyB, dutyC);
}

float adcToVoltage(uint32_t raw, float vref, uint32_t resolution, float gain, float offset) {
  return ((float)raw / resolution) * vref / gain + offset;
}

float adcToCurrent(uint32_t raw, float vref, uint32_t resolution, float gain, float offset, float shunt) {
  float voltage = adcToVoltage(raw, vref, resolution, 1.0f, offset);
  return voltage / (gain * shunt);
}

// ─────────────────────────────────────────────────────────────────────────────
//  USB Command Processing
// ─────────────────────────────────────────────────────────────────────────────

static void process_command(const char* cmd) {

  // Start
  if (strcmp(cmd, "start") == 0) {
    control_mode = MotorControlMode::MOTOR_STARTUP;
    relay.write(1);
    startUpSequence();
    CDC_Transmit_HS((uint8_t*)"Starting\r\n", 4);

  // Stop
  } else if (strcmp(cmd, "stop") == 0) {
    control_mode = MotorControlMode::MOTOR_STOP;
    relay.write(0);
    CDC_Transmit_HS((uint8_t*)"Stopping\r\n", 4);

  // Six-step commutation
  } else if (strcmp(cmd, "sixstep") == 0) {
    control_mode = MotorControlMode::MOTOR_SIX_STEP;
    relay.write(1);
    sixStepCommutation();
    CDC_Transmit_HS((uint8_t*)"Six-step running\r\n", 31);

  // Set speed for speed loop
  } else if (strncmp(cmd, "speed ", 6) == 0) {
    float speed;
    if (sscanf(cmd + 6, "%f", &speed) == 1) {
      relay.write(1);
      motorPWM.setDuty(speed, speed, speed);
      CDC_Transmit_HS((uint8_t*)"Speed set\r\n", 11);
    } else {
      // Wrong speed format
      const char* err = "Invalid speed\r\n";
      CDC_Transmit_HS((uint8_t*)err, strlen(err));
    }

  // Set duty cycle for each phase
  } else if (strncmp(cmd, "duty ", 5) == 0) {
    // Copy arguments to a temporary buffer for tokenization, ensuring null-termination
    char cmd_copy[CMD_MAX_LEN];
    strncpy(cmd_copy, cmd + 5, CMD_MAX_LEN - 1);
    cmd_copy[CMD_MAX_LEN - 1] = '\0';

    // Parse three float values from the command
    float values[3] = {0};
    int count = 0;
    char* token = strtok(cmd_copy, ",");   // Comma as delimiter for three duty values
    while (token != NULL && count < 3) {
        // Remove leading spaces from the token
        while (*token == ' ') token++;
        char* endptr;
        float val = strtof(token, &endptr);
        // Check if the entire token was a valid float (endptr should point to the end of the token or a space)
        if (endptr != token && (*endptr == '\0' || *endptr == ' ')) {
            values[count++] = val;
        } else {
            break; // Invalid float format, exit parsing
        }
        token = strtok(NULL, ",");
    }

    // Validate 3 values and in range [-1.0, 1.0]
    if (count == 3) {
        if (values[0] >= -1.0f && values[0] <= 1.0f &&
            values[1] >= -1.0f && values[1] <= 1.0f &&
            values[2] >= -1.0f && values[2] <= 1.0f) {
              motorPWM.setDuty(values[0], values[1], values[2]);
              if (values[0] >= 0.0f &&
                  values[1] >= 0.0f &&
                  values[2] >= 0.0f) relay.write(1);
              CDC_Transmit_HS((uint8_t*)"Duty set\r\n", 4);
        } else {
          CDC_Transmit_HS((uint8_t*)"Duty values out of range [-1,1]\r\n", 34);
        }
    } else {
      CDC_Transmit_HS((uint8_t*)"Invalid duty format. Usage: duty 0.3,0.3,0.3\r\n", 48);
    }

  // Set specific six-step vector
  } else if (strncmp(cmd, "vec ", 4) == 0) {
    relay.write(1);
    int vec_num = atoi(cmd + 4);  // Convert the vector number from string to integer
    if (vec_num >= 0 && vec_num <= 5) {
      // Commutation table
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
      
      // Convert to duty cycles or -1 (high impedance)
      float dutyA = (a_state == 1) ? MOTOR_SIXSTEP_DUTYCYCLE : 
                    (a_state == 0) ? (1.0f - MOTOR_SIXSTEP_DUTYCYCLE) : -1.0f;
      float dutyB = (b_state == 1) ? MOTOR_SIXSTEP_DUTYCYCLE : 
                    (b_state == 0) ? (1.0f - MOTOR_SIXSTEP_DUTYCYCLE) : -1.0f;
      float dutyC = (c_state == 1) ? MOTOR_SIXSTEP_DUTYCYCLE : 
                    (c_state == 0) ? (1.0f - MOTOR_SIXSTEP_DUTYCYCLE) : -1.0f;
      
      motorPWM.setDuty(dutyA, dutyB, dutyC);
      
      // Read current Hall sensor state for debugging
      uint8_t hall_state = hallsensor.getState();  // Ensure this function exists
      char resp[64];
      snprintf(resp, sizeof(resp), "Vector %d applied, Hall=%d\r\n", vec_num, hall_state);
      CDC_Transmit_HS((uint8_t*)resp, strlen(resp));
    } else {
        CDC_Transmit_HS((uint8_t*)"Invalid vector. Use 0-5.\r\n", 26);
    }
    
  // Handle invalid case
  } else {
      // Unknown command
      const char* err = "Unknown command\r\n";
      CDC_Transmit_HS((uint8_t*)err, strlen(err));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  USB Communication and Command Processing
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief A simple printf-like function that formats a string and sends it over USB using the CDC interface. This function uses a fixed-size buffer to hold the formatted string and supports variable arguments like printf.
 * @param format The format string, similar to printf.
 */
void usb_printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);

    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        CDC_Transmit_HS((uint8_t*)buffer, len);
    }
}

static bool ring_buffer_write(uint8_t data) {
    uint16_t next_head = (rx_ring.head + 1) % RX_RING_SIZE;
    if (next_head != rx_ring.tail) {
        rx_ring.buffer[rx_ring.head] = data;
        rx_ring.head = next_head;
        return true;
    }
    return false;
}

static bool read_line_from_ring(char* line, int max_len) {
    static char line_buffer[CMD_MAX_LEN];
    static int idx = 0;

    while (rx_ring.tail != rx_ring.head) {
        uint8_t c = rx_ring.buffer[rx_ring.tail];
        rx_ring.tail = (rx_ring.tail + 1) % RX_RING_SIZE;

        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                line_buffer[idx] = '\0';
                strncpy(line, line_buffer, max_len);
                idx = 0;
                return true;
            }
        } else if (idx < max_len - 1) {
            line_buffer[idx++] = c;
        } else {
            idx = 0;
        }
    }
    return false;
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