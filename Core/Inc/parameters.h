#pragma once

#define M_PI 3.14159265358979323846264338327950288f
#define SQRT3 1.73205080756887729352744634150587236f

#define RX_RING_SIZE 256
#define CMD_MAX_LEN 64

#define ADC1_NUM_CHANNELS   3U
#define ADC2_NUM_CHANNELS   2U
#define ADC3_NUM_CHANNELS   2U

#define ADC_IA_SHUNT        0.013f // Ia channel gain (shunt resistor)
#define ADC_IB_SHUNT        0.013f // Ib channel gain (shunt resistor)
#define ADC_IC_SHUNT        0.013f // Ic channel gain (shunt resistor)
#define ADC_VA_GAIN         11.0f // Va channel gain (voltage divider)
#define ADC_VB_GAIN         11.0f // Vb channel gain (voltage divider)
#define ADC_IBATT_SHUNT     0.013f // Battery current channel gain (shunt resistor)
#define ADC_VBATT_GAIN      0.130435f // Battery voltage channel gain (voltage divider)

#define TIM6_FREQ_HZ        1000U

#define MOTOR_POLE_PAIRS            4U

#define MOTOR_ROTATION_DIRECTION    1 // 1 for clockwise, -1 for counterclockwise

#define SIXSTEP_DUTYCYCLE           1.0f // Range 1.0 to 0.5

#define VVVF_RAMP_UP_SPEED          60U // 60 RPM/s
#define VVVF_THRESHOLD_RPM          2000U // Minimum RPM to maintain after ramp-up

#define ENCODER_PPR                 2048U // Pulses per revolution for the encoder
#define ENCODER_STALL_THRESHOLD     10U // Threshold for detecting stall

#define USTIMER_ENCODER_PULSE_ID  0U // Identifier for encoder pulse timing in the microsecond timer
#define USTIMER_ENCODER_INDEX_ID  1U // Identifier for encoder index timing in the microsecond timer



enum class MotorControlMode : uint8_t {
    MOTOR_STOP,
    MOTOR_MANUAL,
    MOTOR_STARTUP,
    MOTOR_VVVF,
    MOTOR_SIX_STEP,
    MOTOR_FOC_LINEAR,
    MOTOR_FOC_DPWM
};

typedef struct {
    uint8_t buffer[RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

typedef struct {
    bool    is_vvvf_running;
    bool    is_sixstep_running;
    bool    is_foc_running;
    uint8_t led_increment_counter;
} SystemStatus_t;

typedef struct {
    float ia_shunt;
    float ib_shunt;
    float ic_shunt;
    float va_gain;
    float vb_gain;
    float ibatt_shunt;
    float vbatt_gain;
} ADCGain_t;

typedef struct {
    float speed;
    float torque;
} Target_t;

#pragma pack(1)
typedef struct {
    uint16_t ia;
    uint16_t ib;
    uint16_t ic;
    float speed;
    uint16_t pos;
} LogData_t;
#pragma pack()


/*
Timer allocation

TIM1: ADC trigger
TIM2: 10 Hz interrupt
TIM3: 4 Hz interrupt
TIM4: Encoder pulse timing
TIM6: 1000 Hz interrupt
TIM7: Microsecond timer
TIM8: PWM generation for motor control


ADC channel allocation

I_A     ADC1  PA7 INP7
I_B     ADC2  PB1 INP5
I_C     ADC3  PC1 INP11
V_A+    ADC2  PC4 INP4
V_A-    ADC2  PC5 INN4
V_B+    ADC1  PA0 INP16
V_B-    ADC1  PA1 INN16
I_BATT  ADC3  PC0 INP10
V_BATT  ADC1  PB0 INP9
*/

