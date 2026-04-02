#pragma once

#define M_PI 3.14159265358979323846264338327950288f
#define SQRT3 1.73205080756887729352744634150587236f

#define ADC1_NUM_CHANNELS   3U
#define ADC2_NUM_CHANNELS   2U
#define ADC3_NUM_CHANNELS   2U

#define ADC_IA_SHUNT        0.0062f // Ia channel gain (shunt resistor)
#define ADC_IB_SHUNT        0.004f // Ib channel gain (shunt resistor)
#define ADC_IC_SHUNT        0.0033f // Ic channel gain (shunt resistor)
#define ADC_IA_OFFSET       0.006f // Ia channel offset
#define ADC_IB_OFFSET       0.006f // Ib channel offset
#define ADC_IC_OFFSET       0.005f // Ic channel offset
#define ADC_VA_GAIN         0.0316f // Va channel gain (voltage divider)
#define ADC_VB_GAIN         0.0316f // Vb channel gain (voltage divider)
#define ADC_VA_OFFSET       0.0f // Va channel offset
#define ADC_VB_OFFSET       0.0f // Vb channel offset
#define ADC_IBATT_SHUNT     0.0028f // Battery current channel gain (shunt resistor)
#define ADC_IBATT_OFFSET    0.004f // Battery current channel offset
#define ADC_VBATT_GAIN      0.130435f // Battery voltage channel gain (voltage divider)
#define ADC_VBATT_OFFSET    0.0f // Battery voltage channel offset

#define TIM6_FREQ_HZ        1000U

#define MOTOR_POLE_PAIRS            4U

#define MOTOR_ROTATION_DIRECTION    1 // 1 for anticlockwise, -1 for clockwise

#define SIXSTEP_DUTYCYCLE           1.0f // Range 1.0 to 0.5

#define VVVF_RAMP_UP_SPEED          60U // 60 RPM/s
#define VVVF_MAX_RPM                3000U // Max RPM for VVVF mod
#define VVVF_THRESHOLD_RPM          1500U // Minimum RPM to maintain after ramp-up

#define FOC_ALLOWED                 true // Allow FOC mode in the system (set to false to disable FOC-related code and save flash/RAM)
#define FOC_INITIAL_RPM             1500U // Target RPM for FOC mode (used when FOC is enabled and selected)
#define FOC_OVERSAMPLING_SIZE       16U // Number of samples to average for oversampling (must be a power of 2 for efficient averaging)

#define ENCODER_PPR                 2048U // Pulses per revolution for the encoder
#define ENCODER_MT_THRESHOLD        500U // Threshold in RPM for switching between M and T methods
#define ENCODER_ONEPULSE_THRESHOLD  1000U // Threshold in RPM for using one pulse counting
#define ENCODER_STALL_THRESHOLD     10U // Threshold for detecting stall

#define USTIMER_ENCODER_PULSE_ID  0U // Identifier for encoder pulse timing in the microsecond timer
#define USTIMER_ENCODER_INDEX_ID  1U // Identifier for encoder index timing in the microsecond timer



enum class MotorControlMode : uint8_t {
    MOTOR_PROTECTION,
    MOTOR_STOP,
    MOTOR_MANUAL,
    MOTOR_STARTUP,
    MOTOR_VVVF,
    MOTOR_SIX_STEP,
    MOTOR_FOC_LINEAR,
    MOTOR_FOC_DPWM
};

enum SystemFlag : uint32_t {
    FLAG_VVVF_RUNNING       = 1 << 0,
    FLAG_VVVF_RAMP_UP       = 1 << 1,
    FLAG_AUDIBLE            = 1 << 2,
    FLAG_SIXSTEP_RUNNING    = 1 << 3,
    FLAG_FOC_RUNNING        = 1 << 4,
    FLAG_FOC_ALLOWED        = 1 << 5
};

enum ErrorFlag : uint32_t {
    ERROR_PWM_CONFIG        = 1 << 0,
    ERROR_ADC_CONFIG        = 1 << 1,
    ERROR_DMA_CONFIG        = 1 << 2,
    ERROR_TIM_CONFIG        = 1 << 3,
    ERROR_ENCODER_CONFIG    = 1 << 4,
    ERROR_FOC_CONFIG        = 1 << 5,
    ERROR_OVERCURRENT       = 1 << 6
};

typedef struct {
    float ia_shunt;
    float ib_shunt;
    float ic_shunt;
    float ia_offset;
    float ib_offset;
    float ic_offset;
    float va_gain;
    float vb_gain;
    float va_offset;
    float vb_offset;
    float ibatt_shunt;
    float ibatt_offset;
    float vbatt_gain;
    float vbatt_offset;
} ADCGain_t;

typedef struct {
    float speed;
    float torque;
} Target_t;

enum PrintData : uint32_t {
    PRINT_HALLBIN   = 1 << 0,
    PRINT_HALLDEC   = 1 << 1,
    PRINT_RPM       = 1 << 2,
    PRINT_POS       = 1 << 3,
    PRINT_DUTY_A    = 1 << 4,
    PRINT_DUTY_B    = 1 << 5,
    PRINT_DUTY_C    = 1 << 6,
    PRINT_IA        = 1 << 7,
    PRINT_IB        = 1 << 8,
    PRINT_IC        = 1 << 9,
    PRINT_VA        = 1 << 10,
    PRINT_VB        = 1 << 11,
    PRINT_VBATT     = 1 << 12,
    PRINT_IBATT     = 1 << 13,
    PRINT_IA_RAW    = 1 << 14,
    PRINT_IB_RAW    = 1 << 15,
    PRINT_IC_RAW    = 1 << 16,
    PRINT_VA_RAW    = 1 << 17,
    PRINT_VB_RAW    = 1 << 18,
    PRINT_VBATT_RAW = 1 << 19,
    PRINT_IBATT_RAW = 1 << 20,
    PRINT_COUNT     = 1 << 21
};

enum class PrintFormat : uint8_t {
    PRINT_UTF8,
    PRINT_BINARY
};

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

