#pragma once

#define M_PI 3.14159265358979323846264338327950288f
#define SQRT3 1.73205080756887729352744634150587236f

#define RPM_TO_RAD_S        (2.0f * M_PI / 60.0f)
#define RAD_S_TO_RPM        (60.0f / (2.0f * M_PI))

#define DEG_TO_RAD          (M_PI / 180.0f)
#define RAD_TO_DEG          (180.0f / M_PI)

#define FREQ_TO_OMEGA       (2.0f * M_PI)
#define OMEGA_TO_FREQ       (1.0f / (2.0f * M_PI))

#define ADC1_NUM_CHANNELS   3U
#define ADC2_NUM_CHANNELS   2U
#define ADC3_NUM_CHANNELS   2U

#define ADC_IA_SHUNT        0.00368f // Ia channel gain (shunt resistor)
#define ADC_IB_SHUNT        0.00214f // Ib channel gain (shunt resistor)
#define ADC_IC_SHUNT        0.00181f // Ic channel gain (shunt resistor)
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

#define PWM_FREQ_DEFAULT_HZ 20000U

#define MOTOR_POLE_PAIRS            4U

#define MOTOR_ROTATION_DIRECTION    1 // 1 for anticlockwise, -1 for clockwise

#define MOTOR_ALIGNMENT_POS_WINDOW  2048 // Counts of the encoder position within which alignment is considered successful (tuned experimentally)
#define MOTOR_ALIGNMENT_THRESHOLD   1 // Encoder position delta window for successful alignment
#define MOTOR_ALIGNMENT_VOLTAGE     5 // Volts to apply during encoder alignment

#define SIXSTEP_DUTYCYCLE           1.0f // Range 1.0 to 0.5

#define VVVF_RAMP_UP_SPEED          60U // 60 RPM/s
#define VVVF_MAX_RPM                3000U // Max RPM for VVVF mod
#define VVVF_THRESHOLD_RPM          1500U // Minimum RPM to maintain after ramp-up

#define FOC_ALLOWED                 false // Allow FOC mode in the system (set to false to disable FOC-related code and save flash/RAM)
#define FOC_INITIAL_RPM             1500U // Target RPM for FOC mode (used when FOC is enabled and selected)
#define FOC_OVERSAMPLING_SIZE       16U // Number of samples to average for oversampling (must be a power of 2 for efficient averaging)

#define MASTER_MODE                 true // Set master or slave mode in load testing
#define BATTERY_PROTECTION          false // Set to true when powered with supply without current limit
#define BATTERY_LOW_VOLTAGE_THRESHOLD   4.0f // Voltage threshold for low battery protection (in volts)
#define BATTERY_OVERVOLTAGE_THRESHOLD   5.0f // Voltage threshold for overvoltage protection (in volts)

#define ENCODER_PPR                 2048U // Pulses per revolution for the encoder
#define ENCODER_MT_THRESHOLD        500U // Threshold in RPM for switching between M and T methods
#define ENCODER_ONEPULSE_THRESHOLD  1000U // Threshold in RPM for using one pulse counting
#define ENCODER_STALL_THRESHOLD     10U // Threshold for detecting stall

#define USTIMER_ENCODER_PULSE_ID    0U // Identifier for encoder pulse timing in the microsecond timer
#define USTIMER_ENCODER_INDEX_ID    1U // Identifier for encoder index timing in the microsecond timer

#define LOG_MAX_VALUE_WINDOW_SIZE   32U // Number of samples to consider when calculating max current for logging (must be a power of 2 for efficient averaging)



enum class MotorControlMode : uint8_t {
    MOTOR_PROTECTION,
    MOTOR_STOP,
    MOTOR_MANUAL,
    MOTOR_ALIGN,
    MOTOR_STARTUP,
    MOTOR_VVVF,
    MOTOR_SIX_STEP,
    MOTOR_FOC_MANUAL,
    MOTOR_FOC_LINEAR,
    MOTOR_FOC_DPWM
};

enum SystemFlag : uint32_t {
    FLAG_VVVF_RUNNING       = 1 << 0,   // Indicates VVVF mode is active, for resetting ramp-up on mode change
    FLAG_VVVF_RAMP_UP       = 1 << 1,   // Indicates ramping up in VVVF, default 0 for fail-safe(ramping down), only set when ramping up
    FLAG_AUDIBLE            = 1 << 2,
    FLAG_SIXSTEP_RUNNING    = 1 << 3,   // Indicates six-step mode is active, for reading encoder at stand still
    FLAG_FOC_RUNNING        = 1 << 4,   // For resetting FOC state at mode change
    FLAG_ROTOR_ALIGNING     = 1 << 5,   // For sending duty cycle at beginning of alignment
    FLAG_ELEC_ZERO_ALIGNED  = 1 << 6,   // Indicates electrical zero acquired after alignment
    FLAG_ACW                = 1 << 7    // Set for anti-clockwise rotation, reset for clockwise
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
    PRINT_HALL      = 1 << 0,
    PRINT_RPM       = 1 << 1,
    PRINT_POS       = 1 << 2,
    PRINT_ELPOS     = 1 << 3,
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
    PRINT_IA_MAX    = 1 << 21,
    PRINT_IB_MAX    = 1 << 22,
    PRINT_IC_MAX    = 1 << 23,
    PRINT_IBATT_MAX = 1 << 24,
    PRINT_FOC_ID    = 1 << 25,
    PRINT_FOC_IQ    = 1 << 26,
    PRINT_FOC_IDSP  = 1 << 27,
    PRINT_FOC_IQSP  = 1 << 28,
    PRINT_FOC_VD    = 1 << 29,
    PRINT_FOC_VQ    = 1 << 30
};

enum class PrintFormat : uint8_t {
    PRINT_UTF8,
    PRINT_BINARY
};

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

