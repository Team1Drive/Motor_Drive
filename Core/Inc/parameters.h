#pragma once

#define M_PI 3.14159265358979323846264338327950288f
#define SQRT3 1.73205080756887729352744634150587236f

#define APB_CLOCK_FREQ_HZ   275000000U // 275 MHz APB clock frequency (for timer calculations)

#define RPM_TO_RAD_S        (2.0f * M_PI / 60.0f)
#define RAD_S_TO_RPM        (60.0f / (2.0f * M_PI))

#define DEG_TO_RAD          (M_PI / 180.0f)
#define RAD_TO_DEG          (180.0f / M_PI)

#define FREQ_TO_OMEGA       (2.0f * M_PI)
#define OMEGA_TO_FREQ       (1.0f / (2.0f * M_PI))

#define ADC1_NUM_CHANNELS   3U
#define ADC2_NUM_CHANNELS   2U
#define ADC3_NUM_CHANNELS   2U

#define ADC_BUF_SIZE        2048U
#define ADC_HALF_BUF_SIZE   (ADC_BUF_SIZE / 2U)

#define ADC1_BUF_LEN  (ADC_BUF_SIZE * ADC1_NUM_CHANNELS)
#define ADC2_BUF_LEN  (ADC_BUF_SIZE * ADC2_NUM_CHANNELS)
#define ADC3_BUF_LEN  (ADC_BUF_SIZE * ADC3_NUM_CHANNELS)

// ---------- ADC Calibration Parameters ----------
#define BOARD_SELECTION     1U
// Board 1
#define ADC_IA_SHUNT_1      0.00371f // Ia channel gain (shunt resistor)
#define ADC_IB_SHUNT_1      0.00212f // Ib channel gain (shunt resistor)
#define ADC_IC_SHUNT_1      0.00174f // Ic channel gain (shunt resistor)
#define ADC_IA_OFFSET_1     0.00376f // Ia channel offset
#define ADC_IB_OFFSET_1     0.00491f // Ib channel offset
#define ADC_IC_OFFSET_1     0.00414f // Ic channel offset
#define ADC_VA_GAIN_1       0.0316f // Va channel gain (voltage divider)
#define ADC_VB_GAIN_1       0.0316f // Vb channel gain (voltage divider)
#define ADC_VA_OFFSET_1     0.0f // Va channel offset
#define ADC_VB_OFFSET_1     0.0f // Vb channel offset
#define ADC_IBATT_SHUNT_1   -0.0032f // Battery current channel gain (shunt resistor)
#define ADC_IBATT_OFFSET_1  0.0035f // Battery current channel offset
#define ADC_VBATT_GAIN_1    0.12817f // Battery voltage channel gain (voltage divider)
#define ADC_VBATT_OFFSET_1  0.00051f // Battery voltage channel offset
// Board 2
#define ADC_IA_SHUNT_2      0.00358f // Ia channel gain (shunt resistor)
#define ADC_IB_SHUNT_2      0.00211f // Ib channel gain (shunt resistor)
#define ADC_IC_SHUNT_2      0.00246f // Ic channel gain (shunt resistor)
#define ADC_IA_OFFSET_2     0.00444f // Ia channel offset
#define ADC_IB_OFFSET_2     0.00524f // Ib channel offset
#define ADC_IC_OFFSET_2     0.00952f // Ic channel offset
#define ADC_VA_GAIN_2       0.0316f // Va channel gain (voltage divider)
#define ADC_VB_GAIN_2       0.0316f // Vb channel gain (voltage divider)
#define ADC_VA_OFFSET_2     0.0f // Va channel offset
#define ADC_VB_OFFSET_2     0.0f // Vb channel offset
#define ADC_IBATT_SHUNT_2   0.00266f // Battery current channel gain (shunt resistor)
#define ADC_IBATT_OFFSET_2  0.00352f // Battery current channel offset
#define ADC_VBATT_GAIN_2    0.12952f // Battery voltage channel gain (voltage divider)
#define ADC_VBATT_OFFSET_2  0.00749f // Battery voltage channel offset

#define TIM6_FREQ_HZ        1000U
#define TIM15_FREQ_HZ       1000000U
#define SPEEDLOOP_FREQ_HZ   1000U

#define PWM_FREQ_DEFAULT_HZ         20000U
#define PWM_DEADTIME_DEFAULT_NS     1000U

#define MOTOR_POLE_PAIRS            4U

#define MOTOR_ROTATION_DIRECTION    1 // 1 for anticlockwise, -1 for clockwise

#define MOTOR_ALIGNMENT_POS_WINDOW  32768 // Counts of the encoder position within which alignment is considered successful (tuned experimentally)
#define MOTOR_ALIGNMENT_THRESHOLD   1 // Encoder position delta window for successful alignment
#define MOTOR_ALIGNMENT_VOLTAGE     5 // Volts to apply during encoder alignment
#define MOTOR_ALIGNMENT_ID_REF      1.0f
#define MOTOR_ALIGNMENT_IQ_REF      0.0f
#define MOTOR_SPEED_LIMIT_RPM       10000.0f

#define MOTOR_MAX_PHASE_CURRENT     3.5f // Maximum phase current for safety (in amps)
#define MOTOR_INSTANT_TRIP_CURRENT  5.0f // Instantaneous trip threshold for overcurrent protection (in amps)
#define MOTOR_INT_CURRENT_THRESHOLD 3.0f
#define MOTOR_MAX_CURRENT           5.0f // Maximum current for safety (in amps), including battery current
#define MOTOR_MIN_VOLTAGE           6.0f // Minimum voltage for operation (in volts) 

#define SIXSTEP_DUTYCYCLE           1.0f // Range 1.0 to 0.5

#define VVVF_RAMP_UP_SPEED          60U // 60 RPM/s
#define VVVF_MAX_RPM                3000U // Max RPM for VVVF mod
#define VVVF_THRESHOLD_RPM          1500U // Minimum RPM to maintain after ramp-up

#define FOC_ALLOWED                 false // Allow FOC mode in the system (set to false to disable FOC-related code and save flash/RAM)
#define FOC_INITIAL_RPM             1500U // Target RPM for FOC mode (used when FOC is enabled and selected)
#define FOC_OVERSAMPLING_SIZE       1U // Number of samples to average for oversampling (must be a power of 2 for efficient averaging)
#define FOC_RAMP_DOWN_SPEED         200U // 200 RPM/s

#define MASTER_MODE                 true // Set master or slave mode in load testing
#define BATTERY_PROTECTION          false // Set to true when powered with supply without current limit
#define BATTERY_LOW_VOLTAGE_THRESHOLD   4.0f // Voltage threshold for low battery protection (in volts)
#define BATTERY_OVERVOLTAGE_THRESHOLD   5.0f // Voltage threshold for overvoltage protection (in volts)

#define ENCODER_PPR                 2048U // Pulses per revolution for the encoder
#define ENCODER_T_THRESHOLD         400U // Threshold in RPM for using T method (Linear Interpolation)
#define ENCODER_M_THRESHOLD         550U // Threshold in RPM for using M method (Linear Interpolation)
#define ENCODER_ONEPULSE_THRESHOLD  1000U // Threshold in RPM for using one pulse counting
#define ENCODER_STALL_THRESHOLD     100U // Threshold for detecting stall

#define USTIMER_ENCODER_PULSE_ID    0U // Identifier for encoder pulse timing in the microsecond timer
#define USTIMER_ENCODER_INDEX_ID    1U // Identifier for encoder index timing in the microsecond timer

#define LOG_MAX_VALUE_WINDOW_SIZE   32U // Number of samples to consider when calculating max current for logging (must be a power of 2 for efficient averaging)

#define TELEMETRY_HEADER_MAGIC      0xAA55 // Magic number for telemetry data packets to identify the start of a valid packet
#define TELEMETRY_PACKET_VERSION    0U
#define ADC_SAMPLE_HEADER_MAGIC     0xAA46 // Magic number for ADC sample packets to identify the start of a valid packet
#define ADC_SAMPLE_PACKET_VERSION   0U

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
    FLAG_ACW                = 1 << 7,   // Set for anti-clockwise rotation, reset for clockwise
    FLAG_TARGET_RAMP        = 1 << 8,   // Indicates ramping to new target in FOC mode
    FLAG_SPEED_RAMP_INIT    = 1 << 9    // Indicates ramping speed to new target in FOC mode
};

enum ErrorFlag : uint32_t {
    ERROR_PWM_CONFIG        = 1 << 0,
    ERROR_ADC_CONFIG        = 1 << 1,
    ERROR_DMA_CONFIG        = 1 << 2,
    ERROR_TIM_CONFIG        = 1 << 3,
    ERROR_ENCODER_CONFIG    = 1 << 4,
    ERROR_FOC_CONFIG        = 1 << 5,
    ERROR_COMM_CONFIG       = 1 << 6,
    ERROR_OVERCURRENT       = 1 << 7,
    ERROR_UNDERVOLTAGE      = 1 << 8
};

enum SimulationFlag : uint16_t {
    SIM_FLAG_RUNNING        = 1 << 0,
    SIM_FLAG_PAUSED         = 1 << 1,
    SIM_FLAG_READY_ALIGN    = 1 << 2,
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
    uint8_t preset;
} ADCGain_t;

typedef struct {
    float speed;
    float torque;
    float time;
    bool is_torque_control;
} Target_t;

enum PrintData : uint32_t {
    PRINT_RPM       = 1 << 0,
    PRINT_RPMSP     = 1 << 1,
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
    PRINT_FOC_VQ    = 1 << 30,
};

enum PrintDataEx : uint32_t {
    PRINT_CP_MODE   = 1 << 0,
    PRINT_M_INDEX   = 1 << 1,
    PRINT_FW        = 1 << 2,
    PRINT_UMAG      = 1 << 3,
    PRINT_IMAG      = 1 << 4,
    PRINT_FFT       = 1 << 5
};

enum class PrintFormat : uint8_t {
    PRINT_UTF8,
    PRINT_BINARY
};

typedef struct {
    float ia;
    float ib;
    float ic;
    float va;
    float vb;
    float vbatt;
    float ibatt;
} Sampling_t;

/*
Timer allocation

TIM1: ADC trigger
TIM2: 10 Hz interrupt
TIM3: 4 Hz interrupt
TIM4: Encoder pulse timing
TIM5: Incremental counter at APB frequency
TIM6: 5000 Hz interrupt for binary telemetry
TIM7: 2 Hz interrupt for communication timeout
TIM8: PWM generation for motor control
TIM12: Upper 16 bits for TIM5 incremental counter (for microsecond timing)
TIM14: Communication timing
TIM15: Remapped TIM4 for encoder index timing
TIM16: Speed loop timer (1000 Hz)



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

