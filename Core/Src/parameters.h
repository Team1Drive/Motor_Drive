#pragma once

#define M_PI 3.14159265358979323846264338327950288f
#define SQRT3 1.73205080756887729352744634150587236f

#define ADC1_NUM_CHANNELS   3U
#define ADC2_NUM_CHANNELS   2U
#define ADC3_NUM_CHANNELS   2U

#define MOTOR_ROTATION_DIRECTION    -1 // 1 for clockwise, -1 for counterclockwise
#define MOTOR_SIXSTEP_DUTYCYCLE     0.7f // Range 1.0 to 0.5

#define ENCODER_PPR     2048U // Pulses per revolution for the encoder

#define USTIMER_ENCODER_PULSE_ID  0U // Identifier for encoder pulse timing in the microsecond timer
#define USTIMER_ENCODER_INDEX_ID  1U // Identifier for encoder index timing in the microsecond timer

typedef enum {
    MOTOR_STOP,
    MOTOR_STARTUP,
    MOTOR_SIX_STEP,
    MOTOR_FOC_LINEAR,
    MOTOR_FOC_DPWM
} MotorControlMode;