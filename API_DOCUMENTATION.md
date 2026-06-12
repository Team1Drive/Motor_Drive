# Motor Drive Firmware API Documentation

This document describes the project-owned firmware API for the Motor Drive STM32H725 codebase. It focuses on the C/C++ modules under `Core/Inc` and `Core/Src`, plus the USB CDC command and telemetry protocol exposed to a host PC.

Generated STM32CubeMX, HAL, CMSIS, USB middleware, and build-output files are intentionally not documented here except where project code calls into them.

## API Overview

The firmware exposes three main integration surfaces:

1. C/C++ firmware modules used inside the embedded application.
2. USB CDC command-line commands for tuning, control, and diagnostics.
3. USB CDC telemetry streams for text, binary, and bulk ADC sample output.

The main application wires the modules together in `Core/Src/main.cpp`. Hardware interrupts call static module handlers, while the main loop services USB command input, USB output queues, and optional bulk ADC capture.

## Common Types And Constants

Header: `Core/Inc/parameters.h`

### Timing And Conversion Constants

| Constant | Meaning |
| --- | --- |
| `APB_CLOCK_FREQ_HZ` | APB timer clock used for high-resolution timer calculations. |
| `RPM_TO_RAD_S` | RPM to mechanical rad/s conversion factor. |
| `RAD_S_TO_RPM` | Mechanical rad/s to RPM conversion factor. |
| `DEG_TO_RAD`, `RAD_TO_DEG` | Angle conversion factors. |
| `FREQ_TO_OMEGA`, `OMEGA_TO_FREQ` | Frequency and angular-frequency conversion factors. |

### ADC Constants

| Constant | Meaning |
| --- | --- |
| `ADC1_NUM_CHANNELS`, `ADC2_NUM_CHANNELS`, `ADC3_NUM_CHANNELS` | Number of configured channels per ADC. |
| `ADC_BUF_SIZE` | Sample groups per ADC DMA buffer. |
| `ADC_HALF_BUF_SIZE` | Half-buffer sample-group count. |
| `ADC1_BUF_LEN`, `ADC2_BUF_LEN`, `ADC3_BUF_LEN` | DMA buffer lengths in `uint16_t` elements. |

### Motor And Safety Constants

| Constant | Meaning |
| --- | --- |
| `PWM_FREQ_DEFAULT_HZ` | Default PWM frequency. |
| `PWM_DEADTIME_DEFAULT_NS` | Default complementary PWM dead time. |
| `MOTOR_POLE_PAIRS` | Motor pole-pair count. |
| `MOTOR_SPEED_LIMIT_RPM` | Speed command limit. |
| `MOTOR_MAX_PHASE_CURRENT` | Phase current safety threshold. |
| `MOTOR_INSTANT_TRIP_CURRENT` | Instantaneous over-current trip threshold. |
| `MOTOR_MIN_VOLTAGE` | Minimum operating bus voltage. |

### Control Modes

`MotorControlMode` describes the active controller state:

| Value | Meaning |
| --- | --- |
| `MOTOR_PROTECTION` | Fault/protection state. |
| `MOTOR_STOP` | Outputs disabled and no control running. |
| `MOTOR_MANUAL` | Direct manual PWM duty command mode. |
| `MOTOR_ALIGN` | Rotor/encoder alignment mode. |
| `MOTOR_STARTUP` | Startup transition mode. |
| `MOTOR_VVVF` | V/f open-loop control mode. |
| `MOTOR_SIX_STEP` | Hall-sensor six-step commutation mode. |
| `MOTOR_FOC_MANUAL` | Manual FOC voltage/current setpoint mode. |
| `MOTOR_FOC_LINEAR` | FOC linear modulation mode. |
| `MOTOR_FOC_DPWM` | FOC DPWM/overmodulation mode. |

### Flags

`SystemFlag`, `ErrorFlag`, and `SimulationFlag` are bit masks stored in global state.

Important error flags:

| Flag | Meaning |
| --- | --- |
| `ERROR_PWM_CONFIG` | PWM setup failure. |
| `ERROR_ADC_CONFIG` | ADC setup/calibration failure. |
| `ERROR_DMA_CONFIG` | DMA setup/start failure. |
| `ERROR_TIM_CONFIG` | Timer setup/start failure. |
| `ERROR_ENCODER_CONFIG` | Encoder setup/start failure. |
| `ERROR_FOC_CONFIG` | FOC setup failure. |
| `ERROR_COMM_CONFIG` | Communication setup failure. |
| `ERROR_OVERCURRENT` | Over-current trip. |
| `ERROR_UNDERVOLTAGE` | Under-voltage trip. |

### Data Structures

```cpp
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
```

```cpp
typedef struct {
    float speed;
    float torque;
    float time;
    bool is_torque_control;
} Target_t;
```

```cpp
typedef struct {
    float ia;
    float ib;
    float ic;
    float va;
    float vb;
    float vbatt;
    float ibatt;
} Sampling_t;
```

## ADC Sampler

Header: `Core/Inc/adc_sampler.h`

`ADCSampler` wraps one ADC plus its DMA buffer. It supports interrupt-driven half/full-buffer updates, latest-sample reads, averaged reads, and bulk packet assembly for host-side ADC capture.

### Bulk ADC Packet Header

```cpp
typedef struct __attribute__((packed)) {
    uint8_t  magic1;
    uint8_t  magic2;
    uint8_t  version;
    uint8_t  adc_id;
    uint16_t sample_count;
    uint8_t  resolution_bit;
    uint32_t sequence;
    uint32_t timestamp_us;
    float    shunt;
    float    offset;
} adc_bulk_sampling_t;
```

The packed header is 23 bytes. `magic1` and `magic2` are `0xAA` and `0x46`. `adc_id` is `1`, `2`, or `3`. `resolution_bit` is 16 for ADC1/ADC2 and 12 for ADC3.

Payload contains `sample_count` little-endian `uint16_t` samples for the first channel in the ready DMA half-buffer.

### Constructor

```cpp
ADCSampler(ADC_HandleTypeDef* hadc,
           DMA_HandleTypeDef* hdma,
           volatile uint16_t* buffer,
           uint32_t length);
```

Creates a sampler over an ADC, DMA handle, and circular DMA buffer.

### Interrupt Entry Points

```cpp
static void irqConvCplt(ADC_HandleTypeDef* hadc);
static void irqConvHalfCplt(ADC_HandleTypeDef* hadc);
```

Call these from `HAL_ADC_ConvCpltCallback()` and `HAL_ADC_ConvHalfCpltCallback()` to route ADC interrupts to the correct sampler instance.

### Control

```cpp
HAL_StatusTypeDef startADC(void);
HAL_StatusTypeDef startDMA(void);
void initBulkHeader(float shunt, float offset);
void setProcessingBuffer(uint16_t* proc_buf, uint32_t proc_len);
```

`startDMA()` is the normal project path. `setProcessingBuffer()` enables cache-safe half-buffer copies used by `assembleBulkPacket()`.

### Data Reads

```cpp
uint32_t getLatestData(uint16_t* data_ptr);
uint32_t getLatestData(uint16_t* data_ptr, uint32_t set_length);
uint32_t getLatestDataMean(uint16_t* data_ptr, uint32_t set_length);
uint16_t getLatestChannel(uint8_t channel);
uint32_t getLatestChannel(uint8_t channel, uint16_t* data_ptr, uint32_t set_length);
uint16_t getLatestChannelMean(uint8_t channel, uint32_t set_length);
bool assembleBulkPacket(const uint8_t** packet_ptr, uint16_t* length_ptr);
```

Notes:

- `set_length` must not exceed half the DMA buffer divided by channel count.
- Mean functions require power-of-two sample counts.
- Reads return zeros before the first DMA half-buffer completion.
- Callers must provide output buffers large enough for the requested data.

### Sequence Synchronization

```cpp
static inline void syncSequence(ADCSampler* adc1,
                                ADCSampler* adc2,
                                ADCSampler* adc3);
```

Synchronizes ADC bulk packet sequence counters before multi-ADC capture.

## PWM Output

Header: `Core/Inc/pwm3phase_timer.h`

`ThreePhasePWMOut` wraps a timer configured for three-phase PWM with complementary outputs.

```cpp
ThreePhasePWMOut(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef init(void);
HAL_StatusTypeDef start(void);
HAL_StatusTypeDef stop(void);
void setDuty(float duty_A, float duty_B, float duty_C);
float getDuty(uint8_t phase) const;
HAL_StatusTypeDef setDeadTime(uint32_t deadtime_ns);
HAL_StatusTypeDef setFrequency(uint32_t freq_Hz);
uint32_t getFrequency(void) const;
```

`setDuty()` expects values in `[0.0, 1.0]`. Values below `-0.5f` disable the corresponding phase output.

## Encoder

Header: `Core/Inc/encoder.h`

`Encoder` wraps the quadrature encoder timer, index pulse, electrical zero alignment state, and speed estimation.

```cpp
Encoder(TIM_HandleTypeDef* htim,
        TIM_HandleTypeDef* htim_t,
        uint16_t index_pin,
        uint32_t pulses_per_rev,
        uint32_t speedloop_freq,
        uint8_t stall_threshold);
```

### Lifecycle And Interrupts

```cpp
void init(void);
HAL_StatusTypeDef start(void);
void elecZeroAlign(void);
void reset(void);

static void irqHandlerIndex(uint16_t pin);
static void irqHandlerSpeed(void);
static void irqHandlerEncoderOverflow(void);
static void irqHandlerTimerOverflow(void);
```

Call the static handlers from GPIO index, speed-loop timer, encoder overflow, and index timing overflow interrupts.

### Reads

```cpp
uint16_t getPos(void) const;
uint16_t getPosBypass(void) const;
uint16_t getElecPos(void) const;
int8_t getDirection(void) const;
float getRPM(void) const;
float getPos_deg(void) const;
float getPos_rad(void) const;
float getElecPos_rad(void) const;
```

Public status fields:

```cpp
volatile bool is_synchronized_;
volatile bool is_zeroed_;
```

## Hall Sensor

Header: `Core/Inc/hallsensor.h`

`HallSensor` reads the three digital Hall channels and routes edge interrupts.

```cpp
HallSensor(GPIO_TypeDef* portA, uint16_t pinA,
           GPIO_TypeDef* portB, uint16_t pinB,
           GPIO_TypeDef* portC, uint16_t pinC);

void init(void);
uint8_t read(void);
static void irqHandlerRising(uint16_t pin);
static void irqHandlerFalling(uint16_t pin);
uint8_t getState(void) const;
void printState(char* buffer);
```

`printState()` writes a `CBA` string and requires at least four bytes including the null terminator.

## Digital I/O Helpers

Header: `Core/Inc/digitalio.h`

```cpp
class DigitalOut {
public:
    DigitalOut(GPIO_TypeDef* port, uint16_t pin);
    void write(bool value);
    void write(GPIO_PinState state);
    void toggle(void);
};

class DigitalIn {
public:
    DigitalIn(GPIO_TypeDef* port, uint16_t pin);
    bool read(void) const;
};
```

These are thin wrappers around `HAL_GPIO_WritePin()`, `HAL_GPIO_TogglePin()`, and `HAL_GPIO_ReadPin()`.

## Timers

Headers: `Core/Inc/timer.h`, `Core/Inc/ustimer.h`

### General Timer

```cpp
Timer(TIM_HandleTypeDef* htim);
static void irqHandler(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef init(void);
HAL_StatusTypeDef start(void);
HAL_StatusTypeDef startIT(void);
HAL_StatusTypeDef setFrequency(uint32_t freq_Hz);
uint32_t getFrequency(void);
```

`Timer` manages timer start and frequency update for the configured STM32 timer.

### Microsecond Timer

```cpp
MicrosecondTimer(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef init(void);
static void irqHandler(TIM_HandleTypeDef* htim);
void start(uint8_t identifier);
uint64_t getTick(void) const;
uint64_t getElapsedTime_us(uint8_t identifier) const;
uint64_t getElapsedTime_ms(uint8_t identifier) const;
float getElapsedTimef_ms(uint8_t identifier) const;
uint64_t getElapsedTime_s(uint8_t identifier) const;
float getElapsedTimef_s(uint8_t identifier) const;
uint64_t reset(uint8_t identifier);
```

Only one `MicrosecondTimer` instance should be created because it uses a static interrupt instance pointer.

### High-Resolution Timer

```cpp
static HAL_StatusTypeDef HighResTimer::start(void);
static uint64_t HighResTimer::getTicks(void);
static uint64_t HighResTimer::getTime_us(void);
static float HighResTimer::getTimef_us(void);
static uint64_t HighResTimer::getTicksDelta(uint64_t last_ticks);
static float HighResTimer::getTimeDelta_us(uint64_t last_ticks);
```

`HighResTimer` combines TIM12 and TIM5 into a wider high-resolution tick counter.

## Modulation

Header: `Core/Inc/modulation.h`

### Modulation Type

```cpp
enum class ModulationType : uint8_t {
    SVPWM,
    SVPWM_COMP,
    SVPWM_SUPERPOS,
    SYM_PWM,
    DPWM0,
    DPWM1,
    DPWM2,
    DPWM3,
    OPTIMAL_FINAL
};
```

### Coordinate Transforms

```cpp
void clarke(float a, float b, float c, float* alpha, float* beta);
void inv_clarke(float alpha, float beta, float* a, float* b, float* c);
void park(float alpha, float beta, float theta, float* d, float* q);
void inv_park(float d, float q, float theta, float* alpha, float* beta);
```

All angles are radians.

### Unified Modulation

```cpp
void modulate(ModulationType type,
              float v_alpha,
              float v_beta,
              float v_dc,
              float Ts,
              float* dutyA,
              float* dutyB,
              float* dutyC,
              float omega_e = 0.0f,
              float* applied_mag = nullptr);
```

Converts alpha-beta voltage references into phase duty cycles. `Ts` is required for `SVPWM_COMP` and `SVPWM_SUPERPOS`; pass `0.0f` for modes that do not use it.

### Optimal Final Modulation

```cpp
void modulate_optimal_final(float v_alpha,
                            float v_beta,
                            float v_dc,
                            float Ts,
                            float theta_e,
                            float theta_m,
                            float m_act,
                            float m_sixstep,
                            float omega_e,
                            float iq_ref,
                            float iq_max,
                            OptimalFinalState& S,
                            float* dutyA,
                            float* dutyB,
                            float* dutyC,
                            bool* just_exited = nullptr,
                            bool* is_active = nullptr);
```

Uses `OptimalFinalState` for thresholds, six-step transition state, hold counters, and region tracking.

## Field-Oriented Control

Header: `Core/Inc/foc.h`

The FOC loop is designed to run from the TIM8 PWM update interrupt. It consumes calibrated phase currents, DC-link voltage, electrical angle, and mechanical speed, then outputs PWM duty cycles.

### PI Controller

```cpp
typedef struct {
    float kp;
    float ki;
    float integrator;
    float clamp_upper;
    float clamp_lower;
} PI_t;

static inline float PI_update(PI_t* pi, float error, float dt);
static inline float piUpdateConditionalIntegration(PI_t* pi, float error, float dt);
static inline void PI_reset(PI_t* pi);
```

### FOC State

`FOC_State_t` stores PI controllers, setpoints, observables, limits, modulation state, and fault status. Important fields include:

| Field | Meaning |
| --- | --- |
| `target_rpm` | Commanded speed target. |
| `omega_ref` | Ramped mechanical speed reference. |
| `Id_ref`, `Iq_ref` | d-axis and q-axis current references. |
| `Id`, `Iq`, `Ia`, `Ib`, `Ic` | Current observables. |
| `Vdc`, `Vd_cmd`, `Vq_cmd` | DC-link and voltage commands. |
| `theta_e`, `omega_e`, `omega_m`, `rpm` | Position and speed observables. |
| `m_index` | Modulation index. |
| `fw_active` | Field-weakening active flag. |
| `fault` | FOC fault latch. |

### Public Functions

```cpp
void foc_init(FOC_State_t* foc);

void foc_run(FOC_State_t* foc,
             float Ia,
             float Ib,
             float Ic,
             float Vdc,
             float theta_e,
             float omega_m,
             float* dutyA,
             float* dutyB,
             float* dutyC);

void foc_reset(FOC_State_t* foc);
void focResetPI(FOC_State_t* foc);
static inline void focResetOuterPI(FOC_State_t* foc);

void focAlignZero(FOC_State_t* foc,
                  float Vmag,
                  float Vdc,
                  float* dutyA,
                  float* dutyB,
                  float* dutyC);

void foc(ModulationType modulation_type,
         FOC_State_t* foc,
         float va,
         float vb,
         float vc,
         float vdc,
         float theta_e,
         float omega_m,
         float* dutyA,
         float* dutyB,
         float* dutyC);

void focInjection(FOC_State_t* foc, float freq);
```

Call `foc_init()` once after hardware initialization. Call `foc_reset()` when stopping or clearing a fault. `foc()` is the project wrapper used by `focTick()` and selects the active modulation strategy.

## Math Helpers

Header: `Core/Inc/math_helpers.h`

```cpp
class RollingMax {
public:
    RollingMax();
    void newValue(float value);
    float getMax() const;
};

float adcToVoltage(uint32_t raw,
                   float vref,
                   uint32_t resolution,
                   float gain,
                   float offset);

float adcToCurrent(uint32_t raw,
                   float vref,
                   uint32_t resolution,
                   float gain,
                   float offset,
                   float shunt);

float clampf(float value, float lower, float upper);
uint16_t fastAverage(uint16_t* data_ptr, uint16_t size);
bool isPowerOfTwo(uint16_t x);
```

`fastAverage()` requires a power-of-two `size`; otherwise it returns `0`.

## Coupled Communication

Header: `Core/Inc/coupled_comm.h`

`CoupledCommunication` coordinates a simple digital link between master/slave motor-drive boards for coupled simulation and load-test sequences.

***Note: This feature is not being used in the final version of firmware due to noise in the connection jumper causing interference.***

### Enums

```cpp
enum class CouplingMode : uint8_t;
enum class IncomingSignalType : uint8_t;
enum class TimeoutPurpose : uint8_t;
enum class CoupledControlResult : uint8_t;
```

### Simulation Step And Callbacks

```cpp
typedef struct {
    float master_speed;
    float master_torque;
    float slave_speed;
    float slave_torque;
} CoupledSimulationStep_t;

struct CoupledSimulationCallbacks {
    CoupledControlResult (*canStart)(void);
    bool (*start)(bool master);
    void (*applyStep)(float speed, float torque);
    void (*stop)(bool fault);
};
```

### Public API

```cpp
CoupledCommunication(GPIO_TypeDef* tx_port,
                     uint16_t tx_pin,
                     GPIO_TypeDef* rx_port,
                     uint16_t rx_pin,
                     TIM_HandleTypeDef* htim_clk,
                     TIM_HandleTypeDef* htim_timeout);

static void irqHandlerRxRising(void);
static void irqHandlerRxFalling(void);
static void irqHandlerClock(void);
static void irqHandlerTimeout(void);

HAL_StatusTypeDef init(void);
CouplingMode getCouplingMode(void) const;
IncomingSignalType getReceivingSignal(void) const;
IncomingSignalType getOutgoingSignal(void) const;
TimeoutPurpose getTimeoutPurpose(void) const;
uint32_t getReceiveEventCounter(void) const;
GPIO_PinState getRxState(void) const;
GPIO_PinState getTxState(void) const;
void setSimulationCallbacks(CoupledSimulationCallbacks callbacks);
bool startSimulation(void);
void reset(void);
```

Register callbacks before `startSimulation()` so the communication layer can request control startup, apply speed/torque steps, and stop on faults.

## USB CDC Command API

Headers: `Core/Inc/cmd.h`; implementation in `Core/Src/cmd.cpp` and command handlers in `Core/Src/main.cpp`.

Commands are ASCII text lines received over USB CDC. Commands are split on spaces and terminated by `\r`, `\n`, or `\r\n`.

Limits:

| Constant | Value | Meaning |
| --- | --- | --- |
| `RX_RING_SIZE` | 256 | USB RX ring bytes. |
| `CMD_MAX_LEN` | 64 | Maximum parsed command line length. |
| `MAX_ARGC` | 8 | Maximum command tokens. |
| `USB_TX_CHUNK_SIZE` | 500 | Maximum CDC TX chunk submitted at once. |

### Command Processing Functions

```cpp
void usb_printf(const char* format, ...);
bool usb_sendTelemetry(const uint8_t* buffer, uint16_t length);
bool usb_sendBulk(const uint8_t* buffer, uint16_t length);
void usb_tx_service(void);
void usb_tx_onTransmitComplete(void);

bool ring_buffer_write(ring_buffer_t* rx_ring, uint8_t data);
bool read_line_from_ring(ring_buffer_t* rx_ring, char* line, int max_len);
void process_command(const char* cmd_str);

void protectionModePrint(void);
void batteryProtectionPrint(void);
```

Call `usb_tx_service()` frequently from the main loop. Call `usb_tx_onTransmitComplete()` from the CDC transmit-complete callback.

### Supported Commands

| Command | Arguments | Description |
| --- | --- | --- |
| `start` | `[foc\|vvvf]` | Starts motor control. Defaults to FOC if no valid mode is supplied. |
| `stop` | none | Requests controlled ramp-down for active FOC/VVVF where applicable, otherwise stops immediately. |
| `align` | `[reset]` | Runs encoder/electrical zero alignment, or clears alignment state with `reset`. |
| `reset` | none | Stops outputs, clears over-current/under-voltage faults, and resets FOC state. |
| `foc` | `status\|stat` | Prints FOC status. |
| `foc` | `manual` | Enters manual FOC mode. |
| `foc` | `vd\|vq\|id\|iq <value>` | Sets manual FOC d/q voltage or current command. Valid only in manual FOC mode. |
| `sixstep` | none | Starts Hall-sensor six-step commutation. |
| `speed` | `<target> [time]` | Sets speed target in RPM. Optional time applies a ramp. |
| `torque` | `<target> [time]` | Sets torque target in Nm. Optional time applies a ramp. |
| `mod` | `<type>` | Selects modulation type. |
| `duty` | `<a>,<b>,<c>` | Sets direct phase duty values. Use only with current-limited supply. |
| `vec` | `<0-5>` | Applies a six-step switching vector. |
| `tune` | `<subsys> <param> <value\|?>` | Sets or queries tunable parameters. |
| `increment` | `<subsys> <param> <delta>` | Increments tunable parameters. |
| `board` | `[preset]` | Queries or loads ADC calibration preset. |
| `log preset` | `<number>` | Applies a logging preset. |
| `log add` | `<variable>` | Enables a telemetry field. |
| `log rm` | `<variable\|all>` | Disables a telemetry field or clears all fields. |
| `log` | `utf8\|bin` | Selects telemetry output format. |
| `log add adc` | none | Enables bulk ADC sample output. |
| `log rm adc` | none | Disables bulk ADC sample output. |
| `sim` | `start\|status\|reset` | Placeholder simulation command surface. Current handlers are not implemented. |
| `audible` | none | Toggles audible PWM frequency mode. |
| `sin` | `<value>` | Compares `sinf()` with LUT sine timing. |
| `cos` | `<value>` | Compares `cosf()` with LUT cosine timing. |
| `arctan` | `<y> <x>` | Compares `atan2f()` with LUT atan2 timing. |
| `hypot` | `<x> <y>` | Compares `hypotf()` with LUT hypot timing. |

### Modulation Command Values

| CLI value | Firmware enum |
| --- | --- |
| `svpwm` | `ModulationType::SVPWM` |
| `svpwms` | `ModulationType::SVPWM_SUPERPOS` |
| `sym` | `ModulationType::SYM_PWM` |
| `dpwm0` | `ModulationType::DPWM0` |
| `dpwm1` | `ModulationType::DPWM1` |
| `dpwm2` | `ModulationType::DPWM2` |
| `dpwm3` | `ModulationType::DPWM3` |
| `opt`, `optimal` | `ModulationType::OPTIMAL_FINAL` |

### Tunable Parameters

`tune` sets a parameter directly. `increment` adds the supplied value to the existing parameter.

| Subsystem | Parameters |
| --- | --- |
| `speed` | `p`, `i` |
| `id` | `p`, `i` |
| `iq` | `p`, `i` |
| `fw` | `p`, `i` |
| `gain` | `ia`, `ib`, `ic`, `va`, `vb`, `ibatt`, `vbatt` |
| `offset` | `ia`, `ib`, `ic`, `va`, `vb`, `ibatt`, `vbatt` |
| `opt` | `phase`, `phase_advance`, `exit`, `six_exit`, `enter`, `six_enter` |

Values changed by `tune` and `increment` are runtime values. Persisting them across power cycles requires adding storage support or updating defaults in code.

## Telemetry Protocol

Telemetry is sent over USB CDC via the telemetry TX queue. Select fields with `log add`, remove with `log rm`, and select encoding with `log utf8` or `log bin`.

### UTF-8 Telemetry

UTF-8 telemetry is a space-separated line of `name value` pairs ending in `\n`. Fields appear in the same order as the binary telemetry field order.

Example:

```text
rpm 1000.00 rpmsp 1000.00 duty_a 0.50 duty_b 0.25 duty_c 0.75
```

### Binary Telemetry Packet

Binary packets begin with a fixed header followed by enabled fields.

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 1 | `uint8_t` | Header byte `0xAA` |
| 1 | 1 | `uint8_t` | Header byte `0x55` |
| 2 | 1 | `uint8_t` | Packet version (`TELEMETRY_PACKET_VERSION`) |
| 3 | 4 | `uint32_t` | `error_flag` |
| 7 | 1 | `uint8_t` | `MotorControlMode` |
| 8 | 2 | `uint16_t` | High timestamp bits |
| 10 | 4 | `uint32_t` | Low timestamp bits |
| 14 | 4 | `uint32_t` | `print_mask` |
| 18 | 4 | `uint32_t` | `print_mask_ex` |
| 22 | variable | mixed | Enabled fields in fixed order |

Multi-byte values are copied from STM32 memory with `memcpy`, so host parsers should treat them as little-endian on this target.

### Binary Field Order

Fields controlled by `print_mask`:

| CLI name | Type | Source |
| --- | --- | --- |
| `rpm` | `float` | `encoder.getRPM()` |
| `rpmsp` | `float` | FOC target RPM or generic target speed |
| `pos` | `uint16_t` | Encoder position |
| `elpos` | `uint16_t` | Encoder electrical position |
| `duty_a` | `float` | PWM phase A duty |
| `duty_b` | `float` | PWM phase B duty |
| `duty_c` | `float` | PWM phase C duty |
| `ia` | `float` | Phase A current |
| `ib` | `float` | Phase B current |
| `ic` | `float` | Phase C current |
| `va` | `float` | Phase A voltage |
| `vb` | `float` | Phase B voltage |
| `vbatt` | `float` | Battery/DC-link voltage |
| `ibatt` | `float` | Battery current |
| `ia_raw` | `uint16_t` | Raw ADC1 channel 0 |
| `ib_raw` | `uint16_t` | Raw ADC2 channel 0 |
| `ic_raw` | `uint16_t` | Raw ADC3 channel 0 |
| `va_raw` | `uint16_t` | Raw ADC2 channel 1 |
| `vb_raw` | `uint16_t` | Raw ADC1 channel 1 |
| `vbatt_raw` | `uint16_t` | Raw ADC1 channel 2 |
| `ibatt_raw` | `uint16_t` | Raw ADC3 channel 1 |
| `ia_max` | `float` | Rolling max phase A current |
| `ib_max` | `float` | Rolling max phase B current |
| `ic_max` | `float` | Rolling max phase C current |
| `ibatt_max` | `float` | Rolling max battery current |
| `id` | `float` | FOC d-axis current |
| `iq` | `float` | FOC q-axis current |
| `idsp` | `float` | FOC d-axis current setpoint |
| `iqsp` | `float` | FOC q-axis current setpoint |
| `vd` | `float` | FOC d-axis voltage command |
| `vq` | `float` | FOC q-axis voltage command |

Fields controlled by `print_mask_ex`:

| CLI name | Type | Source |
| --- | --- | --- |
| `cp_mode` | integer-like | Current control/protection mode state |
| `m_index` | `float` | FOC modulation index |
| `fw` | integer-like | Field-weakening active flag |
| `umag` | `float` | FOC voltage vector magnitude |
| `imag` | `float` | FOC current vector magnitude |
| `valpha` | `float` | Alpha-axis voltage |
| `vbeta` | `float` | Beta-axis voltage |
| `mod` | integer-like | Active modulation enum value |
| `msixstep` | `float` | Six-step modulation index |
| `region` | integer-like | Optimal modulation region |

### Bulk ADC Telemetry

Bulk ADC capture is enabled with:

```text
log add adc
```

and disabled with:

```text
log rm adc
```

When enabled, the main loop calls `assembleBulkPacket()` for ADC1, ADC2, and ADC3 and queues complete ADC bulk packets with `usb_sendBulk()`.

Bulk ADC packet layout:

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 1 | `uint8_t` | Header byte `0xAA` |
| 1 | 1 | `uint8_t` | Header byte `0x46` |
| 2 | 1 | `uint8_t` | ADC sample packet version |
| 3 | 1 | `uint8_t` | ADC id (`1`, `2`, or `3`) |
| 4 | 2 | `uint16_t` | Sample count |
| 6 | 1 | `uint8_t` | ADC resolution bits |
| 7 | 4 | `uint32_t` | Sequence number |
| 11 | 4 | `uint32_t` | Timestamp in microseconds |
| 15 | 4 | `float` | Shunt calibration |
| 19 | 4 | `float` | Offset calibration |
| 23 | variable | `uint16_t[]` | ADC samples |

## Interrupt Integration Points

The main application uses these callback routes:

| Interrupt/Callback | Project handler |
| --- | --- |
| `HAL_GPIO_EXTI_Callback()` Hall pins | `HallSensor::irqHandlerRising/Falling()` and `sixStepCommutation()` when active |
| `HAL_GPIO_EXTI_Callback()` encoder index | `Encoder::irqHandlerIndex()` |
| TIM8 period elapsed | `sampleAndProtect()` followed by `focTick()`, `vvvfRampUp()`, or `alignRotor()` depending on mode |
| TIM16 period elapsed | `Encoder::irqHandlerSpeed()` and `speedControl()` for FOC modes |
| TIM6 period elapsed | `printTelemetryBinary()` |
| TIM2 period elapsed | `printTelemetryUTF8()` |
| TIM4 period elapsed | `Encoder::irqHandlerEncoderOverflow()` |
| TIM15 period elapsed | `Encoder::irqHandlerTimerOverflow()` |
| `HAL_ADC_ConvHalfCpltCallback()` | `ADCSampler::irqConvHalfCplt()` |
| `HAL_ADC_ConvCpltCallback()` | `ADCSampler::irqConvCplt()` |
| USB CDC receive | `ring_buffer_write()` for each received byte |
| USB CDC transmit complete | `usb_tx_onTransmitComplete()` |

## Safety Notes

- Direct `duty` and `vec` commands bypass normal control paths. Use them only with a current-limited supply.
- Protection mode blocks restart attempts until `reset` clears the relevant error flags.
- `BATTERY_PROTECTION` disables selected unsafe commands when the firmware is configured for battery-protected operation.
- FOC assumes encoder alignment has been completed before closed-loop operation.
- ADC mean functions require power-of-two sample counts.
- USB TX queue functions copy caller data before returning, but may return `false` if the selected queue does not have enough space for the whole packet.
