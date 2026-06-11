# M.Eng. Project - Motor Drive Codebase

This repository is the main motor drive microcontroller firmware codebase for M.Eng. Project Team 1.

This project aims to investigate different methods of modulation and compare their performances across linear and overmodulation region. For the hardware testing part of the project, a custom PCB together with a back-to-back drive and load motor test rig is developed for physical testing of the modulation algorithms. The PCB integrates a STM32H725VGT microcontroller to implement FOC, which has been programmed using C/C++. This repository consists of the firmware used on the microcontroller.

The software has been written using a combination of STM32CubeMX generated library in C and custom classes in C++. It is heavily oriented towards STM32H725VGT with pins defined under [Motor_Drive.ioc](Motor_Drive.ioc). Subsequent development based on this project is advised to pay close attention to the definition of the hardware usage including the on chip peripherals as they may differ from chip to chip. A detailed list of all the available public member functions can be found in [API Documentation](API_DOCUMENTATION.md)

A separate software has been developed for Windows to control and display the telemetry data in a GUI. This can be found at [Tuning Master](https://github.com/Gilbert526/Motor_Drive_Frontend).

## File Structure

    Root
    ├── Core
    │   ├── Inc/
    │   │   ├── adc.h                   # CubeMX generated file
    │   │   ├── adc_sampler.h           # Non-generated: ADC and DMA sampler interface
    │   │   ├── cmd.h                   # Non-generated: USB CLI command interface
    │   │   ├── coupled_comm.h          # Non-generated: coupled communication interface
    │   │   ├── digitalio.h             # Non-generated: digital I/O helpers
    │   │   ├── dma.h                   # CubeMX generated file
    │   │   ├── encoder.h               # Non-generated: encoder interface
    │   │   ├── fft_analyzer.h          # Non-generated: FFT analyzer interface
    │   │   ├── foc.h                   # Non-generated: field-oriented control interface
    │   │   ├── gpio.h                  # CubeMX generated file
    │   │   ├── hallsensor.h            # Non-generated: Hall sensor interface
    │   │   ├── lut.h                   # Non-generated: lookup table data
    │   │   ├── main.h                  # CubeMX generated file
    │   │   ├── math_helpers.h          # Non-generated: math helper interface
    │   │   ├── modulation.h            # Non-generated: modulation algorithms interface
    │   │   ├── parameters.h            # Non-generated: tunable firmware parameters
    │   │   ├── pwm3phase_timer.h       # Non-generated: three-phase PWM timer interface
    │   │   ├── stm32h7xx_hal_conf.h    # CubeMX generated file
    │   │   ├── stm32h7xx_it.h          # CubeMX generated file
    │   │   ├── tim.h                   # CubeMX generated file
    │   │   ├── timer.h                 # Non-generated: timer helper interface
    │   │   └── ustimer.h               # Non-generated: microsecond timer interface
    │   │
    │   └── Src/
    │       ├── adc.c                   # CubeMX generated file
    │       ├── adc_sampler.cpp         # Non-generated: ADC and DMA sampler implementation
    │       ├── cmd.cpp                 # Non-generated: USB CLI command handling
    │       ├── coupled_comm.cpp        # Non-generated: coupled communication implementation
    │       ├── digitalio.cpp           # Non-generated: digital I/O implementation
    │       ├── dma.c                   # CubeMX generated file
    │       ├── encoder.cpp             # Non-generated: encoder implementation
    │       ├── fft_analyzer.cpp        # Non-generated: FFT analyzer implementation
    │       ├── foc.cpp                 # Non-generated: field-oriented control implementation
    │       ├── gpio.c                  # CubeMX generated file
    │       ├── hallsensor.cpp          # Non-generated: Hall sensor implementation
    │       ├── main.c                  # CubeMX generated main file
    │       ├── main.cpp                # Non-generated: main programme
    │       ├── math_helpers.cpp        # Non-generated: math helper implementation
    │       ├── modulation.cpp          # Non-generated: modulation algorithms implementation
    │       ├── pwm3phase_timer.cpp     # Non-generated: three-phase PWM timer implementation
    │       ├── stm32h7xx_hal_msp.c     # CubeMX generated file
    │       ├── stm32h7xx_it.c          # CubeMX generated file
    │       ├── syscalls.c              # CubeMX generated file
    │       ├── sysmem.c                # CubeMX generated file
    │       ├── system_stm32h7xx.c      # CubeMX generated file
    │       ├── tim.c                   # CubeMX generated file
    │       ├── timer.cpp               # Non-generated: timer helper implementation
    │       └── ustimer.cpp             # Non-generated: microsecond timer implementation
    │
    ├── Drivers/                        # STM32 HAL and CMSIS vendor files
    ├── Middlewares/                    # STM32 USB library and CMSIS-DSP vendor files
    ├── USB_DEVICE/                     # CubeMX generated USB device files
    ├── Plot/                           # Non-generated: plotting and debug scripts
    ├── cmake/
    ├── out/build/gcc-arm/              # Generated build output
    │   ├── Motor_Drive.bin             # Compiled binary file for flashing
    │   ├── Motor_Drive.elf             # Compiled ELF image
    │   └── Motor_Drive.map             # Linker map file
    ├── CMakeLists.txt
    ├── CMakePresets.json
    ├── Motor_Drive.ioc                 # STM32CubeMX project file
    ├── startup_stm32h725xx.s           # Startup assembly file
    ├── STM32H725XG_FLASH.ld            # Linker script
    ├── use_of_git.md                   # Non-generated: Git workflow notes
    └── README.md

## Flashing Programme onto STM32

To set the STM32 into Bootloader mode, pull the `BOOT0` pin to 5 V and press the RESET button.

Open STM32CubeProgrammer.

On the left-hand side select `Erasing & Programming` tab.

For the device selection on the right, select USB from the drop-down menu.

The device should show up in STM32CubeProgrammer as a USB-DFU device.

Click `Connect` to connect, a green dot will show if successful

Select the path to the `.bin`file, then click `Program` to flash the programme, no other changes are needed, a success message will show if flashing is successful.

Unplug the jumper cable on the board and press RESET, the STM32 should be running the programme.

## MCU Command Table

The firmware has a build in CLI that can be accessed when connected to PC using USB serial monitor for the purpose of tuning and debugging. The table of commands is listed below.

### `start {foc|vvvf}`

Start motor with desired control method, defaulted to start with FOC if argument is left empty.

### `stop`

Stop the motor with immediate effect. If running FOC or VVVF, a speed ramp to 0 is requested and can be interrupted by another stop command.

### `align [reset]`

Run the aligning sequence, if reset is selected then current align offset will be deleted.

### `reset`

Reset protection state.

### `foc {status|stat|manual}`

Return internal FOC operating values if argument is status or stat, or enter manual FOC mode if argument is manual.

### `foc {vd|vq|id|iq} <value>`

Manually setting the d-q voltage or current setpoint, only available when in FOC manual mode.

### `sixstep`

Start motor with six-step commutation using hall effect sensors.

### `speed <target> [time]`

Set speed target in RPM directly, optional positive time generates a speed ramp.

### `torque <target> [time]`

Set torque target in Nm directly, optional positive time generates a torque ramp.

### `mod <modulation>`

Select modulation type, valid types: `svpwm`, `svpwms`, `sym`, `dpwm0`, `dpwm1`, `dpwm2`, `dpwm3`.

### `duty <duty_A, duty_B, duty_C>`

Set duty ratio for each phase directly, must be in range `[0.0, 1.0]` to set duty cycle, `-1.0` on any channel will disable phase output.

### `vec <0-5>`

Applies one of the switching states in the switch table for six-step commutation

### `tune <subsys> <param> <value>`

**`<subsys>` Select subsystem**

**`<param>` Select parameter to tune**

**`<value>` New value to be set**

Tune specific value. Value may be `?` for query. Subsystem include `speed`, `fw`, `id`, `iq` with parameter `p`, `i`; `offset`, `gain` with `ia`, `ib`, `ic`, `va`, `vb`, `vbatt`, `ibatt`; `opt` with `phase`, `phase_advance`, `exit`, `six_exit`, `enter`, `six_enter`.

***SAVE DATA BEFORE POWER DOWN***

### `increment <subsys> <param> <value>`

Increment specific tuneable value, support the same names as tune.

***SAVE DATA BEFORE POWER DOWN***

### `board [board_num]`

Select ADC tuning preset for board, leave argument blank for query. Currently support `[0, 2]`

### `log preset <number>`

Apply logging preset.

### `log {add|rm} <variable>`

Add or remove a variable to be logged, variables include `rpm`, `rpmsp`, `pos`, `elpos`, `duty_a`, `duty_b`, `duty_c`, `ia`, `ib`, `ic`, `va`, `vb`, `vbatt`, `ibatt`, `ia_raw`, `ib_raw`, `ic_raw`, `va_raw`, `vb_raw`, `vbatt_raw`, `ibatt_raw`, `ia_max`, `ib_max`, `ic_max`, `ibatt_max`, `id`, `iq`, `idsp`, `iqsp`, `vd`, `vq`, `cp_mode`, `m_index`, `fw`, `umag`, `imag`, `valpha`, `vbeta`, `mod`, `msixstep`, `region`, `adc`.

### `log {bin|utf8}`

Select log information output format.


## Development Environment

Before you begin, make sure you are familiar with developing STM32 using CMake toolchain.

An example test project can be found at [Test project](https://github.com/Gilbert526/MEng_Project-CMake_Demo_Project).

A brief tutorial of git is available [here](use_of_git.md).
