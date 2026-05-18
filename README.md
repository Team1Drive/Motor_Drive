# M.Eng. Project - Motor Drive Codebase

This repository is the main motor drive microcontroller firmware codebase for M.Eng. Project Team 1.

This project aims to investigate different methods of modulation and compare their performances across linear and overmodulation region. For the hardware testing part of the project, a custom PCB together with a back-to-back drive and load motor test rig is developed for physical testing of the modulation algorithms. The PCB integrates a STM32H725VGT microcontroller to implement FOC, which has been programmed using C/C++. This repository consists of the firmware used on the microcontroller.

A separate software has been developed for Windows to control and display the telemetry data in a GUI. This can be found at [Tuning Master](https://github.com/Gilbert526/Motor_Drive_Frontend).

## File Structure

    Root
    ├── Core
    │   ├── Inc/
    │   │   ├── adc_sampler.cpp         # Class for ADC and DMA sampling
    │   │   ├── adc_sampler.h           # Declaration of class and functions for ADC and DMA sampling
    │   │   ├── adc.c                   # CubeMX generated file
    │   │   ├── digitalio.cpp
    │   │   ├── digitalio.h
    │   │   ├── dma.c                   # CubeMX generated file
    │   │   ├── encoder.cpp
    │   │   ├── encoder.h
    │   │   ├── gpio.c                  # CubeMX generated file
    │   │   ├── hallsensor.cpp
    │   │   ├── hallsensor.h
    │   │   ├── main.c                  # CubeMX generated main file
    │   │   ├── main.cpp                # Main programme
    │   │   ├── modulation.cpp
    │   │   ├── modulation.h
    │   │   ├── pwm3phase_timer.cpp
    │   │   ├── pwm3phase_timer.h
    │   │   ├── stm32h7xx_hal_msp.c     # CubeMX generated file
    │   │   ├── stm32h7xx_it.c          # CubeMX generated file
    │   │   ├── syscalls.c              # CubeMX generated file
    │   │   ├── sysmem.c                # CubeMX generated file
    │   │   ├── system_stm32h7xx.c      # CubeMX generated file
    │   │   ├── tim.c                   # CubeMX generated file
    │   │   ├── ustimer.cpp
    │   │   └── ustimer.h
    │   │
    │   └── Src                     
    │       ├── adc_sampler.cpp         # Class for ADC and DMA sampling
    │       ├── adc_sampler.h           # Declaration of class and functions for ADC and DMA sampling
    │       ├── adc.c                   # CubeMX generated file
    │       ├── digitalio.cpp
    │       ├── digitalio.h
    │       ├── dma.c                   # CubeMX generated file
    │       ├── encoder.cpp
    │       ├── encoder.h
    │       ├── gpio.c                  # CubeMX generated file
    │       ├── hallsensor.cpp
    │       ├── hallsensor.h
    │       ├── main.c                  # CubeMX generated main file
    │       ├── main.cpp                # Main programme
    │       ├── modulation.cpp
    │       ├── modulation.h
    │       ├── pwm3phase_timer.cpp
    │       ├── pwm3phase_timer.h
    │       ├── stm32h7xx_hal_msp.c     # CubeMX generated file
    │       ├── stm32h7xx_it.c          # CubeMX generated file
    │       ├── syscalls.c              # CubeMX generated file
    │       ├── sysmem.c                # CubeMX generated file
    │       ├── system_stm32h7xx.c      # CubeMX generated file
    │       ├── tim.c                   # CubeMX generated file
    │       ├── ustimer.cpp
    │       └── ustimer.h
    │
    ├── Drivers/
    ├── Middlewares/ST/STM32_USB_Device_Library/
    ├── USB_DEVICE/       
    ├── cmake/
    ├── out/build/gcc-arm
    │   ├── .cmake/
    │   ├── cmake/
    │   ├── CMakeFiles/
    │   ├── cmake_install.cmake
    │   ├── CMakeCache.txt
    │   ├── compile_commands.json
    │   ├── Makefile
    │   ├── Motor_Drive.bin             # Compiled binary file for flashing
    │   ├── Motor_Drive.elf
    │   └── Motor_Drive.map
    │
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

An example test project can be found at [Test project]("https://github.com/Gilbert526/MEng_Project-CMake_Demo_Project").

A brief tutorial of git is available [here]("use_of_git.md").
