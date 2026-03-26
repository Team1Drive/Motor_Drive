# M.Eng. Project - Motor Drive Codebase

## Development Environment

Before you begin, make sure you are familiar with developing STM32 using CMake toolchain.

An example test project can be found at [Test project]("https://github.com/Gilbert526/MEng_Project-CMake_Demo_Project").

## Git

Ensure [Git](https://git-scm.com/downloads) is installed on your PC.

Login your account in Git and you should be able to take advantage of the source control tab in VS Code.

**Ensure you PULL FIRST BEFORE PUSHING any changes**

### Common Git Terminology

| Term          | Meaning                                                                       |
|:--------------|:------------------------------------------------------------------------------|
|Repository     |The project folder that Git tracks.                                            |
|Branch         |A separate line of development. Use branches to work on features or fixes.     |
|Commit         |A snapshot of your changes with a descriptive message.                         |
|Push           |Upload your local commits to the remote repository on GitHub.                  |
|Pull           |Download the latest changes from the remote repository to your local machine.  |
|Pull Request   |A request to merge your branch into another branch (usually main).             |

### The Basic Workflow

1. Update your local main branch – get the latest code.
2. Create a new branch for your work.
3. Make changes and commit them locally.
4. Push the branch to GitHub.
5. Open a Pull Request (PR) to merge your changes.
6. After review, the PR is merged and you can delete your branch.

### Using the Command Line

Cloning the repository for the first time

    cd "your-desired-location"
    git clone https://github.com/Team1Drive/Motor_Drive.git

Pulling from the main branch

    git pull origin main

Create and switch to a new branch

    # Use a descriptive name, e.g., feature/overmodulation or fix/interface
    git checkout -b your-branch-name

Add and commit

    # Add all changes (or use `git add <file>` for specific files)
    git add .

    # Commit with a clear message
    git commit -m "Add login form validation"

Pushing your branch

    git push origin your-branch-name

### Using GUI in VS Code

#### 1. Clone the repository

- Open VS Code.

- Press `Ctrl+Shift+P` and type `Git: Clone`.

- Paste the repository URL and select a local folder.

#### 2. Update your local `main` branch

- Click the Source Control icon in the left sidebar (or `Ctrl+Shift+G`).

- In the bottom-left corner, click the branch name (usually `main`) and select `main` from the list.

- Click the `…` menu and choose `Pull` to get the latest changes.

#### 3. Create and switch to a new branch

- Click the branch name in the bottom-left corner.

- Choose `Create new branch…` and enter a name (e.g., `feature/overmodulation`).

#### 4. Make changes and commit them

- Edit files as needed. Modified files appear in the Source Control view.

- Hover over a file and click the `+` to stage it (or click the `+` next to Changes to stage all).

- Enter a commit message in the text box above.

- Click the checkmark Commit button to commit locally.

#### 5. Push your branch to GitHub

- Click the `…` menu in Source Control and choose `Push`.

- If this is the first push from this branch, VS Code may ask if you want to publish the branch – confirm.

### Opening a Pull Request

You can do this from VS Code (if you have the GitHub Pull Requests extension) or directly on GitHub.

#### Option A: In VS Code (with extension)

Click the `Pull Requests` icon in the left sidebar (or install the extension first).

Click the `+` to create a new PR, select your branch as the `compare` branch, and `main` as `base`.

Fill in the details and click `Create`.

#### Option B: On GitHub

Go to the repository page on GitHub.

You’ll likely see a prompt to create a PR from your recently pushed branch. Click `Compare & pull request`.

Otherwise, go to the `Pull requests` tab, click `New pull request`, select your branch as `compare` and `main` as `base`, then `Create pull request`.

## File Structure

    Root
    ├── Core
    │   ├── Inc/
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

## Programming Convention

### Naming

Use `FULL_CAPITAL` for defined macros and enumeration.

e.g.

    enum class MotorControlMode : uint8_t {
        MOTOR_STOP,
        MOTOR_STARTUP,
        MOTOR_SIX_STEP,
        MOTOR_FOC_LINEAR,
        MOTOR_FOC_DPWM
    };

Always name macros with `DEVICE_DESCRIPTION_UNIT`.

e.g.

    #define ADC1_NUM_CHANNELS   3U
    #define ADC2_NUM_CHANNELS   2U
    #define ADC3_NUM_CHANNELS   2U

Use `camelCase()` for functions.

e.g.

    void setDuty(float duty_A, float duty_B, float duty_C);

Use `snake_case` for variables.

e.g.

    bool use_proc_buffer;

## Flashing Programme onto STM32

To set the STM32 into Bootloader mode, pull the `BOOT0` pin to 5 V and press the RESET button.

Open STM32CubeProgrammer.

On the left-hand side select `Erasing & Programming` tab.

For the device selection on the right, select USB from the drop-down menu.

The device should show up in STM32CubeProgrammer as a USB-DFU device.

Click `Connect` to connect, a green dot will show if successful

Select the path to the `.bin`file, then click `Program` to flash the programme, no other changes are needed, a success message will show if flashing is successful.

Unplug the jumper cable on the board and press RESET, the STM32 should be running the programme.

## MCU Command

### `start`

Starting the motor with open loop VVVF.

### `stop`

Immediately stopping the motor.

### `sixstep`

Run motor in six-step commutation.

### `speed <target_speed>`

Set target speed for speed loop.

**`<target_speed>` Range -5000 to 5000 in RPM**

### `duty <duty_A, duty_B, duty_C>`

Set duty cycle for each phase.

**`<duty_A, duty_B, duty_C>` Range -1.0 to 1.0, value below 0.0 will disable the phase**

### `vec <vector>`

Set three-phase output that follows a specific field orientation.

**`<vector>` Range 0 to 5 representing 6 orientation**

### `tune <subsys> <param> <value>`

Tuning specific value. ***SAVE DATA BEFORE POWER DOWN***

**`<subsys>` Select subsystem**

**`<param>` Select parameter to tune**

**`<value>` New value to be set**

| Subsystem | Parameter                                 |
|:---------:|:------------------------------------------|
|`Speed`    |`p` `i` `d`                                |
|`Current`  |`p` `i`                                    |
|`adc`      |`ia` `ib` `ic` `va` `vb` `ibatt` `vbatt`   |

### `print <operation> <variable>`

Print selected variable through USB COM.

**`<operation>` add or remove variable**

**`<variable>` select variable to print**

| Operation | Variable                                  |
|:---------:|:------------------------------------------|
|`add` `rm` |`hall` `hall_dec` `rpm` `pos` `duty_a` `duty_b` `duty_c` `ia` `ib` `ic` `va` `vb` `vbatt` `ibatt` `ia_raw` `ib_raw` `ic_raw` `va_raw` `vb_raw` `vbatt_raw` `ibatt_raw`     |

### `print <format>`

**`<operation>` select `utf8` for readable text in UTF8 or `bin` for binary**
