# M.Eng. Project - Motor Drive Codebase

## Development Environment

Before you begin, make sure you are familiar with developing STM32 using CMake toolchain.

An example test project can be found at [Test project]("https://github.com/Gilbert526/MEng_Project-CMake_Demo_Project").

## Git

Ensure [Git](https://git-scm.com/downloads) is installed on your PC.

Login your account in Git and you should be able to take advantage of the source control tab in VS Code.

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



## File Structure

    Root
    ├── Core
    │   ├── Inc                     
    │   └── Src                     
    │       ├── adc_sampler.cpp
    │       ├── adc_sampler.h
    │       ├── adc.c
    │       ├── digitalio.cpp
    │       ├── digitalio.h
    │       ├── dma.c
    │       ├── encoder.cpp
    │       ├── encoder.h
    │       ├── gpio.c
    │       ├── hallsensor.cpp
    │       ├── hallsensor.h
    │       ├── main.c
    │       ├── main.cpp                # Main programme
    │       ├── modulation.cpp
    │       ├── modulation.h
    │       ├── pwm3phase_timer.cpp
    │       ├── pwm3phase_timer.h
    │       ├── stm32h7xx_hal_msp.c
    │       ├── stm32h7xx_it.c
    │       ├── syscalls.c
    │       ├── sysmem.c
    │       ├── system_stm32h7xx.c
    │       ├── tim.c
    │       ├── ustimer.cpp
    │       └── ustimer.h
    │
    ├── Drivers/                     # Python implementation (NumPy/SciPy-based)
    │   ├── src/
    │   ├── tests/
    │   └── notebooks/             # Jupyter notebooks for demonstration
    ├── Middlewares\ST\STM32_USB_Device_Library/
    ├── USB_DEVICE/       
    ├── cmake/
    ├── out/                      
    └── README.md
