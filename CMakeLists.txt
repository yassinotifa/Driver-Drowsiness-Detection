# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(blink C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.c
    led.c
    ir.c
    buzzer.c
    imu.c
    lcd_driver.c
    tact_switch.c
    motor.c
    # esp01s.c
    # dwm_pico_5110_LCD.c
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_i2c 
    hardware_spi
    hardware_pwm
    hardware_clocks
    # pico_platform
    # pico_stdlib mpu6050
    # Arduino_LSM6DSOX
    # pico_nokia_lcd
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_simple_hardware_target(i2c)
pico_mirrored_target_link_libraries(hardware_i2c INTERFACE pico_time hardware_resets hardware_clocks)
