# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_SUPPRESS_REGENERATION ON)

# Workaround: prevent FindDoxygen from running inside Pico SDK docs on Windows
# paths with non-ASCII characters, where template generation can fail.
set(CMAKE_DISABLE_FIND_PACKAGE_Doxygen TRUE)
set(DOXYGEN_FOUND FALSE)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ISTE-Project C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ISTE-Project
    app/main.cpp
    app/app_init.cpp
    app/app_data.cpp
    drivers/bmp280/bmp280.cpp
    drivers/ina219/ina219.cpp
    drivers/voltage_divider/voltage_divider.cpp
    drivers/sx1278/sx1278.cpp
    drivers/soil_moisture/soil_moisture.cpp
    hal/i2c/i2c.cpp
    hal/spi/spi.cpp
    hal/gpio/gpio.cpp
    hal/adc/adc.cpp
    hal/timer/timer.cpp
    utils/assert/assert.cpp
)

pico_set_program_name(ISTE-Project "ISTE-Project")
pico_set_program_version(ISTE-Project "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ISTE-Project 0)
pico_enable_stdio_usb(ISTE-Project 1)

# Pico SDK libraries required by the current project sources.
set(PROJECT_SDK_LIBS
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_i2c
    hardware_adc
    hardware_uart
    hardware_timer
    hardware_pwm
    hardware_watchdog
)

target_link_libraries(ISTE-Project PRIVATE
    ${PROJECT_SDK_LIBS}
)

# Add the standard include files to the build
target_include_directories(ISTE-Project PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Avoid linker map output on non-ASCII Windows paths.
pico_add_dis_output(ISTE-Project)
pico_add_hex_output(ISTE-Project)
pico_add_bin_output(ISTE-Project)
if (NOT (PICO_NO_UF2 OR PICO_NO_PICOTOOL))
    pico_add_uf2_output(ISTE-Project)
endif()

