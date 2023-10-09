# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_PATH "/YOUR_PICO_SDK_PATH/pico-sdk")

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_parrot C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico_parrot main.cpp )

pico_set_program_name(pico_parrot "pico_parrot")
pico_set_program_version(pico_parrot "0.1")

pico_enable_stdio_uart(pico_parrot 0)
pico_enable_stdio_usb(pico_parrot 1)

# Add the standard library to the build
target_link_libraries(pico_parrot pico_stdlib)

# Add any user requested libraries
target_link_libraries(pico_parrot
        hardware_dma
        hardware_pio
        hardware_timer
        hardware_clocks
        hardware_pwm
        hardware_adc
        )

pico_add_extra_outputs(pico_parrot)

