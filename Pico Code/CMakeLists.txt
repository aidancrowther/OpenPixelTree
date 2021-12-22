# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/aidancrowther/Documents/Projects/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Christmas-Lights C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Christmas-Lights Christmas-Lights.cpp WS2812.cpp)
pico_generate_pio_header(Christmas-Lights ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)
pico_generate_pio_header(Christmas-Lights ${CMAKE_CURRENT_LIST_DIR}/clocked_input.pio)

pico_set_program_name(Christmas-Lights "Christmas-Lights")
pico_set_program_version(Christmas-Lights "0.1")

pico_enable_stdio_uart(Christmas-Lights 1)
pico_enable_stdio_usb(Christmas-Lights 1)

# Add the standard library to the build
target_link_libraries(Christmas-Lights pico_stdlib pico_multicore pico_bootrom hardware_spi hardware_dma hardware_pio)

pico_add_extra_outputs(Christmas-Lights)

