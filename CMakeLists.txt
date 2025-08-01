# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(2wiCC_Comms C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(2wiCC_Comms 2wiCC_Comms.c usb-descriptors.c)

pico_set_program_name(2wiCC_Comms "2wiCC_Comms")
pico_set_program_version(2wiCC_Comms "1.0")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(2wiCC_Comms 0)
pico_enable_stdio_usb(2wiCC_Comms 0)

# Add the standard library to the build
target_link_libraries(2wiCC_Comms
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(2wiCC_Comms PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(2wiCC_Comms 
        hardware_spi
        hardware_pio
        tinyusb_device
        hardware_flash
        hardware_pio
        pico_multicore
        )

pico_generate_pio_header(2wiCC_Comms ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

pico_add_extra_outputs(2wiCC_Comms)

