cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
include(pico_libs_import.cmake)

project(pico-firmware C CXX ASM)
# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(pico-firmware)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

target_sources(pico-firmware PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)
        
target_include_directories(pico-firmware PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pico-firmware pico_stdlib pico_stdio hardware_gpio ili934x hardware_spi)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico-firmware)
