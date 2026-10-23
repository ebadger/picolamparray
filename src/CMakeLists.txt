cmake_minimum_required(VERSION 3.12)

# Pull in the Raspberry Pi Pico SDK. The PICO_SDK_PATH environment variable
# must point at a pico-sdk checkout (with its TinyUSB submodule initialized).
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()

project(pico_lamp_array)
add_executable(pico_lamp_array)

# generate the header file into the source tree as it is included in the RP2040 datasheet
# (convention inherited from pico-examples): pioasm turns ws2812.pio into
# generated/ws2812.pio.h, which main.c includes.
pico_generate_pio_header(pico_lamp_array ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

# Application and LED driver (main.c); USB descriptors and HID LampArray reports (usb_descriptors.c).
target_sources(pico_lamp_array PRIVATE main.c usb_descriptors.c)

# Lets the sources, and TinyUSB, find this project's headers (including tusb_config.h).
target_include_directories(pico_lamp_array PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# pico_multicore runs the LED driver on core 1, hardware_pio/hardware_dma generate
# the LED signal, tinyusb_device is the USB stack and tinyusb_board provides
# board_init(), board_led_write() and board_millis().
target_link_libraries(pico_lamp_array PRIVATE pico_stdlib pico_multicore hardware_pio hardware_dma tinyusb_device tinyusb_board pico_malloc pico_mem_ops)
# Also produce the .uf2 file for drag-and-drop flashing (plus .bin/.hex/.dis).
pico_add_extra_outputs(pico_lamp_array)

# The app provides its own TinyUSB device (HID only, no CDC), so the SDK's USB
# stdio driver is built as a no-op (expect a compiler warning saying so).
# To see printf output, enable UART stdio instead (UART0: TX on GP0, 115200 baud).
pico_enable_stdio_usb(pico_lamp_array 1)
pico_enable_stdio_uart(pico_lamp_array 0)