cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()

project(pico_lamp_array)
add_executable(pico_lamp_array)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(pico_lamp_array ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(pico_lamp_array PRIVATE ws2812.c main.c usb_descriptors.c)

target_include_directories(pico_lamp_array PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(pico_lamp_array PRIVATE pico_stdlib pico_multicore hardware_pio tinyusb_device tinyusb_board)
pico_add_extra_outputs(pico_lamp_array)

pico_enable_stdio_usb(pico_lamp_array 0)
pico_enable_stdio_uart(pico_lamp_array 1)