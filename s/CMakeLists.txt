cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico-synth C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(master master.c)

# make header file of .pio so it can be imported in c file
pico_generate_pio_header(master ${CMAKE_CURRENT_LIST_DIR}/master.pio)

target_include_directories(master PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(master PRIVATE pico_stdlib pico_time hardware_pio hardware_gpio hardware_irq hardware_adc hardware_pwm hardware_spi tinyusb_device tinyusb_board)

# create uf2/map/bin/hex file etc.
pico_add_extra_outputs(master)
