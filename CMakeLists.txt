cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(rorando)

# Initialize the SDK
pico_sdk_init()

add_executable(synth synth.c)

# make header file of .pio so it can be imported in c file
pico_generate_pio_header(synth ${CMAKE_CURRENT_LIST_DIR}/synth.pio)

# pull in common dependencies and additional spi hardware support
target_link_libraries(synth pico_stdlib hardware_spi hardware_pio)

# create map/bin/hex file etc.
pico_add_extra_outputs(synth)
