cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(rorando)

# Initialize the SDK
pico_sdk_init()

add_executable(synth synth.c)

# pull in common dependencies and additional spi hardware support
target_link_libraries(synth pico_stdlib hardware_spi)

# create map/bin/hex file etc.
pico_add_extra_outputs(synth)
