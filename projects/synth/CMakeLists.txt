add_executable(synth synth.c)

# pull in common dependencies and additional spi hardware support
target_link_libraries(
        synth
        pico_stdlib
        hardware_spi
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(synth)

# add url via pico_set_program_url
example_auto_set_url(synth)
