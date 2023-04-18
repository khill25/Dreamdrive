add_executable(dreamdrivecast_mcu2
    mcu2.c
)


target_include_directories(dreamdrivecast_mcu2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# pull in common dependencies
target_link_libraries(dreamdrivecast_mcu2
    pico_multicore
    pico_stdlib
    hardware_pio

    dreamdrivecast_shared
)

pico_generate_pio_header(dreamdrivecast_mcu2
    ${CMAKE_CURRENT_LIST_DIR}/sega_cs_detect.pio
)

set(PICO_COPY_TO_RAM 1)

# stdio config
pico_enable_stdio_usb(dreamdrivecast_mcu2 0)
pico_enable_stdio_uart(dreamdrivecast_mcu2 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(dreamdrivecast_mcu2)