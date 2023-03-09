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
    stdio_async_uart
    hardware_pio
)

# stdio config
pico_enable_stdio_usb(dreamdrivecast_mcu2 0)
pico_enable_stdio_uart(dreamdrivecast_mcu2 1)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(dreamdrivecast_mcu2)