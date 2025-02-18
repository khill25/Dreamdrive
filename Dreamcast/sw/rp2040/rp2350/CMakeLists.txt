add_executable(dreamdrivecast_rp2350
    rp2350.c
    # sdcard/internal_sd_card.c
    # sdcard/hw_config.c
    # sdcard/simple.c
)

pico_generate_pio_header(dreamdrivecast_rp2350
    ${CMAKE_CURRENT_LIST_DIR}/ide_handling.pio
)

target_include_directories(dreamdrivecast_rp2350 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# pull in common dependencies
target_link_libraries(dreamdrivecast_rp2350
    pico_multicore
    pico_stdlib
    hardware_pio

    # 3rd party libs
    # FreeRTOS-Kernel-Static # FreeRTOS kernel, static allocations
    # FatFs_SPI # SDCard and FatFS
)

# Copy program to ram during boot
set(PICO_COPY_TO_RAM 1)

# stdio config
pico_enable_stdio_usb(dreamdrivecast_rp2350 0)
pico_enable_stdio_uart(dreamdrivecast_rp2350 1)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(dreamdrivecast_rp2350)
