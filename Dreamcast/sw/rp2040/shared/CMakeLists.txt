add_library(dreamdrivecast_shared INTERFACE)

pico_generate_pio_header(dreamdrivecast_shared
    ${CMAKE_CURRENT_LIST_DIR}/pio_uart/pio_uart.pio
)

pico_generate_pio_header(dreamdrivecast_shared
    ${CMAKE_CURRENT_LIST_DIR}/sega_databus/sega_databus.pio
)

target_sources(dreamdrivecast_shared INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/shared.c
    ${CMAKE_CURRENT_LIST_DIR}/pio_uart/pio_uart.c
    ${CMAKE_CURRENT_LIST_DIR}/sega_databus/sega_databus.c
    ${CMAKE_CURRENT_LIST_DIR}/sega_packet_interface.c
)

target_include_directories(dreamdrivecast_shared INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(dreamdrivecast_shared INTERFACE ${CMAKE_CURRENT_LIST_DIR}/sega_databus)
target_include_directories(dreamdrivecast_shared INTERFACE ${CMAKE_CURRENT_LIST_DIR}/pio_uart)
