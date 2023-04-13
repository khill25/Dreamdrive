add_library(dreamdrivecast_shared INTERFACE)

target_sources(dreamdrivecast_shared INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/shared.c
    ${CMAKE_CURRENT_LIST_DIR}/serial_bridge/serial_bridge.c
    ${CMAKE_CURRENT_LIST_DIR}/sega_databus/sega_databus.c
    ${CMAKE_CURRENT_LIST_DIR}/sega_packet_interface.c
)

pico_generate_pio_header(dreamdrivecast_shared
    ${CMAKE_CURRENT_LIST_DIR}/serial_bridge/serial_bridge.pio
)

target_include_directories(dreamdrivecast_shared INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(dreamdrivecast_shared INTERFACE ${CMAKE_CURRENT_LIST_DIR}/serial_bridge)
