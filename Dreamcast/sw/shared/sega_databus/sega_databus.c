/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include "pico/stdlib.h"
#include "sega_databus.h"
#include "sega_packet_interface.h"
#include "mcu2_pins.h"

volatile uint16_t cached_control_line_register = 0;

volatile bool cached_port_function_register_cs0 = 0;
volatile bool cached_port_function_register_cs1 = 0;
volatile bool cached_port_function_register_da2 = 0;
volatile bool cached_port_function_register_da1 = 0;
volatile bool cached_port_function_register_da0 = 0;
volatile bool cached_port_function_register_dior = 0;
volatile bool cached_port_function_register_diow = 0;
volatile bool cached_port_function_register_iordy = 0;
volatile bool cached_port_function_register_intrq = 0;
volatile bool cached_port_function_register_dmarq = 0;
volatile bool cached_port_function_register_dmack = 0; 

bool databus_selected_register_is_valid;
uint8_t* databus_selected_register;
uint8_t databus_selected_register_index;

typedef enum SEGA_DATABUS_STATE {
    SEGA_DATABUS_STATE_IDLE = 0,
    SEGA_DATABUS_STATE_READ,
    SEGA_DATABUS_STATE_WRITE,
} SEGA_DATABUS_STATE;
uint8_t databus_state = SEGA_DATABUS_STATE_IDLE;

// Used to transform data from MCU2 into the various control lines
void sega_databus_extract_raw_control_line_packet(uint8_t rawData, bool rd, bool wr) {
    // MCU2 has 16 lines, Easier to sample them all at this point
    /* LSB ... MSB
     * a0, a1, a2, cs0, cs1, read, write, iordy, 
     * intrq, x, x, x, dmarq, dmack, x, x
     */

    // This might take too long cycle count wise... This data will take time before it gets here
    // plus this will take another ~33 cycles worth of processing.

     cached_port_function_register_da0  =  (rawData >> MCU2_PIN_A0) & 0x1;
     cached_port_function_register_da1  =  (rawData >> MCU2_PIN_A1) & 0x1;
     cached_port_function_register_da2  =  (rawData >> MCU2_PIN_A2) & 0x1;
     cached_port_function_register_cs0  =  (rawData >> MCU2_PIN_IDE_CS0) & 0x1;
     cached_port_function_register_cs1  =  (rawData >> MCU2_PIN_IDE_CS1) & 0x1;
     cached_port_function_register_dior = rd;
     cached_port_function_register_diow = wr;
    //  cached_port_function_register_dior =  (rawData >> MCU2_PIN_READ) & 0x1;
    //  cached_port_function_register_diow =  (rawData >> MCU2_PIN_WRITE) & 0x1;
    //  cached_port_function_register_iordy = (rawData >> MCU2_PIN_IORDY) & 0x1;
    //  cached_port_function_register_intrq = (rawData >> MCU2_PIN_INTRQ) & 0x1;
    //  cached_port_function_register_dmarq = (rawData >> MCU2_PIN_DMARQ) & 0x1;
    //  cached_port_function_register_dmack = (rawData >> MCU2_PIN_DMACK) & 0x1;
}

void sega_databus_process_control_line_data() {
    // Get the SPI_REGISTER "name" and actual register.
    databus_selected_register_index = SPI_REGISTER_COUNT; // Will contain the "name" of the register
    databus_selected_register_is_valid = SPI_select_register(
        cached_port_function_register_cs0,
        cached_port_function_register_cs1,
        cached_port_function_register_da2,
        cached_port_function_register_da1,
        cached_port_function_register_da0,
        cached_port_function_register_dior,
        cached_port_function_register_diow,
        databus_selected_register,
        &databus_selected_register_index
    );
}