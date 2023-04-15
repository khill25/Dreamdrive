/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "mcu1_pins.h"
#include "shared.h"
#include "serial_bridge/serial_bridge.h"
#include "hardware/pio.h"
#include "sega_databus/sega_databus.h"

#include "ff.h" /* Obtains integer types */
#include "diskio.h" /* Declarations of disk functions */
#include "f_util.h"

#include "sega_packet_interface.h"

/*
 * Once MCU1 has recieved info from MCU2 (control line update)
 * Process it and act if needed
 */
static void process_mcu2_data_and_exec_SPI_cmd_if_needed() {
    // Grab the state of the pins
    uint16_t current_databus_value = gpio_get_all(); /// TODO mask this data for the actual data bus pins
    bool rd = current_databus_value & 0x10000000; // pin 28
    bool wr = current_databus_value & 0x20000000; // pin 29

    sega_databus_extract_raw_control_line_packet(mcu1_received_control_line_data, rd, wr);
    sega_databus_process_control_line_data();
    // At this point we have the register and register "name" (index)

    printf("data:%04x, index:%02x, rd:%u, wr:%u\n", databus_selected_register, databus_selected_register_index, rd, wr);

    if (databus_selected_register_index == SPI_REGISTER_COUNT) {
        return; // nothing to do here, invalid state
    }

    // Need to put sampled data into relevant registers
    
    // Once the register is the COMMAND_REGISTER, we can start processing as we will have all the data 
    // set in the appropriate registers
    if(databus_selected_register_index == SPI_COMMAND_REGISTER_INDEX) {

    }

    // In most cases BSY register must be set within 400ns of the host sending this data
    // and we have likely spent most of that shuffling the data from mcu2 and then decoding it.
            
}
/*
May need to do something faster than waiting for mcu2 to send a packet every time it samples pins

MCU2 captures all the signal lines state changes
    * asserts one of the MCU control lines
    * MCU1 interrupts on that control line
    * That means to sample the pins
    * Store that result in a buffer
    * Once the control lines specify it's COMMAND_REGISTER time, MCU2 asserts control line like usual
        * but then sends all the data it received (so all the control line samples)
        * and MCU1 can line up the control lines with it's sampled (parallel) buffer

BSY must be set within 400ns of the command register being written    
 */

void sdcard_read_test() {

}

int main(void) {
    stdio_init_all();
    current_mcu = MCU1;

    sleep_ms(1000);
    printf("MCU1- Init interconnect\n");

    gpio_init(26);
    gpio_set_dir(26, false);

    interconnect_init(MCU1_PIN_PIO_COMMS_CTRL1, MCU1_PIN_PIO_COMMS_CTRL2, MCU1_PIN_PIO_COMMS_D0, false);

    // init the data pins
    for (int i = 0; i < 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, false); // set to input
    }

    gpio_init(MCU1_PIN_READ);
    gpio_set_dir(MCU1_PIN_READ, false);
    gpio_init(MCU1_PIN_WRITE);
    gpio_set_dir(MCU1_PIN_WRITE, false);
    
    int numReadValues = 0;
    while(1) {
        process_dreamlink_buffer();

        if (mcu1_control_line_data_ready) {
            mcu1_control_line_data_ready = false;
            
            process_mcu2_data_and_exec_SPI_cmd_if_needed();
        }
    }

    return 0;
}
