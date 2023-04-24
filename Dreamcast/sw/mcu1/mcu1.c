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
 * Using 8line mux 2:1 (Common A, ControlLine B, Data C)
 * (A) gpio0-?
 * (B) A0, A1, A2, CS0, CS1 ()
 * (C) D0-D?
 *
 * 6 gpio SD Card
 * 16 Databus/muxed
 * 1 Mux select
 * ===== 23 pins
 * 5: rd, wr, intrq, dmack, dmarq
 * ===== 28 pins
 * X(whatever is left) for comms to mcu2
 *
 * These need to be available and not muxed?
 * rd
 * wr
 * intrq
 * dmack (host drives signal when signalling for more dma data)
 * dmarq (device drives signal when dma available)
 *
 * What sets of pins are used at the same time?
 * (20) d0-d15, marq, mack, intrq(?), rd, wr
 * (5)  a0-a2, cs0, cs2
 */

/*
 * Once MCU1 has recieved info from MCU2 (control line update)
 * Process it and act if needed
 */
static void process_mcu2_data_and_exec_SPI_cmd_if_needed(bool rd, bool wr) {
    // Grab the state of the pins
    uint32_t current_databus_value = gpio_get_all(); /// TODO mask this data for the actual data bus pins
    // bool rd = current_databus_value & 0x10000000; // pin 28
    // bool wr = current_databus_value & 0x20000000; // pin 29

    uint8_t controlValues = mcu1_received_control_line_buffer.buf[mcu1_received_control_line_buffer.head-1];

    sega_databus_extract_raw_control_line_packet(controlValues, rd, wr);

    // at least 22 cycles coming to this line
    sega_databus_process_control_line_data(); // worst case another at least 24 cycles here

    //

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

    setup_sega_pio_programs();

    // init the data pins
    for (int i = 0; i < 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, false); // set to input
    }

    gpio_init(MCU1_PIN_MUX_SELECT);
    gpio_set_dir(MCU1_PIN_MUX_SELECT, true);

    // MUX to control lines
    gpio_put(MCU1_PIN_MUX_SELECT, true);

    bool last_rd = 0;
    bool last_wr = 0;
    bool rd = 0;
    bool wr = 0;
    int numReadValues = 0;
    bool csIsReady = false;
    while(1) {

        uint32_t pins = gpio_get_all();

        // If CS1 line is low
        if ((pins & 0x1) == false) {
            gpio_put(MCU1_PIN_MUX_SELECT, false); // switch to data lines
        } else {
            gpio_put(MCU1_PIN_MUX_SELECT, true); // control lines
        }

        // rd = pins & 0x10000000; // pin 28
        // wr = pins & 0x20000000; // pin 29

        // if (rd == 0 && last_rd == 1) {
        //     process_dreamlink_buffer();
        // } else if (wr == 0 && last_wr == 1) {
        //     process_dreamlink_buffer();
        // }

        // last_rd = rd;
        // last_wr = wr;



        // // Wait for the rd or wr lines to go low
        // // THEN process buffer
        // // THEN process cmd
        // do {
        //     rd = gpio_get(MCU1_PIN_READ);
        //     wr = gpio_get(MCU1_PIN_WRITE);
        //     last_rd = rd;
        //     last_wr = wr;
        // } while (rd == 1 && wr == 1);

        // process_dreamlink_buffer();

        // if (mcu1_control_line_data_ready) {
        //     mcu1_control_line_data_ready = false;

        //     // Pass in the last values for the pins
        //     process_mcu2_data_and_exec_SPI_cmd_if_needed(rd, wr);
        // }
    }

    return 0;
}
