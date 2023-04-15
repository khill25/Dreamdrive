/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "mcu2_pins.h"
#include "shared.h"
#include "serial_bridge/serial_bridge.h"
#include "hardware/pio.h"

#include "sega_packet_interface.h"

#define CONTROL_PIN_MASK 0x0000FFFF
volatile uint32_t last_ctrl_pin_sample = 0x0;

// Test to sample the control pins 0-15 and send them to mcu1
void test_sample_pins() {
    /// Should we sample the pins every loop cycle and compare to last sample?
    /// Send if there is a change?
    uint32_t pins = gpio_get_all() & CONTROL_PIN_MASK;

    // Check if the pins have changed
    if (last_ctrl_pin_sample != pins) {
        // Send data to mcu1
        interconnect_tx16(pins);
    }

    last_ctrl_pin_sample = pins;
}

// Setup the gpio pins connected to the ide control lines, we want input for all of them?
// TODO some pins might be input/output
void setup_gpio() {
    // for(int i = 0; i < 16; i++) {
    //     gpio_init(i);
    //     gpio_set_dir(i, false);
    // }

    gpio_init(MCU2_PIN_A0);
    gpio_set_dir(MCU2_PIN_A0, false);

    gpio_init(MCU2_PIN_A1);
    gpio_set_dir(MCU2_PIN_A1, false);

    gpio_init(MCU2_PIN_A2);
    gpio_set_dir(MCU2_PIN_A2, false);

    gpio_init(MCU2_PIN_IDE_CS0);
    gpio_set_dir(MCU2_PIN_IDE_CS0, false);

    gpio_init(MCU2_PIN_IDE_CS1);
    gpio_set_dir(MCU2_PIN_IDE_CS1, false);

    // read and write pins
    gpio_init(5);
    gpio_set_dir(MCU2_PIN_IDE_CS1, false);

    gpio_init(6);
    gpio_set_dir(MCU2_PIN_IDE_CS1, false);
}

uint8_t cached_control_line_data = 0;

int main(void) {
    stdio_init_all();
    current_mcu = MCU2;

    setup_gpio();

    sleep_ms(1500);

    printf("MCU2- Setting up interconnect\n");
    interconnect_init(MCU2_PIN_PIO_COMMS_CTRL1, MCU2_PIN_PIO_COMMS_CTRL2, MCU2_PIN_PIO_COMMS_D0, true);
    
    const int freq_khz = 266000;
    bool clockWasSet = set_sys_clock_khz(freq_khz, false);
    printf("Clock of %uMhz was set: %u\n", freq_khz / 1000, clockWasSet);

    uint8_t lastSampled = 0;
    bool didProcessBuffer = false;
    volatile uint32_t lineChangeCount = 0;
    volatile uint32_t startTime = 0;
    // volatile uint8_t buf[] = {0};
    // volatile uint8_t v = 0;
    bool last_rd = 0;
    bool last_wr = 0;
    bool rd = 0;
    bool wr = 0;
    volatile uint32_t c = 0;
    while(1) {

        // interconnect_tx(buf, 1);
        // buf[0] = ++v;
        // sleep_ms(1000);

        tight_loop_contents();

        // if (interconnect_rx_buffer_has_data()) {
        //     process_dreamlink_buffer();
        //     didProcessBuffer = true;
        // }

        // if (mcu2_fetch_control_lines) {
        //     mcu2_fetch_control_lines = false;

            // fetch control line data
            uint8_t values = (uint8_t)(gpio_get_all() & MCU2_REGISTER_AND_RD_WR_SELECT_PIN_MASK);

            // Also connected to mcu1 but still connected to prototype board's mcu2
            rd = values & 0x20; // pin 5
            wr = values & 0x40; // pin 6

            // Skip if both cs lines are high
            // OR both read and write are low
            if ((values & 0x18) != 0x18 && (values & 0x18) != 0x0) {
            // READ
            //     if (rd == 0 && last_rd == 1) {

            //         volatile bool da0  =  (values >> MCU2_PIN_A0) & 0x1;
            //         volatile bool da1  =  (values >> MCU2_PIN_A1) & 0x1;
            //         volatile bool da2  =  (values >> MCU2_PIN_A2) & 0x1;
            //         volatile bool cs0  =  (values >> MCU2_PIN_IDE_CS0) & 0x1;
            //         volatile bool cs1  =  (values >> MCU2_PIN_IDE_CS1) & 0x1;
            //         volatile uint8_t selected_register_index = 14;
            //         volatile uint8_t selected_register = 0;
            //         // Decode with flipped cs values
            //         SPI_select_register(cs1, cs0, da2, da1, da0, rd, wr, &selected_register, &selected_register_index);
            //         printf("r-0x%02x(%u) ", selected_register, selected_register_index);

            //         // printf("r-%02x ", values & MCU2_REGISTER_SELECT_PIN_MASK);
            //         // SPI_select_register(cs1, cs0, da2, da1, da0, rd, wr, &selected_register, &selected_register_index);
            //         // printf("!r-%02x(%02x) ", selected_register, selected_register_index);

            // // WRITE
            //     } else 
                if (wr == 0 && last_wr == 1) {
                    volatile bool da0  =  (values >> MCU2_PIN_A0) & 0x1;
                    volatile bool da1  =  (values >> MCU2_PIN_A1) & 0x1;
                    volatile bool da2  =  (values >> MCU2_PIN_A2) & 0x1;
                    volatile bool cs0  =  (values >> MCU2_PIN_IDE_CS0) & 0x1;
                    volatile bool cs1  =  (values >> MCU2_PIN_IDE_CS1) & 0x1;
                    volatile uint8_t selected_register_index = 14;
                    volatile uint8_t selected_register = 0;
                    // Decode with flipped cs values
                    SPI_select_register(cs1, cs0, da2, da1, da0, rd, wr, &selected_register, &selected_register_index);
                    printf("w-0x%02x(%u) ", selected_register, selected_register_index);

                    // printf("w-%02x ", values & MCU2_REGISTER_SELECT_PIN_MASK);
                    // SPI_select_register(cs1, cs0, da2, da1, da0, rd, wr, &selected_register, &selected_register_index);
                    // printf("!w-%02x(%02x) ", selected_register, selected_register_index);
                }

                c++;
                if (c > 128) {
                    printf("\n");
                    c = 0;
                }

                last_rd = rd;
                last_wr = wr;
            }
            // // If both cs1, cs0 are high, this is not a valid address
            // // If cs1 and cs0 are both low, not valid... data bus high imped?
            // if ((values & 0x18) == 0x18 || (values & 0x18) == 0x0) {
            //     continue;
            // }
            
            // // Debounce the values in case we are sampling faster than is being sent
            // // Also dont send if both cs lines are high
            // if (values != cached_control_line_data) {
            //     // if (lineChangeCount == 0) {
            //     //     startTime = time_us_32();
            //     // }
            //     cached_control_line_data = values;

            //     // send control line data
            //     dreamlink_send_control_line_data_cmd(values);
            //     // lineChangeCount++;
            // }
        // }

        // // Hack to constantly process the control pins and send the data to mcu1...
        // if (didProcessBuffer) {
        //     didProcessBuffer = false;
        // } else {
        //     mcu2_fetch_control_lines = true;
        // }

        // if (lineChangeCount >= 1000) {
        //     uint32_t diff = time_us_32() - startTime;
        //     printf("%u: %uus\n", lineChangeCount, diff);
        //     startTime = 0;
        //     lineChangeCount = 0;
        // }
    }
}
