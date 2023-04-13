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
    for(int i = 0; i < 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, false);
    }
}

uint16_t cached_control_line_data = 0;

int main(void) {
    stdio_init_all();
    current_mcu = MCU2;

    setup_gpio();

    sleep_ms(1500);

    printf("MCU2- Setting up interconnect\n");
    interconnect_init(MCU2_PIN_PIO_COMMS_CTRL1, MCU2_PIN_PIO_COMMS_CTRL2, MCU2_PIN_PIO_COMMS_D0, true);
    
    uint16_t lastSampled = 0;
    bool didProcessBuffer = false;
    volatile uint32_t lineChangeCount = 0;
    volatile uint32_t startTime = 0;
    while(1) {
        tight_loop_contents();

        if (interconnect_rx_buffer_has_data()) {
            process_dreamlink_buffer();
            didProcessBuffer = true;
        }

        if (mcu2_fetch_control_lines) {
            mcu2_fetch_control_lines = false;

            // fetch control line data
            uint16_t values = (uint16_t)(gpio_get_all() & MCU2_CONTROL_LINE_PIN_MASK);
            
            // Only really care about address and function select, not dma or iordy/intrq lines since those 
            // are supposed to be set BY us.
            // Debounce the values in case we are sampling faster than is being sent
            if (values & 0x7F != cached_control_line_data & 0x7F) {
                // if (lineChangeCount == 0) {
                //     startTime = time_us_32();
                // }
                cached_control_line_data = values;

                // send control line data
                dreamlink_send_control_line_data_cmd(values);
                // lineChangeCount++;
            }
        }

        // Hack to constantly process the control pins and send the data to mcu1...
        if (didProcessBuffer) {
            didProcessBuffer = false;
        } else {
            mcu2_fetch_control_lines = true;
        }

        // if (lineChangeCount >= 1000) {
        //     uint32_t diff = time_us_32() - startTime;
        //     printf("%u: %uus\n", lineChangeCount, diff);
        //     startTime = 0;
        //     lineChangeCount = 0;
        // }
    }
}
