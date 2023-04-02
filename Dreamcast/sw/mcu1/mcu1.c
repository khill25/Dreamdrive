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

int main(void) {
    stdio_init_all();
    current_mcu = MCU1;

    sleep_ms(1000);
    printf("MCU1- Init interconnect\n");

    gpio_init(26);
    gpio_set_dir(26, false);

    interconnect_init(MCU1_PIN_PIO_COMMS_CTRL1, MCU1_PIN_PIO_COMMS_CTRL2, MCU1_PIN_PIO_COMMS_D0, false);
    
    int numReadValues = 0;
    while(1) {

        if(interconnect_rx_buffer_has_data()) {
            if (numReadValues % 16 == 0) { printf("\n"); }

            uint16_t halfWord = interconnect_rx_get();
            printf("%04x ", halfWord);
            
            numReadValues++;
        }
    }

    return 0;
}
