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

int main(void) {
    stdio_init_all();
    current_mcu = MCU2;

    sleep_ms(1500);
    printf("MCU2- Setting up interconnect\n");

    interconnect_init(MCU2_PIN_PIO_COMMS_CTRL1, MCU2_PIN_PIO_COMMS_CTRL2, MCU2_PIN_PIO_COMMS_D0, true);

    uint16_t numSentValues = 0;
    uint16_t halfWordToSend = 0;
    while(1) {
        interconnect_tx((uint8_t*)(&halfWordToSend), 2);   
        printf("Sent %04x\n", halfWordToSend);

        numSentValues++;
        halfWordToSend = numSentValues;

        if (numSentValues % 16 == 0) {
            sleep_ms(500);
        } else {
            sleep_ms(10);
        }
    }
}
