/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "ide_databus.h"
#include "hardware/pio.h"

#include "ide_databus.pio.h"

void init_ide_databus() {
    PIO pio = pio1;
    uint sm = 0;
    uint offset = pio_add_program(pio, &ide_databus_program);
    for (int i = 0; i < 16; i++) {
        pio_gpio_init(pio, i);
    }

    // Data lines
    // 0-15, both input and output

    // Set [AD0, AD15] as input
    pio_sm_set_consecutive_pindirs(pio, sm, 0, 16, false);

    pio_sm_config c = ide_databus_program_get_default_config(offset);

    // shift_right=false, autopush=false, push_threshold=32
    sm_config_set_in_shift(&c, false, false, 32);

    // shift_right=true, autopull=false, pull_threshold=32
    sm_config_set_out_shift(&c, true, false, 32);

    // Set [AD0, AD15] as in pins
    sm_config_set_in_pins(&c, 0);

    // Set [AD0, AD15] as out pins
    sm_config_set_out_pins(&c, 0, 16);

    pio_sm_init(pio, sm, offset, &c);
}