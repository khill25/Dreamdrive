/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"

#include "interconnect_mux.pio.h"

#define CONTROL_PIN_MASK 0x0000FFFF
volatile uint32_t last_ctrl_pin_sample = 0x0;

#define IDE_DATABUS_MASK 				(0xFFFF) 	// pins 0-15

#define INTERCONNECT_DATABUS_MASK 		(0xFF0000) 	// pins 16-23
#define INTERCONNECT_BUS_DIRECTION_MASK (0x1000000) // pin 24
#define INTERCONNECT_IRQ_LOW_MASK		(0x4000000) // pin 26
#define INTERCONNECT_IRQ_HIGH_MASK 		(0x8000000) // pin 27
#define INTERCONNECT_IRQ_LOW_PIN		(26)
#define INTERCONNECT_IRQ_HIGH_PIN		(27)
#define INTERCONNECT_DIRECTION_PIN		(24)

#define INTERCONNECT_BUS_OUTPUT_LOW_SM 		(0)
#define INTERCONNECT_BUS_OUTPUT_HIGH_SM 	(1)
#define INTERCONNECT_BUS_INPUT_LOW_SM 		(2)
#define INTERCONNECT_BUS_INPUT_HIGH_SM 		(3)
#define INTERCONNECT_READ_FROM_DREAMCAST_SM (2)

#define INTERCONNECT_MCU_PIN_START 			(16)
#define INTERCONNECT_BUS_PIN_LOW_START 		(0)
#define INTERCONNECT_BUS_PIN_HIGH_START 	(8)
#define INTERCONNECT_BUS_PIN_LOW_PIN_COUNT 	(8)
#define INTERCONNECT_BUS_PIN_HIGH_PIN_COUNT (8)

// void setup_interconnect_output_low_program() {
// 	uint sm = INTERCONNECT_BUS_OUTPUT_LOW_SM;
// 	uint offset = pio_add_program(pio1, &interconnect_low_program);
// 	pio_sm_config c = interconnect_low_program_get_default_config(offset);

// 	// Input pins start at pin 16
// 	sm_config_set_in_pins(&c, INTERCONNECT_MCU_PIN_START);

// 	// Output pins are 0-15, but this is the low byte so start at pin 0
// 	sm_config_set_out_pins(&c, INTERCONNECT_BUS_PIN_LOW_START, INTERCONNECT_BUS_PIN_LOW_PIN_COUNT);
// 	pio_sm_set_pindirs_with_mask(pio1, sm, 0x00FFFF, 0xFFFFFF);

// 	pio_sm_init(pio1, sm, offset, &c);
// }

// void setup_interconnect_output_high_program() {
// 	uint sm = INTERCONNECT_BUS_OUTPUT_HIGH_SM;
// 	uint offset = pio_add_program(pio1, &interconnect_high_program);
// 	pio_sm_config c = interconnect_high_program_get_default_config(offset);

// 	// Input pins start at pin 16
// 	sm_config_set_in_pins(&c, INTERCONNECT_MCU_PIN_START);

// 	// Output pins are 0-15, but this is the high byte so start at pin 8
// 	sm_config_set_out_pins(&c, INTERCONNECT_BUS_PIN_HIGH_START, INTERCONNECT_BUS_PIN_HIGH_PIN_COUNT);
// 	pio_sm_set_pindirs_with_mask(pio1, sm, 0x00FFFF, 0xFFFFFF);

// 	pio_sm_init(pio1, sm, offset, &c);
// }

// void setup_interconnect_input_low_program() {
// 	uint sm = INTERCONNECT_BUS_INPUT_LOW_SM;
// 	uint offset = pio_add_program(pio1, &interconnect_low_program);
// 	pio_sm_config c = interconnect_low_program_get_default_config(offset);

// 	sm_config_set_in_pins(&c, INTERCONNECT_BUS_PIN_LOW_START);
// 	sm_config_set_out_pins(&c, INTERCONNECT_MCU_PIN_START, INTERCONNECT_BUS_PIN_LOW_PIN_COUNT);

// 	pio_sm_init(pio1, sm, offset, &c);
// }

// void setup_interconnect_input_high_program() {
// 	uint sm = INTERCONNECT_BUS_INPUT_HIGH_SM;
// 	uint offset = pio_add_program(pio1, &interconnect_high_program);
// 	pio_sm_config c = interconnect_high_program_get_default_config(offset);

// 	sm_config_set_in_pins(&c, INTERCONNECT_BUS_PIN_HIGH_START);
// 	sm_config_set_out_pins(&c, INTERCONNECT_MCU_PIN_START, INTERCONNECT_BUS_PIN_HIGH_PIN_COUNT);

// 	pio_sm_init(pio1, sm, offset, &c);
// }

void setup_interconnect_read_from_dreamcast() {
	uint sm = 1;
	uint offset = pio_add_program(pio1, &interconnect_read_from_dreamcast_program);
	pio_sm_config c = interconnect_read_from_dreamcast_program_get_default_config(offset);

	sm_config_set_in_pins(&c, INTERCONNECT_BUS_PIN_LOW_START);
	sm_config_set_out_pins(&c, INTERCONNECT_MCU_PIN_START, INTERCONNECT_BUS_PIN_LOW_PIN_COUNT);

	pio_sm_set_pindirs_with_mask(pio1, sm, 0xFF0000, 0xFFFFFF);

	pio_sm_init(pio1, sm, offset, &c);
}

void setup_send_to_dreamcast() {
	uint sm = 0;
	uint offset = pio_add_program(pio1, &write_to_dreamcast_program);
	pio_sm_config c = write_to_dreamcast_program_get_default_config(offset);

	// Input pins start at pin 16
	sm_config_set_in_pins(&c, INTERCONNECT_MCU_PIN_START);

	// Output pins are 0-15, but this is the low byte so start at pin 0
	sm_config_set_out_pins(&c, INTERCONNECT_BUS_PIN_LOW_START, INTERCONNECT_BUS_PIN_LOW_PIN_COUNT + INTERCONNECT_BUS_PIN_HIGH_PIN_COUNT);
	
	pio_sm_set_pindirs_with_mask(pio1, sm, 0x0000FFFF, 0x00FFFFFF);
	
	sm_config_set_in_shift(&c, false, false, 8);

	pio_sm_init(pio1, sm, offset, &c);
}

void setup_interconnect() {

	// init all the pins used by pio
	for(int i = 0; i < 24; i++) {
		pio_gpio_init(pio1, i);
	}

	// setup_interconnect_output_low_program();
	// setup_interconnect_output_high_program();
	setup_interconnect_read_from_dreamcast();
	setup_send_to_dreamcast();
	// setup_interconnect_input_low_program();
	// setup_interconnect_input_high_program();
}

// When interconnect is output(output to dreamcast), OUTPUT data from mcu TO BUS
// When interconnect is input (input from dreamcast), OUTPUT data from bus TO MCU
volatile bool read_from_dreamcast = false;

static inline void swap_direction() {
	// if (read_from_dreamcast) {
	// 	pio1->ctrl = 0x33; // read from dreamcast
	// } else {
	// 	pio1->ctrl = 0x44; // send to dreamcast
	// }
	// pio_restart_sm_mask(pio1, 0x3);

	pio_sm_set_enabled(pio1, 0, true);
	pio_sm_set_enabled(pio1, 1, true);
}

int main(void) {
	stdio_init_all();
	// current_mcu = MCU2;

	for(int i = 16; i < 24; i++) {
		gpio_init(i);
		gpio_set_dir(i, GPIO_IN);
	}

	const int freq_khz = 266000;
	// const int freq_khz = 336000;
	// vreg_set_voltage(VREG_VOLTAGE_1_25);
	bool clockWasSet = set_sys_clock_khz(freq_khz, false);

	// Setup control line pins to be input
	gpio_init(INTERCONNECT_DIRECTION_PIN);
	gpio_set_dir(INTERCONNECT_DIRECTION_PIN, GPIO_IN);

	gpio_init(INTERCONNECT_IRQ_LOW_PIN);
	gpio_set_dir(INTERCONNECT_IRQ_LOW_PIN, GPIO_IN);

	gpio_init(INTERCONNECT_IRQ_HIGH_PIN);
	gpio_set_dir(INTERCONNECT_IRQ_HIGH_PIN, GPIO_IN);

	setup_interconnect();
	volatile bool current_direction = read_from_dreamcast;
	swap_direction();

	while(1) {
		tight_loop_contents();
		// current_direction = gpio_get(INTERCONNECT_DIRECTION_PIN);
		// if (current_direction != read_from_dreamcast) {
		// 	read_from_dreamcast = current_direction;
		// 	swap_direction();
		// }

		if(gpio_get(27)) {
			printf("%x, %x\n", pio_sm_get_blocking(pio1, 1), sio_hw->gpio_in);
			// printf("%x, %x\n", sio_hw->gpio_in, sio_hw->gpio_in >> 16);
			// while(gpio_get(INTERCONNECT_IRQ_LOW_PIN)){ tight_loop_contents(); }
		}
	}
}
