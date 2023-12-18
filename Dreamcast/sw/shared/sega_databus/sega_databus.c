/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "sega_databus.h"
#include "sega_packet_interface.h"
#include "mcu1_pins.h"

#include "sega_databus.pio.h"

#include "hardware/structs/systick.h"
#include <stdio.h>

uint sega_cs0_low_sm 			= 0;
uint sega_cs1_low_sm			= 1;
uint sega_bus_read_request_sm	= 2;
uint sega_bus_write_request_sm	= 3;

typedef enum SEGA_DATABUS_STATE {
    SEGA_DATABUS_STATE_IDLE = 0,
    SEGA_DATABUS_STATE_READ,
    SEGA_DATABUS_STATE_WRITE,
} SEGA_DATABUS_STATE;
uint8_t databus_state = SEGA_DATABUS_STATE_IDLE;

typedef struct sega_pio_program_t sega_pio_program_t;

// The detect pio programs are using `wait x GPIO y` format rather than a `pin` value as it's faster.
struct sega_pio_program_t {
	PIO pio;
	uint sm;
	uint offset;
	const pio_program_t* program;
	pio_sm_config (*funcPointer_default_config)(uint offset);
	void (*funcPointer_init)(sega_pio_program_t* sega_pio_program);
	pio_sm_config config;
	bool isRunning;
	bool isLoaded;
	// For easy jmp encoding to move around in the pio program
	uint csxOffset; // the value to start the cs detect at (inclues the loaded program offset)
	uint rwOffset; // the vaue to start read or write detect at (inclues the loaded program offset)
};

void init_cs0_low_program(sega_pio_program_t* sega_pio_program);
void init_cs1_low_program(sega_pio_program_t* sega_pio_program);
void init_bus_read_request_program(sega_pio_program_t* sega_pio_program);
void init_bus_write_request_program(sega_pio_program_t* sega_pio_program);

const int sega_pio_program_count = 4;
sega_pio_program_t sega_pio_programs[4] = {
	{ .pio=pio1, .sm=0, .program=&cs0_low_program,			.funcPointer_default_config=cs0_low_program_get_default_config,				.funcPointer_init=init_cs0_low_program },
	{ .pio=pio1, .sm=1, .program=&cs1_low_program,			.funcPointer_default_config=cs1_low_program_get_default_config,				.funcPointer_init=init_cs1_low_program },
	{ .pio=pio1, .sm=2, .program=&bus_read_request_program,	.funcPointer_default_config=bus_read_request_program_get_default_config,	.funcPointer_init=init_bus_read_request_program },
	{ .pio=pio1, .sm=3, .program=&bus_write_request_program,.funcPointer_default_config=bus_write_request_program_get_default_config,	.funcPointer_init=init_bus_write_request_program },
};

void setup_sega_pio_programs() {
	printf("Setting up databus pio programs...\n");

	// Init all the pins used for control/data (0-15)
	for(int i = 0; i < 16; i++) {
		pio_gpio_init(pio1, i);
	}

	// Add and load all programs
	// load sega_pio_program_count programs
	for(uint i = 0; i < sega_pio_program_count; i++) {
		printf("Loading program %u...", i);
		PIO pio = sega_pio_programs[i].pio;
		const pio_program_t* program = sega_pio_programs[i].program;
		uint offset = pio_add_program(pio, program);
		pio_sm_config config = sega_pio_programs[i].funcPointer_default_config(offset);

		sega_pio_programs[i].config = config;
		sega_pio_programs[i].offset = offset;
		sega_pio_programs[i].isLoaded = true;

		// Run init program to setup to setup pins and such
		sega_pio_programs[i].funcPointer_init(&sega_pio_programs[i]);
		printf("done!\n");
	}

	printf("Loading BUS programs...");

	for (int i = 0; i < sega_pio_program_count; i++) {
		pio_sm_init(sega_pio_programs[i].pio, sega_pio_programs[i].sm, sega_pio_programs[i].offset, &sega_pio_programs[i].config);
	}

	printf("DONE!\n");
}

void start_sega_pio_programs() {
	for (int i = 0; i < sega_pio_program_count; i++) {
		pio_sm_set_enabled(sega_pio_programs[i].pio, sega_pio_programs[i].sm, true);
	}
}

void init_sega_databus_handler_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	// Init all the pins used for control/data (0-7)
	// TODO might need to be pins 0-16 for 16 bit data?
	// pin 16 is read
	// pin 17 is write
	for(int i = 0; i < 18; i++) {
		pio_gpio_init(pio, i);
	}
	pio_sm_set_consecutive_pindirs(pio, sm, 0, 18, false); // 16 data pins + read + write = 18
	sm_config_set_in_pins(&c, 0);

	// Think this is going to be needed now that we are doing the cs/r/w/ pin checks here now
	sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_out_shift(&c, true, false, 32);

	// Set OUT pins the same as IN since they are bidirectional
	sm_config_set_out_pins(&c, 0, 16);

	// Setup JMP pin on the READ pin, gpio 16
	pio_gpio_init(pio, 16);
	sm_config_set_jmp_pin(&c, 16);

	// MUX toggle pin (gpio 27 on MUC1)
	// between data bus and control lines
	pio_gpio_init(pio, 27);
	sm_config_set_sideset_pins(&c, 27);

	// DONT START HERE
	// pio_sm_init(pio, sm, offset, &c);
	// pio_sm_set_enabled(pio, sm, true);
}

void init_cs0_low_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	printf("SM: %d\n", sm);

	pio_sm_set_consecutive_pindirs(pio, sm, 0, 16, false);
	sm_config_set_in_pins(&c, 0);

	sm_config_set_in_shift(&c, false, false, 32);
	
	// Setup JMP pin on the MUX pin, we use it as a mutex to proceed with grabbing values
	pio_gpio_init(pio, 27);
	sm_config_set_jmp_pin(&c, 27);

	// MUX toggle pin (gpio 27 on MUC1)
	// between data bus and control lines
	pio_gpio_init(pio, 27);
	sm_config_set_sideset_pins(&c, 27);
}

void init_cs1_low_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	printf("SM: %d\n", sm);

	pio_sm_set_consecutive_pindirs(pio, sm, 0, 16, false);
	sm_config_set_in_pins(&c, 0);

	sm_config_set_in_shift(&c, false, false, 32);
	
	// Setup JMP pin on the MUX pin, we use it as a mutex to proceed with grabbing values
	pio_gpio_init(pio, 27);
	sm_config_set_jmp_pin(&c, 27);

	// MUX toggle pin (gpio 27 on MUC1)
	// between data bus and control lines
	pio_gpio_init(pio, 27);
	sm_config_set_sideset_pins(&c, 27);
}

void init_bus_read_request_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	printf("SM: %d\n", sm);

	pio_sm_set_consecutive_pindirs(pio, sm, 0, 16, true);
	sm_config_set_out_pins(&c, 0, 16);

	sm_config_set_out_shift(&c, false, false, 32);

	// MUX toggle pin (gpio 27 on MUC1)
	// between data bus and control lines
	pio_gpio_init(pio, 27);
	sm_config_set_sideset_pins(&c, 27);
	
}

void init_bus_write_request_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	printf("SM: %d\n", sm);

	pio_sm_set_consecutive_pindirs(pio, sm, 0, 16, false);
	sm_config_set_in_pins(&c, 0);

	sm_config_set_in_shift(&c, false, false, 32);

	// MUX toggle pin (gpio 27 on MUC1)
	// between data bus and control lines
	pio_gpio_init(pio, 27);
	sm_config_set_sideset_pins(&c, 27);
}
