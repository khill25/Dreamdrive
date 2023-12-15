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

uint cs0_rd_program_offset;
uint cs1_wr_program_offset;
uint csx_start_offset = 0;
uint rw_start_offset = 3;
bool isCheckingCS = true; // Toggle to track if programs are processing CSx or RD/WR
uint sega_databus_handler_sm = 0;

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

void init_sega_databus_handler_program(sega_pio_program_t* sega_pio_program);
void init_sega_combined_cs0_read_detect_program(sega_pio_program_t* sega_pio_program);
void init_sega_combined_cs1_write_detect_program(sega_pio_program_t* sega_pio_program);
void init_sega_combined_cs_rw_detect_program(sega_pio_program_t* sega_pio_program);

const int sega_pio_program_count = 3;
sega_pio_program_t sega_pio_programs[4] = {
	// Bus program
	{ .pio=pio1, .sm=0, .program=&sega_databus_handler_program,	.funcPointer_default_config=sega_databus_handler_program_get_default_config,	.funcPointer_init=init_sega_databus_handler_program },

	// // COMBINED PROGRAMS
	{ .pio=pio1, .sm=1, .program=&sega_cs0_read_detect_program,		.funcPointer_default_config=sega_cs0_read_detect_program_get_default_config,	.funcPointer_init=init_sega_combined_cs0_read_detect_program},
	{ .pio=pio1, .sm=2, .program=&sega_cs1_write_detect_program,	.funcPointer_default_config=sega_cs1_write_detect_program_get_default_config,	.funcPointer_init=init_sega_combined_cs1_write_detect_program},

	//sega_cs_rw_detect
	// { .pio=pio1, .sm=1, .program=&sega_cs_rw_detect_program,		.funcPointer_default_config=sega_cs_rw_detect_program_get_default_config,	.funcPointer_init=init_sega_combined_cs_rw_detect_program},
};

void setup_sega_pio_programs() {
	printf("Setting up databus pio programs...\n");

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

	// Load BUS, and combined cs, r/w program
	for (int i = 0; i < sega_pio_program_count; i++) {
		pio_sm_init(sega_pio_programs[i].pio, sega_pio_programs[i].sm, sega_pio_programs[i].offset, &sega_pio_programs[i].config);
	}

	printf("DONE!\n");

	// These are the two functions that need to be called in order to start a program
	//pio_sm_init(pio, sm, offset, &c);
	//pio_sm_set_enabled(pio, sm, true);
}

void start_sega_pio_programs() {
	for (int i = 0; i < sega_pio_program_count; i++) {
		pio_sm_set_enabled(sega_pio_programs[i].pio, sega_pio_programs[i].sm, true);
	}
}

// This should now be significatly faster not swapping programs in and out and just JMP'ing to new parts of code
void swap_cs_detect_for_rw_detect() {
	// If we are in the CS portion of the program, swap to rd or wr part
	if (isCheckingCS) {
		pio_sm_exec(pio1, 1, pio_encode_jmp(cs0_rd_program_offset + rw_start_offset));
		pio_sm_exec(pio1, 2, pio_encode_jmp(cs1_wr_program_offset + rw_start_offset));
	} else {
		pio_sm_exec(pio1, 1, pio_encode_jmp(cs0_rd_program_offset + csx_start_offset));
		pio_sm_exec(pio1, 2, pio_encode_jmp(cs1_wr_program_offset + csx_start_offset));
	}

	isCheckingCS = !isCheckingCS;
}

void init_sega_combined_cs0_read_detect_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	(*sega_pio_program).csxOffset = 0;
	(*sega_pio_program).rwOffset = 3;
}

void init_sega_combined_cs1_write_detect_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	(*sega_pio_program).csxOffset = 0;
	(*sega_pio_program).rwOffset = 3;
}

void init_sega_combined_cs_rw_detect_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	// We want to be able to read in cs0, cs1, read, write
	// Pins 3,4 and 16,17
	// pio program will throw away extra bits when checking values
	// for(int i = 3; i < 18; i++) {
	// 	pio_gpio_init(pio, i);
	// }
	pio_sm_set_consecutive_pindirs(pio, sm, 3, 15, false); // cs lines (plus 11 data lines separating the two chunks of pins) + read + write = 15
	sm_config_set_in_pins(&c, 3);
	sm_config_set_in_shift(&c, false, false, 32);
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
