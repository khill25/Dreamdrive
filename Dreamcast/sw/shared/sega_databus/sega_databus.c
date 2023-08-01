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

volatile uint16_t cached_control_line_register = 0;

volatile bool cached_port_function_register_cs0 = 0;
volatile bool cached_port_function_register_cs1 = 0;
volatile bool cached_port_function_register_da2 = 0;
volatile bool cached_port_function_register_da1 = 0;
volatile bool cached_port_function_register_da0 = 0;
volatile bool cached_port_function_register_dior = 0;
volatile bool cached_port_function_register_diow = 0;
volatile bool cached_port_function_register_iordy = 0;
volatile bool cached_port_function_register_intrq = 0;
volatile bool cached_port_function_register_dmarq = 0;
volatile bool cached_port_function_register_dmack = 0;

bool databus_selected_register_is_valid;
uint8_t* databus_selected_register;
uint8_t databus_selected_register_index;

typedef enum SEGA_DATABUS_STATE {
    SEGA_DATABUS_STATE_IDLE = 0,
    SEGA_DATABUS_STATE_READ,
    SEGA_DATABUS_STATE_WRITE,
} SEGA_DATABUS_STATE;
uint8_t databus_state = SEGA_DATABUS_STATE_IDLE;

// Used to transform data from MCU2 into the various control lines
void sega_databus_extract_raw_control_line_packet(uint8_t rawData, bool rd, bool wr) {
    // MCU2 has 16 lines, Easier to sample them all at this point
    /* LSB ... MSB
     * a0, a1, a2, cs0, cs1, read, write, iordy,
     * intrq, x, x, x, dmarq, dmack, x, x
     */

    // This might take too long cycle count wise...
    // This will take ~33 cycles worth of processing.

	// Table from gdrom docs CS lines use "A" and "N" which in our case are "0" and "1"
	// Flipping the CS line order when computing the value will give the right result

     cached_port_function_register_da0  =  (rawData >> MCU1_PIN_A0) & 0x1;
     cached_port_function_register_da1  =  (rawData >> MCU1_PIN_A1) & 0x1;
     cached_port_function_register_da2  =  (rawData >> MCU1_PIN_A2) & 0x1;
     cached_port_function_register_cs0  =  (rawData >> MCU1_PIN_CS0) & 0x1;
     cached_port_function_register_cs1  =  (rawData >> MCU1_PIN_CS1) & 0x1;
     cached_port_function_register_dior = rd;
     cached_port_function_register_diow = wr;
    //  cached_port_function_register_dior =  (rawData >> MCU2_PIN_READ) & 0x1;
    //  cached_port_function_register_diow =  (rawData >> MCU2_PIN_WRITE) & 0x1;
    //  cached_port_function_register_iordy = (rawData >> MCU2_PIN_IORDY) & 0x1;
    //  cached_port_function_register_intrq = (rawData >> MCU2_PIN_INTRQ) & 0x1;
    //  cached_port_function_register_dmarq = (rawData >> MCU2_PIN_DMARQ) & 0x1;
    //  cached_port_function_register_dmack = (rawData >> MCU2_PIN_DMACK) & 0x1;
}

void sega_databus_process_control_line_data() {
    // Get the SPI_REGISTER "name" and actual register.
    databus_selected_register_index = SPI_REGISTER_COUNT; // Will contain the "name" of the register
    databus_selected_register_is_valid = SPI_select_register(
        cached_port_function_register_cs0,
        cached_port_function_register_cs1,
        cached_port_function_register_da2,
        cached_port_function_register_da1,
        cached_port_function_register_da0,
        cached_port_function_register_dior,
        cached_port_function_register_diow,
        databus_selected_register,
        &databus_selected_register_index
    );
}

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

void init_test_irq_detect_program(sega_pio_program_t* sega_pio_program); // TODO this needs to be renamed (all the way down the PIO program name)
void init_sega_cs0_detect_program(sega_pio_program_t* sega_pio_program);
void init_sega_cs1_detect_program(sega_pio_program_t* sega_pio_program);
void init_sega_read_detect_program(sega_pio_program_t* sega_pio_program);
void init_sega_write_detect_program(sega_pio_program_t* sega_pio_program);
// Combined program forward declares
void init_sega_combined_cs0_read_detect_program(sega_pio_program_t* sega_pio_program);
void init_sega_combined_cs1_write_detect_program(sega_pio_program_t* sega_pio_program);

sega_pio_program_t sega_pio_programs[7] = {
	// Bus program
	{ .pio=pio1, .sm=0, .program=&test_irq_set_gpio_program,	.funcPointer_default_config=test_irq_set_gpio_program_get_default_config,	.funcPointer_init=init_test_irq_detect_program },

	// CS lines
	{ .pio=pio1, .sm=1, .program=&sega_cs0_detect_program,		.funcPointer_default_config=sega_cs0_detect_program_get_default_config,		.funcPointer_init=init_sega_cs0_detect_program},
	{ .pio=pio1, .sm=2, .program=&sega_cs1_detect_program,		.funcPointer_default_config=sega_cs1_detect_program_get_default_config,		.funcPointer_init=init_sega_cs1_detect_program},

	// READ and WRITE lines
	{ .pio=pio1, .sm=1, .program=&sega_read_detect_program,		.funcPointer_default_config=sega_read_detect_program_get_default_config,	.funcPointer_init=init_sega_read_detect_program}, // Use SM1 as this will be swapped out with cs0 detect program
	{ .pio=pio1, .sm=2, .program=&sega_write_detect_program,	.funcPointer_default_config=sega_write_detect_program_get_default_config,	.funcPointer_init=init_sega_write_detect_program}, // Use SM2 as this will be swapped out with cs1 detect program

	// COMBINED PROGRAMS
	{ .pio=pio1, .sm=1, .program=&sega_cs0_read_detect_program,		.funcPointer_default_config=sega_cs0_read_detect_program_get_default_config,	.funcPointer_init=init_sega_combined_cs0_read_detect_program},
	{ .pio=pio1, .sm=2, .program=&sega_cs1_write_detect_program,	.funcPointer_default_config=sega_cs1_write_detect_program_get_default_config,	.funcPointer_init=init_sega_combined_cs1_write_detect_program},
};
bool isRunningCSDetect = true; // used when swapping CS detect for READ/WRITE detect and vice versa.

void setup_sega_pio_programs() {
	printf("Setting up databus pio programs...\n");

	// Add and load all programs
	for(uint i = 0; i < 5; i++) {
		printf("Loading program %u...", i);
		PIO pio = sega_pio_programs[i].pio;
		pio_program_t* program = sega_pio_programs[i].program;
		uint offset = pio_add_program(pio, program);
		pio_sm_config config = sega_pio_programs[i].funcPointer_default_config(offset);

		sega_pio_programs[i].config = config;
		sega_pio_programs[i].offset = offset;
		sega_pio_programs[i].isLoaded = true;

		// Run init program to setup to setup pins and such
		sega_pio_programs[i].funcPointer_init(&sega_pio_programs[i]);
		printf("done!\n");
	}

	printf("Loading BUS, CS0 Detect, CS1 Detect\n");

	// Load BUS, CS0 Detect, and CS1 Detect
	for (int i = 0; i < 3; i++) {
		pio_sm_init(sega_pio_programs[i].pio, sega_pio_programs[i].sm, sega_pio_programs[i].offset, &sega_pio_programs[i].config);
		pio_sm_set_enabled(sega_pio_programs[i].pio, sega_pio_programs[i].sm, true);
	}

	// These are the two functions that need to be called in order to start a program
	//pio_sm_init(pio, sm, offset, &c);
	//pio_sm_set_enabled(pio, sm, true);

	// while(1) { tight_loop_contents(); }
}

void swap_cs_detect_for_rw_detect() {

	/* SETUP CYCLE COUNT MEASUREMENT (Requires 5 cycles to measure, subtract from final value for a more accurate count of the actual code) */
	volatile uint32_t end_tick = 0;
	volatile uint32_t start_tick = systick_hw->cvr;


	uint from_program0;
	uint from_program1;
	uint to_program0;
	uint to_program1;

	// Swap CS to RW
	if (isRunningCSDetect) {
		from_program0 = 1;
		from_program1 = 2;
		to_program0 = 3;
		to_program1 = 4;
		isRunningCSDetect = false;

	// Swap RW to CS
	} else {
		from_program0 = 3;
		from_program1 = 4;
		to_program0 = 1;
		to_program1 = 2;
		isRunningCSDetect = true;
	}

	// Disable the SMs we are swapping from
	pio_sm_set_enabled(sega_pio_programs[from_program0].pio, sega_pio_programs[from_program0].sm, false);
	pio_sm_set_enabled(sega_pio_programs[from_program1].pio, sega_pio_programs[from_program1].sm, false);

	// Start new program 0
	pio_sm_init(sega_pio_programs[to_program0].pio, sega_pio_programs[to_program0].sm, sega_pio_programs[to_program0].offset, &sega_pio_programs[to_program0].config);
	pio_sm_set_enabled(sega_pio_programs[to_program0].pio, sega_pio_programs[to_program0].sm, true);

	// Start new program 1
	pio_sm_init(sega_pio_programs[to_program1].pio, sega_pio_programs[to_program1].sm, sega_pio_programs[to_program1].offset, &sega_pio_programs[to_program1].config);
	pio_sm_set_enabled(sega_pio_programs[to_program1].pio, sega_pio_programs[to_program1].sm, true);


	/* CALCULATE CYCLE COUNT */
	end_tick = systick_hw->cvr;
	volatile uint32_t totalTicks = (start_tick - end_tick);
	printf("%u ticks [s=%u, e=%u]\n", totalTicks, start_tick, end_tick);
}

volatile uint32_t ctrl_line_values = 0;
volatile uint32_t last_ctrl_line_values = 1;
void test_control_line_response_time() {
	bool has_reset = false;
	uint32_t values = 0;
	volatile bool rd = 0;
	volatile bool wr = 0;
	volatile bool da0 = 0;
	volatile bool da1 = 0;
	volatile bool da2 = 0;
	volatile bool cs0 = 0;
	volatile bool cs1 = 0;
	volatile uint8_t selected_register_index = 14;
	volatile uint8_t selected_register = 0;
	while(1) {
		tight_loop_contents();
		// fetch control line data

		// uint32_t values = gpio_get_all();
		// ctrl_line_values = sio_hw->gpio_in & MCU2_REGISTER_SELECT_PIN_MASK;
		// Skip if both cs lines are high or the state hasn't changed
		// if ((ctrl_line_values & 0x18) == 0x18 || last_ctrl_line_values == ctrl_line_values) {
		//     last_ctrl_line_values = ctrl_line_values; // reset the last value
		//     continue;
		// }

		// Continue if both cs lines are high
		if (gpio_get(4) && gpio_get(3)) {
			has_reset = true;
			continue;
		}

		// Continue if the CS lines haven't reset
		if (!has_reset) {
			continue;
		}

		has_reset = false;

		// We have determined that the control line has changed, signal
		gpio_put(18, true);

		// Also connected to mcu1 but still connected to prototype board's mcu2
		values = gpio_get_all();
		rd = values & 0x20; // pin 5
		wr = values & 0x40; // pin 6

		selected_register_index = 14;
		selected_register = 0;

		if (rd == 0) {
			da0 = (values >> MCU1_PIN_A0) & 0x1;
			da1 = (values >> MCU1_PIN_A1) & 0x1;
			da2 = (values >> MCU1_PIN_A2) & 0x1;
			cs0 = (values >> MCU1_PIN_CS0) & 0x1;
			cs1 = (values >> MCU1_PIN_CS1) & 0x1;
			// Decode with flipped cs values
			SPI_select_register(cs1, cs0, da2, da1, da0, rd, wr, &selected_register, &selected_register_index);
		} else if (wr == 0) {
			da0 = (values >> MCU1_PIN_A0) & 0x1;
			da1 = (values >> MCU1_PIN_A1) & 0x1;
			da2 = (values >> MCU1_PIN_A2) & 0x1;
			cs0 = (values >> MCU1_PIN_CS0) & 0x1;
			cs1 = (values >> MCU1_PIN_CS1) & 0x1;

			// Decode with flipped cs values
			SPI_select_register(cs1, cs0, da2, da1, da0, rd, wr, &selected_register, &selected_register_index);
		}

		gpio_put(18, false);
		// last_ctrl_line_values = ctrl_line_values;

	} // end while
} // end func

void init_sega_cs_detect(uint pin, uint sm) {
	PIO pio = pio1;
	uint offset = pio_add_program(pio, &sega_cs_detect_program);
	pio_sm_config c = sega_cs_detect_program_get_default_config(offset);

	pio_gpio_init(pio, pin);
	pio_gpio_init(pio, 18);

	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
	sm_config_set_in_pins(&c, pin);

	sm_config_set_set_pins(&c, 18, 1); // trigger gpio

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	pio->input_sync_bypass |= (1u << pin);
}

void init_sega_cs0_detect_program(sega_pio_program_t* sega_pio_program) {
	// CS0 program

	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	pio_gpio_init(pio, MCU1_PIN_CS0);

	// for trigger gpio, used for benchmarking
	// pio_gpio_init(pio, 18);

	pio_sm_set_consecutive_pindirs(pio, sm, MCU1_PIN_CS0, 1, false);
	sm_config_set_in_pins(&c, MCU1_PIN_CS0);

	// trigger gpio
	// sm_config_set_set_pins(&c, 18, 1);

	// DONT START HERE
	// pio_sm_init(pio, sm, offset, &c);
	// pio_sm_set_enabled(pio, sm, true);

	// DISABLE INPUT SYNC BYPASS for PIN CS0
	pio->input_sync_bypass |= (1u << MCU1_PIN_CS0);
}

void init_sega_cs1_detect_program(sega_pio_program_t* sega_pio_program) {
	// CS1 program

	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	pio_gpio_init(pio, MCU1_PIN_CS1);

	pio_sm_set_consecutive_pindirs(pio, sm, MCU1_PIN_CS1, 1, false);
	sm_config_set_in_pins(&c, MCU1_PIN_CS1);

	// trigger gpio
	// sm_config_set_set_pins(&c, 18, 1);

	// DONT START HERE
	// pio_sm_init(pio, sm, offset, &c);
	// pio_sm_set_enabled(pio, sm, true);

	// DISABLE INPUT SYNC BYPASS for PIN CS1
	pio->input_sync_bypass |= (1u << MCU1_PIN_CS1);
}

void init_sega_read_detect_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;
}

void init_sega_write_detect_program(sega_pio_program_t* sega_pio_program) {
	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;
}

void init_sega_combined_cs0_read_detect_program(sega_pio_program_t* sega_pio_program) {

}

void init_sega_combined_cs1_write_detect_program(sega_pio_program_t* sega_pio_program) {

}

void init_test_irq_detect_program(sega_pio_program_t* sega_pio_program) {

// TODO THIS FUNCTION WAS LEFT OVER WHEN PINS WERE CONNECTED TO MCU2
// I changed pin numbers to reflect the current pis on mcu1
// but unsure of any more changes that need to happen to use this method

	PIO pio = (*sega_pio_program).pio;
	uint sm = (*sega_pio_program).sm;
	uint offset = (*sega_pio_program).offset;
	pio_sm_config c = (*sega_pio_program).config;

	// Control pins
	// pio_gpio_init(pio, 0);
	// pio_gpio_init(pio, 1);
	// pio_gpio_init(pio, 2);
	// pio_gpio_init(pio, 3);
	// pio_gpio_init(pio, 4);

	// Read and write...
	// pio_gpio_init(pio, 16);
	// pio_gpio_init(pio, 17);

	pio_sm_set_consecutive_pindirs(pio, sm, 0, 7, false);
	sm_config_set_in_pins(&c, 0);

	// trigger gpio... eventually this will likely be the mux pin to toggle
	// between data bus and control lines
	pio_gpio_init(pio, 18);
	sm_config_set_set_pins(&c, 18, 1);

	// DONT START HERE
	// pio_sm_init(pio, sm, offset, &c);
	// pio_sm_set_enabled(pio, sm, true);
}

inline void test_swapping_cs_rw_detect_programs() {
volatile uint32_t startTicks[10] = {0};
volatile uint32_t endTicks[10] = {0};
volatile uint32_t st1, et1;
/* SETUP CYCLE COUNT MEASUREMENT (Requires 5 cycles to measure, subtract from final value for a more accurate count of the actual code) */
systick_hw->csr = 0x5;
systick_hw->rvr = 0x00FFFFFF;

st1 = systick_hw->cvr;
et1 = systick_hw->cvr;

// These two lines are apparently taking 106 ticks.....
startTicks[0] = systick_hw->cvr;
endTicks[0] = systick_hw->cvr;

	startTicks[1] = systick_hw->cvr;

/* TEST 1*/

	uint offset1 = 0;
	uint offset2 = 0;

	// pio_sm_exec(pio, sm, pio_encode_jmp(initial_pc));
	pio_sm_exec(pio1, 1, pio_encode_jmp(offset1));
	pio_sm_exec(pio1, 2, pio_encode_jmp(offset2));


	// uint from_program0;
	// uint from_program1;
	// uint to_program0;
	// uint to_program1;
	// uint32_t mask = 0x6;

	// from_program0 = 1;
	// from_program1 = 2;
	// to_program0 = 3;
	// to_program1 = 4;
	// Mask disable
	// startTicks[3] = systick_hw->cvr;
	// 	pio1->ctrl = (pio1->ctrl & ~mask) | (0u);
	// endTicks[3] = systick_hw->cvr;

	// Init new programs
	// START
	// startTicks[2] = systick_hw->cvr;
	// 	// pio_sm_init(pio1, 1, sega_pio_programs[to_program0].offset, &sega_pio_programs[to_program0].config);

	// 	pio_sm_set_config(pio1, 1, &sega_pio_programs[to_program0].config);
	// 	hw_xor_bits(&pio1->sm[1].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    // 	hw_xor_bits(&pio1->sm[1].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
	// 	// Clear FIFO debug flags
    // 	const uint32_t fdebug_sm_mask1 =
    //         (1u << PIO_FDEBUG_TXOVER_LSB) |
    //         (1u << PIO_FDEBUG_RXUNDER_LSB) |
    //         (1u << PIO_FDEBUG_TXSTALL_LSB) |
    //         (1u << PIO_FDEBUG_RXSTALL_LSB);
    // 	pio1->fdebug = fdebug_sm_mask1 << 1;

	// 	// Restart the SM
	// 	pio1->ctrl |= 1u << (PIO_CTRL_SM_RESTART_LSB + 1);
	// 	pio1->ctrl |= 1u << (PIO_CTRL_CLKDIV_RESTART_LSB + 1);

	// 	// pio_sm_exec
	// 	pio1->sm[1].instr = (pio_instr_bits_jmp | (0 << 5u) | ((sega_pio_programs[to_program0].offset) & 0x1fu));

	// // END
	// endTicks[2] = systick_hw->cvr;

	// //pio_sm_init(pio1, 2, sega_pio_programs[to_program1].offset, &sega_pio_programs[to_program1].config);

	// pio_sm_set_config(pio1, 2, &sega_pio_programs[to_program1].config);
	// 	hw_xor_bits(&pio1->sm[2].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    // 	hw_xor_bits(&pio1->sm[2].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
	// 	// Clear FIFO debug flags
    // 	const uint32_t fdebug_sm_mask2 =
    //         (1u << PIO_FDEBUG_TXOVER_LSB) |
    //         (1u << PIO_FDEBUG_RXUNDER_LSB) |
    //         (1u << PIO_FDEBUG_TXSTALL_LSB) |
    //         (1u << PIO_FDEBUG_RXSTALL_LSB);
    // 	pio1->fdebug = fdebug_sm_mask2 << 2;

	// 	// Restart the SM
	// 	pio1->ctrl |= 1u << (PIO_CTRL_SM_RESTART_LSB + 2);
	// 	pio1->ctrl |= 1u << (PIO_CTRL_CLKDIV_RESTART_LSB + 2);

	// 	// pio_sm_exec
	// 	pio1->sm[2].instr = (pio_instr_bits_jmp | (0 << 5u) | ((sega_pio_programs[to_program1].offset) & 0x1fu));

	// // Mask enable
	// startTicks[4] = systick_hw->cvr;
	// 	pio1->ctrl = (pio1->ctrl & ~mask) | (mask);
	// endTicks[4] = systick_hw->cvr;

	// /* CALCULATE CYCLE COUNT */
	// endTicks[1] = systick_hw->cvr;

// /* TEST 2 */
// 	startTicks[2] = systick_hw->cvr;
// 	from_program0 = 3;
// 	from_program1 = 4;
// 	to_program0 = 1;
// 	to_program1 = 2;

// 	// Mask disable
// 	pio1->ctrl = (pio1->ctrl & ~mask) | (0u);
// 	// pio_set_sm_mask_enabled(pio1, mask, true to enable/false to disable)
// 	// Disable the SMs we are swapping from
// 	// pio_sm_set_enabled(pio1, 1, false);
// 	// pio_sm_set_enabled(pio1, 2, false);

// 	// Start new program 0
// 	pio_sm_init(pio1, 1, sega_pio_programs[to_program0].offset, &sega_pio_programs[to_program0].config);
// 	// pio_sm_set_enabled(pio1, 1, true);

// 	// Start new program 1
// 	pio_sm_init(pio1, 2, sega_pio_programs[to_program1].offset, &sega_pio_programs[to_program1].config);
// 	// pio_sm_set_enabled(pio1, 2, true);

// 	// Mask enable
// 	pio1->ctrl = (pio1->ctrl & ~mask) | (mask);

// 	endTicks[2] = systick_hw->cvr;


// 	printf("---[s=%u, e=%u]\n", st1, et1);
	printf("TEST INIT == %u ticks [s=%u, e=%u]\n", startTicks[0] - endTicks[0], startTicks[0], endTicks[0]);
	printf("TEST1: %u ticks [s=%u, e=%u]\n", startTicks[1] - endTicks[1], startTicks[1], endTicks[1]);
	printf("2=%u, 3=%u, 4=%u\n", startTicks[2] - endTicks[2], startTicks[3] - endTicks[3], startTicks[4] - endTicks[4]);
}