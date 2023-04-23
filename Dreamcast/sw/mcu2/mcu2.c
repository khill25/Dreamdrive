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
#include "hardware/vreg.h"

#include "sega_packet_interface.h"

#include "sega_cs_detect.pio.h"

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
			da0 = (values >> MCU2_PIN_A0) & 0x1;
			da1 = (values >> MCU2_PIN_A1) & 0x1;
			da2 = (values >> MCU2_PIN_A2) & 0x1;
			cs0 = (values >> MCU2_PIN_IDE_CS0) & 0x1;
			cs1 = (values >> MCU2_PIN_IDE_CS1) & 0x1;
			// Decode with flipped cs values
			SPI_select_register(cs1, cs0, da2, da1, da0, rd, wr, &selected_register, &selected_register_index);
		} else if (wr == 0) {
			da0 = (values >> MCU2_PIN_A0) & 0x1;
			da1 = (values >> MCU2_PIN_A1) & 0x1;
			da2 = (values >> MCU2_PIN_A2) & 0x1;
			cs0 = (values >> MCU2_PIN_IDE_CS0) & 0x1;
			cs1 = (values >> MCU2_PIN_IDE_CS1) & 0x1;

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

void init_sega_cs0_detect_program() {
	// CS0 program

	PIO pio = pio1;
	uint sm = 0;
	uint offset = pio_add_program(pio, &sega_cs0_detect_program);
	pio_sm_config c = sega_cs0_detect_program_get_default_config(offset);

	pio_gpio_init(pio, MCU2_PIN_IDE_CS0);

	// for trigger gpio, used for benchmarking
	// pio_gpio_init(pio, 18);

	pio_sm_set_consecutive_pindirs(pio, sm, MCU2_PIN_IDE_CS0, 1, false);
	sm_config_set_in_pins(&c, MCU2_PIN_IDE_CS0);

	// trigger gpio
	// sm_config_set_set_pins(&c, 18, 1);

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	pio->input_sync_bypass |= (1u << MCU2_PIN_IDE_CS0);
}

void init_sega_cs1_detect_program() {
	// CS1 program

	PIO pio = pio1;
	uint sm = 1;
	uint offset = pio_add_program(pio, &sega_cs1_detect_program);
	pio_sm_config c = sega_cs1_detect_program_get_default_config(offset);

	pio_gpio_init(pio, MCU2_PIN_IDE_CS1);

	pio_sm_set_consecutive_pindirs(pio, sm, MCU2_PIN_IDE_CS1, 1, false);
	sm_config_set_in_pins(&c, MCU2_PIN_IDE_CS1);

	// trigger gpio
	// sm_config_set_set_pins(&c, 18, 1);

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	pio->input_sync_bypass |= (1u << MCU2_PIN_IDE_CS1);
}

void init_test_irq_detect_program() {

	PIO pio = pio1;
	uint sm = 2;
	uint offset = pio_add_program(pio, &test_irq_set_gpio_program);
	pio_sm_config c = test_irq_set_gpio_program_get_default_config(offset);

	// Control pins
	pio_gpio_init(pio, 0);
	pio_gpio_init(pio, 1);
	pio_gpio_init(pio, 2);
	pio_gpio_init(pio, 3);
	pio_gpio_init(pio, 4);

	// Read and write...
	pio_gpio_init(pio, 5);
	pio_gpio_init(pio, 6);

	pio_sm_set_consecutive_pindirs(pio, sm, 0, 7, false);
	sm_config_set_in_pins(&c, 0);

	// trigger gpio... eventually this will likely be the mux pin to toggle
	// between data bus and control lines
	pio_gpio_init(pio, 18);
	sm_config_set_set_pins(&c, 18, 1);

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	// pio->input_sync_bypass |= (1u << MCU2_PIN_IDE_CS1);
}

void test_control_lines_response_time_with_pio() {

	PIO pio = pio1;

	//setup pio, 1 per CS
	// init_sega_cs_detect(3, 0);
	// init_sega_cs_detect(4, 1);

	init_sega_cs0_detect_program();
	init_sega_cs1_detect_program();
	init_test_irq_detect_program();

	uint8_t registersAccessed[12] = {0};

	volatile uint32_t values = 0;
	volatile uint16_t codedValues = 0;
	volatile uint8_t selected_register;
	volatile uint8_t selected_register_index;
	// uint8_t i = 0;

	while(1) {

		while (!(pio->irq & (1<<0)));
		pio_interrupt_clear(pio, 0);

		gpio_put(18, true);

		values = gpio_get_all();

		codedValues = values & 0x7F;

		SPI_select_register_coded(
			codedValues & 0x1F, // cs0, cs1, a0, a1, a2
			codedValues & 0x20, // read
			codedValues & 0x40, // write
			&selected_register, // pointer to found register
			&selected_register_index // pointer to found register index
			);

		// registersAccessed[i++] = selected_register_index;

		gpio_put(18, false);

		// if (i == 7) {
		//     for(i = 0; i < 7; i++) {
		//         printf("%u, ", registersAccessed[i]);
		//     }
		//     printf("\n");
		// }
	}
}

/*
 * Sega Bus <-> PIO Usage
 **** PIO_0 ****
 *	* SM0
 *		* CS0 Detect
 *	* SM1
 *		* CS1 Detect
 * 	* SM2
 * 		* Sega_bus (handles swapping pins, the mux, pushing the control lines, reading from fifo to write to pins, and reading from pins to fifo)
 *	* SM3 (FREE)
 *
 *
 **** PIO_1 ****
 *	* SM0
 * 		* COMMS_TX
 * 	* SM1
 * 		* COMMS_RX
 * 	* SM2
 * 		* Sega READ Detect
 * 	* SM3
 * 		* Sega WRITE Detect
 *
 * ******
 * Unfortunatly SDIO requires an entire PIO block (I think 3 SM)
 *
 * If we handle the RD/WR detection in two separate SMs we won't have enough to use SD card in SDIO mode and would need to fall back to SPI.
 * I'm unsure if SPI will be fast enough, more testing will be needed so see if it's viable.
 */

void setup_sm_test() {
	PIO pio = pio1;
	uint sm = 0;

	uint offset = pio_add_program(pio, &sega_cs0_detect_program);
	pio_sm_config c = sega_cs0_detect_program_get_default_config(offset);
	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	sm = 1;
	offset = pio_add_program(pio, &sega_cs1_detect_program);
	c = sega_cs1_detect_program_get_default_config(offset);
	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	sm = 2;
	offset = pio_add_program(pio, &test_irq_set_gpio_program);
	c = test_irq_set_gpio_program_get_default_config(offset);
	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);

	// stop cs0 detect // Then start the read detect program?
	// sm = 0;
	// pio_sm_set_enabled(pio, sm, false);
	offset = pio_add_program(pio, &sega_read_detect_program);
	pio_sm_config read_detect_config = sega_read_detect_program_get_default_config(offset);
	// pio_sm_init(pio, sm, offset, &read_detect_config);
	// pio_sm_set_enabled(pio, sm, true);

	// stop cs1 detect // Then start the write detect program?
	// sm = 1;
	// pio_sm_set_enabled(pio, sm, false);
	offset = pio_add_program(pio, &sega_write_detect_program);
	pio_sm_config write_detect_config = sega_write_detect_program_get_default_config(offset);
	// pio_sm_init(pio, sm, offset, &write_detect_config);
	// pio_sm_set_enabled(pio, sm, true);
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
	gpio_set_dir(5, false);

	gpio_init(6);
	gpio_set_dir(6, false);
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
	// const int freq_khz = 336000;
	// vreg_set_voltage(VREG_VOLTAGE_1_25);
	bool clockWasSet = set_sys_clock_khz(freq_khz, false);
	printf("Clock of %uMhz was set: %u\n", freq_khz / 1000, clockWasSet);

	gpio_init(18);
	gpio_set_dir(18, true);
	gpio_set_pulls(18, false, true);

	gpio_put(18, true);
	gpio_put(18, false);

	uint8_t lastSampled = 0;
	bool didProcessBuffer = false;
	volatile uint32_t lineChangeCount = 0;
	volatile uint32_t startTime = 0;
	// volatile uint8_t buf[] = {0};
	// volatile uint8_t v = 0;

	volatile uint32_t c = 0;

	// Loop forever and test
	// test_control_line_response_time();
	test_control_lines_response_time_with_pio();
}
