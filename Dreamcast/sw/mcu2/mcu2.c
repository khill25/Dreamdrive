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

#define CONTROL_PIN_MASK 0x0000FFFF
volatile uint32_t last_ctrl_pin_sample = 0x0;

// void test_control_lines_response_time_with_pio() {

// 	PIO pio = pio1;

// 	//setup pio, 1 per CS
// 	// init_sega_cs_detect(3, 0);
// 	// init_sega_cs_detect(4, 1);

// 	init_sega_cs0_detect_program();
// 	init_sega_cs1_detect_program();
// 	init_test_irq_detect_program();

// 	uint8_t registersAccessed[12] = {0};

// 	volatile uint32_t values = 0;
// 	volatile uint16_t codedValues = 0;
// 	volatile uint8_t selected_register;
// 	volatile uint8_t selected_register_index;
// 	// uint8_t i = 0;

// 	while(1) {

// 		while (!(pio->irq & (1<<0)));
// 		pio_interrupt_clear(pio, 0);

// 		gpio_put(18, true);

// 		values = gpio_get_all();

// 		codedValues = values & 0x7F;

// 		SPI_select_register_coded(
// 			codedValues & 0x1F, // cs0, cs1, a0, a1, a2
// 			codedValues & 0x20, // read
// 			codedValues & 0x40, // write
// 			&selected_register, // pointer to found register
// 			&selected_register_index // pointer to found register index
// 			);

// 		// registersAccessed[i++] = selected_register_index;

// 		gpio_put(18, false);

// 		// if (i == 7) {
// 		//     for(i = 0; i < 7; i++) {
// 		//         printf("%u, ", registersAccessed[i]);
// 		//     }
// 		//     printf("\n");
// 		// }
// 	}
// }

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

	sleep_ms(1000);

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
	// test_control_lines_response_time_with_pio();
}
