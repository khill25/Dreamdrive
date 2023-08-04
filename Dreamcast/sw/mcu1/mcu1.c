/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "mcu1_pins.h"
#include "shared.h"
#include "pio_uart/pio_uart.h"
#include "hardware/pio.h"
#include "sega_databus/sega_databus.h"

#include "ff.h" /* Obtains integer types */
#include "diskio.h" /* Declarations of disk functions */
#include "f_util.h"

#include "sega_packet_interface.h"

#include "hardware/structs/systick.h"

bool isMuxDataLines = true;

/*
 * Using 8line mux 2:1 (Common A, ControlLine B, Data C)
 * (A) gpio0-?
 * (B) A0, A1, A2, CS0, CS1 ()
 * (C) D0-D?
 *
 * 6 gpio SD Card
 * 16 Databus/muxed
 * 1 Mux select
 * ===== 23 pins
 * 5: rd, wr, intrq, dmack, dmarq
 * ===== 28 pins
 * X(whatever is left) for comms to mcu2
 *
 * These need to be available and not muxed?
 * rd
 * wr
 * intrq
 * dmack (host drives signal when signalling for more dma data)
 * dmarq (device drives signal when dma available)
 *
 * What sets of pins are used at the same time?
 * (20) d0-d15, marq, mack, intrq(?), rd, wr
 * (5)  a0-a2, cs0, cs2
 */

/*
May need to do something faster than waiting for mcu2 to send a packet every time it samples pins

MCU2 captures all the signal lines state changes
	* asserts one of the MCU control lines
	* MCU1 interrupts on that control line
	* That means to sample the pins
	* Store that result in a buffer
	* Once the control lines specify it's COMMAND_REGISTER time, MCU2 asserts control line like usual
		* but then sends all the data it received (so all the control line samples)
		* and MCU1 can line up the control lines with it's sampled (parallel) buffer

BSY must be set within 400ns of the command register being written
 */

void sdcard_read_test() {

}

void printNameOfRegister(uint8_t regIndex) {

	switch(regIndex) {
		case 0: printf("ATA_IO"); break;
		case 1: printf("STATUS"); break;
		case 2: printf("ALT_STATUS"); break;
		case 3: printf("COMMAND"); break;
		case 4: printf("BYTE_COUNT_LOW"); break;
		case 5: printf("BYTE_COUNT_HIGH"); break;
		case 6: printf("DATA"); break;
		case 7: printf("DEVICE_CONTROL"); break;
		case 8: printf("DRIVE_SELECT"); break;
		case 9: printf("ERROR"); break;
		case 10: printf("FEATURES"); break;
		case 11: printf("INTERRUPT_REASON"); break;
		case 12: printf("SECTOR_COUNT"); break;
		case 13: printf("SECTOR_NUMBER"); break;
		case 14: printf("INVALID"); break;
		default: break;
	}

// /* 0 */    SPI_ATA_IO_REGISTER_INDEX = 0       ,
// /* 1 */    SPI_STATUS_REGISTER_INDEX           ,
// /* 2 */    SPI_ALTERNATE_STATUS_REGISTER_INDEX ,
// /* 3 */    SPI_COMMAND_REGISTER_INDEX         ,
// /* 4 */    SPI_BYTE_COUNT_REGISTER_LOW_INDEX      , // Low bits
// /* 5 */    SPI_BYTE_COUNT_REGISTER_HIGH_INDEX      , // high bits
// /* 6 */    SPI_DATA_REGISTER_INDEX             , // use `SPI_data_register` to access this register
// /* 7 */    SPI_DEVICE_CONTROL_REGISTER_INDEX   ,
// /* 8 */    SPI_DRIVE_SELECT_REGISTER_INDEX     ,// ATA Drive/Head register
// /* 9 */    SPI_ERROR_REGISTER_INDEX            ,
// /* 10*/    SPI_FEATURES_REGISTER_INDEX         ,
// /* 11*/    SPI_INTERRUPT_REASON_REGISTER_INDEX , // Read only
// /* 12*/    SPI_SECTOR_COUNT_REGISTER_INDEX     , // Write only
// /* 13*/    SPI_SECTOR_NUMBER_REGISTER_INDEX    , // ATA Sector Number Register
// /* 14*/    SPI_REGISTER_COUNT // 14 = (0xE)

}

#define DEBUG_UART_BAUD_RATE 115200

int main(void) {
	current_mcu = MCU1;

	// Set clock speed to 266MHz (3.76ns per cycle)
	const int freq_khz = 266000;
	// const int freq_khz = 336000;
	// vreg_set_voltage(VREG_VOLTAGE_1_25); // Usually needed for clocks over 266MHz
	bool clockWasSet = set_sys_clock_khz(freq_khz, false);

	// stdio_init_all();
	stdio_uart_init_full(uart0, DEBUG_UART_BAUD_RATE, 28, -1);

	printf("Clock of %uMhz was set: %u\n", freq_khz / 1000, clockWasSet);

	// Setup the Mux select line
	gpio_init(MCU1_PIN_MUX_SELECT);
	gpio_set_dir(MCU1_PIN_MUX_SELECT, true);
	gpio_set_pulls(MCU1_PIN_MUX_SELECT, false, true); // enable pull down to default to data lines

	// TODO Reestablish uart comms with mcu2
	// pio_uart_init(MCU1_PIN_PIO_COMMS_D0, MCU1_PIN_PIO_COMMS_D1);

	// TODO Revist how these pio programs are being used and if we still want them
	// setup_sega_pio_programs();

	printf("MCU1- Init pins...\n");

	// init the pins, 0-15 = data, 16 = read, 17 = write, 18 = interrupt, 19 = DMACK, 20 = DMARQ
	for (int i = 0; i <= 20; i++) {
		gpio_init(i);
		gpio_set_dir(i, false); // set to input
	}

	// gpio_init(MCU1_PIN_MUX_SELECT);
	// gpio_set_dir(MCU1_PIN_MUX_SELECT, true);

	// MUX to control lines
	// gpio_put(MCU1_PIN_MUX_SELECT, true);

	//invalid address
	//00xxx = 0; cs0 & cs1 are both low; cs0 == 0 && cs1 == 0

	//Data bus high impedance
	//11xxx = 0x18
	//010xx = 0x8
	//0110x = 0xC
	const uint32_t databus_high_imped0 = 0x8;
	const uint32_t databus_high_imped1 = 0xC;
	/* INVALID VALUES
	 * cs1, cs0, a2, a1, a0 | CS0-Assert, CS1-Assert, a2-0 : when looking at the table in the pdf,
	 		you can use the values as is without swapping them because an "A" in the table is 0v
	 *
	 * For completness and sanity:
	 * 11xxx = 11000, 11001, 11010, 11011, 11100, 11101, 11110, 11111
	 * 010xx = 01000, 01001, 01010, 01011
	 * 0110x = 01100, 01101
	 *
	 * Hex:
	 * 11xxx = 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
	 * 010xx = 0x8, 0x9, 0xA, 0xB
	 * 0110x = 0xC, 0xD
	 *
	 * 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xF?[0xF isn't mentioned in the table???]
	*/

	const uint32_t read_mask = 0x00010000; // pin 16
	const uint32_t write_mask = 0x00020000; // pin 17
	const uint32_t cs_mask = 0x00000018; // pin 3 and 4
	const uint32_t control_pin_mask = 0x1F;
	volatile bool last_rd = 0;
	volatile bool last_wr = 0;
	volatile bool last_csMask = 0;
	volatile bool rd = 0;
	volatile bool wr = 0;
	int numReadValues = 0;
	bool csIsReady = false;
	volatile uint32_t pins = 0;

	printf("MCU1- Start main loop\n");

	// Map values to commands, start with all values loaded to invalid (register count)
	uint8_t registerIndex_map[128] = {SPI_REGISTER_COUNT};

	// Now fill in the values we need
	// Register Index = Bits = W, R, CS1, CS0, A2, A1, A0 (most->least)
	registerIndex_map[0x2E] = SPI_ALTERNATE_STATUS_REGISTER_INDEX; // read
	registerIndex_map[0x4E] = SPI_DEVICE_CONTROL_REGISTER_INDEX; // write

	registerIndex_map[0x30] = SPI_DATA_REGISTER_INDEX; // read
	registerIndex_map[0x50] = SPI_DATA_REGISTER_INDEX; // write

	registerIndex_map[0x51] = SPI_FEATURES_REGISTER_INDEX;
	registerIndex_map[0x31] = SPI_ERROR_REGISTER_INDEX;

	registerIndex_map[0x32] = SPI_INTERRUPT_REASON_REGISTER_INDEX; // read only

	registerIndex_map[0x33] = SPI_SECTOR_NUMBER_REGISTER_INDEX; // read only

	registerIndex_map[0x34] = SPI_BYTE_COUNT_REGISTER_LOW_INDEX; // read
	registerIndex_map[0x54] = SPI_BYTE_COUNT_REGISTER_LOW_INDEX; // write

	registerIndex_map[0x35] = SPI_BYTE_COUNT_REGISTER_HIGH_INDEX; // read
	registerIndex_map[0x55] = SPI_BYTE_COUNT_REGISTER_HIGH_INDEX; // write

	registerIndex_map[0x56] = SPI_DRIVE_SELECT_REGISTER_INDEX; // write
	registerIndex_map[0x36] = SPI_DRIVE_SELECT_REGISTER_INDEX; // read

	registerIndex_map[0x37] = SPI_STATUS_REGISTER_INDEX; // read
	registerIndex_map[0x57] = SPI_COMMAND_REGISTER_INDEX; // write

	printf("FINISHED!\n");

	// The Dreamcast(something?) does a startup with the cd drive and it toggles all the control, read, and write lines.
	// Wait for that to be finished before we start out programs
	while(1) {
		pins = gpio_get_all();

		// Was off and is now on
		if(last_csMask == 0 && ((pins & cs_mask) == cs_mask)) {
			break;
		}

		last_csMask = (pins & cs_mask);
	}

	printf("Dreamcast started!\n");
	// Setup the databus programs and run them
	setup_sega_pio_programs();
	busy_wait_ms(100); // TODO this is likely not needed
	start_sega_pio_programs();

	PIO pio = pio1;
	volatile uint32_t controlLineValue = 0;
	volatile bool isRead = false;

	// Main Loop
	while(1) {
		/*
		 * databus program sends control line values
		 * Toggles mux and waits for rd or wr low
		 * Read pushes data
		 * Write pulls data
		 */
		controlLineValue = pio_sm_get_blocking(pio, sega_databus_handler_sm) & control_pin_mask;

		// TODO Need to know if this will be a rd or wr
		// ...
		



	}

	return 0;
}
