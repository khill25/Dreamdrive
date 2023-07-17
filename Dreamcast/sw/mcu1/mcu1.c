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
 * Once MCU1 has recieved info from MCU2 (control line update)
 * Process it and act if needed
 */
static void process_mcu2_data_and_exec_SPI_cmd_if_needed(bool rd, bool wr) {
	// Grab the state of the pins
	uint32_t current_databus_value = gpio_get_all(); /// TODO mask this data for the actual data bus pins
	// bool rd = current_databus_value & 0x10000000; // pin 28
	// bool wr = current_databus_value & 0x20000000; // pin 29

	uint8_t controlValues = mcu1_received_control_line_buffer.buf[mcu1_received_control_line_buffer.head-1];

	sega_databus_extract_raw_control_line_packet(controlValues, rd, wr);

	// at least 22 cycles coming to this line
	sega_databus_process_control_line_data(); // worst case another at least 24 cycles here

	//

	// At this point we have the register and register "name" (index)

	printf("data:%04x, index:%02x, rd:%u, wr:%u\n", databus_selected_register, databus_selected_register_index, rd, wr);

	if (databus_selected_register_index == SPI_REGISTER_COUNT) {
		return; // nothing to do here, invalid state
	}

	// Need to put sampled data into relevant registers

	// Once the register is the COMMAND_REGISTER, we can start processing as we will have all the data
	// set in the appropriate registers
	if(databus_selected_register_index == SPI_COMMAND_REGISTER_INDEX) {

	}

	// In most cases BSY register must be set within 400ns of the host sending this data
	// and we have likely spent most of that shuffling the data from mcu2 and then decoding it.

}
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

int main(void) {
	stdio_init_all();
	current_mcu = MCU1;

	sleep_ms(1000);

	// Set clock speed to 266MHz (3.76ns per cycle)
	const int freq_khz = 266000;
	// const int freq_khz = 336000;
	// vreg_set_voltage(VREG_VOLTAGE_1_25); // Usually needed for clocks over 266MHz
	bool clockWasSet = set_sys_clock_khz(freq_khz, false);
	printf("Clock of %uMhz was set: %u\n", freq_khz / 1000, clockWasSet);

	printf("MCU1- Init interconnect\n");

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

	gpio_init(MCU1_PIN_MUX_SELECT);
	gpio_set_dir(MCU1_PIN_MUX_SELECT, true);

	// MUX to control lines
	gpio_put(MCU1_PIN_MUX_SELECT, true);

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

	// debugging stuff
	volatile uint8_t wasARead[100] = {};
	volatile uint32_t values[100] = {};
	volatile uint32_t valIndex = 0;
	volatile bool hasPrinted = false;

	printf("MCU1- Start main loop\n");

	while(1) {
		pins = gpio_get_all();

		// Was off and is now on
		if(last_csMask == 0 && ((pins & cs_mask) == cs_mask)) {
			break;
		}

		last_csMask = (pins & cs_mask);
	}

	printf("Dreamcast started!\n");
	busy_wait_ms(1350);

	while(1) {
		pins = gpio_get_all();

		if (((pins & cs_mask) == cs_mask) || ((pins & cs_mask) == 0)) {
			continue;
		}

		rd = pins & read_mask;
		wr = pins & write_mask;

		if ((rd == 0 && last_rd == 1) || (wr == 0 && last_wr == 1)) {

			if (valIndex < 100) {
				wasARead[valIndex] = rd == 0 ? 1 : 0;
				values[valIndex++] = (pins & control_pin_mask);
			} else {
				if (!hasPrinted) {
					for(int i = 0; i < valIndex; i++) {
						// if (i % 8 == 0) { printf("\n"); }
						bool readPinVal = 0;
						bool writePinVal = 0;

						if (wasARead[i] == 1) {
							readPinVal = 1;
							writePinVal = 0;
						} else {
							readPinVal = 0;
							writePinVal = 1;
						}

						sega_databus_extract_raw_control_line_packet(values[i], readPinVal, writePinVal);
						sega_databus_process_control_line_data();

						printf("[%x+%u]%u:", values[i], wasARead[i], databus_selected_register_index);
						printNameOfRegister(databus_selected_register_index);
						printf("\n");
					}
					hasPrinted = true;
				}
			}

			//  Lets just print the pins out
			// rd and wr values need to be negated either HERE or in the METHOD as it's checking for read==1 when setting the register
			// sega_databus_extract_raw_control_line_packet(pins, !rd, !wr); // 33 cycles (~125ns @ 266MHz)
			// sega_databus_process_control_line_data();

			// if (databus_selected_register_index != 14) {
			// 	printf("%u=0x%x ", databus_selected_register_index, (pins & control_pin_mask));
			// }

			// This register pointer contains the data from the last command
			// databus_selected_register

			/* FOR NOW PRINT EVERYTHING OUT
			// If this is a read, we need to send data back and set some lines
			//SPI_COMMAND_REGISTER_INDEX == databus_selected_register_index
			if (wr == 1 && databus_selected_register_index != 14) {
				gpio_put(MCU1_PIN_MUX_SELECT, false); // switch to data lines

				uint32_t dataPins = gpio_get_all();

				gpio_put(MCU1_PIN_MUX_SELECT, true); // switch back to control lines

				uint8_t cmd = (uint8_t)(dataPins & 0x000000FF);
				printf("%u=0x%x ", databus_selected_register_index, cmd);
			}
			*/

			// printf("%u. ", databus_selected_register_index);
		}

		last_rd = rd;
		last_wr = wr;
	}

	// Debugging cs lines and IMPORTANT, var & bitmask need to be in PARENS!!!!!
	// LEAVING TO REMEMBER THE WARNING
	// while(1) {
	//     pins = gpio_get_all();

	//     if (((pins & cs_mask) == cs_mask) || ((pins & cs_mask) == 0)) {
	//         continue;
	//     } else {
	//         if (numPrints < 100) {
	//             printf("%x ", pins);
	//             numPrints++;
	//         }
	//     }

	//     rd = pins & read_mask;
	//     wr = pins & write_mask;

	//     if (rd == 0 && last_rd == 1) {
	//         printf("r ");
	//     } else if (wr == 0 && last_wr == 1) {
	//         printf("w ");
	//     }

	//     last_rd = rd;
	//     last_wr = wr;
	// }

	return 0;
}
