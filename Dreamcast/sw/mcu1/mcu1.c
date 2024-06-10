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

#include "sega_databus.pio.h"

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

// You can use the result of this function to pass into printNameOfRegister to get a string name of the register
int registerIndexFromControlValue(uint32_t controlValue) {
	switch(controlValue) {
    case 0x2E:
        return SPI_ALTERNATE_STATUS_REGISTER_INDEX; // read
    case 0x4E:
        return SPI_DEVICE_CONTROL_REGISTER_INDEX; // write
    case 0x30:
        return SPI_DATA_REGISTER_INDEX; // read
    case 0x50:
        return SPI_DATA_REGISTER_INDEX; // write
    case 0x51:
        return SPI_FEATURES_REGISTER_INDEX;
    case 0x31:
        return SPI_ERROR_REGISTER_INDEX;
    case 0x32:
        return SPI_INTERRUPT_REASON_REGISTER_INDEX; // read only
    case 0x33:
        return SPI_SECTOR_NUMBER_REGISTER_INDEX; // read only
    case 0x34:
        return SPI_BYTE_COUNT_REGISTER_LOW_INDEX; // read
    case 0x54:
        return SPI_BYTE_COUNT_REGISTER_LOW_INDEX; // write
    case 0x35:
        return SPI_BYTE_COUNT_REGISTER_HIGH_INDEX; // read
    case 0x55:
        return SPI_BYTE_COUNT_REGISTER_HIGH_INDEX; // write
    case 0x56:
        return SPI_DRIVE_SELECT_REGISTER_INDEX; // write
    case 0x36:
        return SPI_DRIVE_SELECT_REGISTER_INDEX; // read
    case 0x37:
        return SPI_STATUS_REGISTER_INDEX; // read
    case 0x57:
        return SPI_COMMAND_REGISTER_INDEX; // write
    default:
        // Handle unexpected index
        return SPI_REGISTER_COUNT;
	}
}

#define DEBUG_UART_BAUD_RATE 115200

// Map values to commands, start with all values loaded to invalid (register count)
uint16_t* registerIndex_map[128] = {0};
volatile uint16_t* selectedRegister = 0;
volatile uint32_t register_index = SPI_REGISTER_COUNT;

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
	gpio_set_pulls(MCU1_PIN_MUX_SELECT, false, true); // enable pull down to default to control lines

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
	gpio_put(MCU1_PIN_MUX_SELECT, false);

	// MUX LOW  = CONTROL lines
	// MUX HIGH = DATA lines

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

	printf("Setting up register map...");

	// This is used to quickly get the right register, read/write should be handled by whatever is doing the lookup
	// Register Index = Bits = W, R, CS1, CS0, A2, A1, A0 (most->least)
	registerIndex_map[0x2E] = &SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]; // read
	registerIndex_map[0x4E] = &SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]; // write

	registerIndex_map[0x30] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // read
	registerIndex_map[0x50] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // write

	registerIndex_map[0x51] = &SPI_registers[SPI_FEATURES_REGISTER_INDEX]; // read
	registerIndex_map[0x31] = &SPI_registers[SPI_ERROR_REGISTER_INDEX]; // write

	registerIndex_map[0x32] = &SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX]; // read only

	registerIndex_map[0x33] = &SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX]; // read only

	registerIndex_map[0x34] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]; // read
	registerIndex_map[0x54] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]; // write

	registerIndex_map[0x35] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]; // read
	registerIndex_map[0x55] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]; // write

	registerIndex_map[0x56] = &SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]; // write
	registerIndex_map[0x36] = &SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]; // read

	registerIndex_map[0x37] = &SPI_registers[SPI_STATUS_REGISTER_INDEX]; // read
	registerIndex_map[0x57] = &SPI_registers[SPI_COMMAND_REGISTER_INDEX]; // write

	// For the rest of the values, just use a dump register
	for(int i = 0; i < 128; i++) {
		if (registerIndex_map[i] == 0) {
			registerIndex_map[i] = &SPI_registers[SPI_REGISTER_COUNT]; // Use the SPI_REGISTER_COUNT as a dump register
		}
	}
	
	printf("Dreamcast booting...\n");

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

	while(!gpio_get(3)); // loop until the cs lines are active (really only useful when powering the board on before the console)

	busy_wait_ms(1200); // TODO this is likely not needed? 

	volatile uint32_t readWriteLineValues = 0;
	// volatile uint32_t rawLineValues = 0;
	selectedRegister = registerIndex_map[SPI_REGISTER_COUNT];
	volatile uint8_t dreamcastWantsRead = 0;
	// 00 (0x0) - nothing
	// 01 (0x1) - read
	// 10 (0x2) - write
	// 11 (0x3) - nothing

	while(1) {
		
		// Worst case loop is 172ns
		// Read/write are low for ~300ns
		// This leaves us with 128ns (32cycles @ 4ns)
		// TODO do we need to do anything with the register data???

		// Signal is active low
		// 1 and 2 are the only valid value. Either read OR write is low, but not both high or both low
		do {
			readWriteLineValues = sio_hw->gpio_in & READ_WRITE_PIN_MASK;						// 16ns (4 cycles)
		} while(readWriteLineValues == 3 || readWriteLineValues == 0);							// 12ns (3 cycles)

		// bit shift in read/write values to the control line values
		register_index = (sio_hw->gpio_in & 0x1F) | (readWriteLineValues << 5);  				// 24ns (6 cycles @ 4ns)
		// get the pointer to the selected register
		selectedRegister = registerIndex_map[register_index]; 									// 24ns (6 cycles)

		// flip the mux to data lines
		sio_hw->gpio_set = 1ul << MCU1_PIN_MUX_SELECT;											// 12ns (3 cycles)	

		// READ - SEND data to dreamcast
		if (readWriteLineValues == 0x1) {														// 16ns (2-4 cycles)
			// Set register values on lines
			sio_hw->gpio_togl = (sio_hw->gpio_out ^ *selectedRegister) & ATA_REGISTER_PIN_MASK; // 32ns (8 cycles?)

			// ... 136ns to put data on the lines
			// read and write latches are low for 304ns
			// this *SHOULD* work

			// wait for latch?
			while(gpio_get(MCU1_PIN_READ) == 0) { tight_loop_contents(); };						// 24ns (6 cycles)

		// WRITE - GET data from dreamcast into register
		} else {
			// Since the mux is flipped to data lines we want to read all 16 bits 
			// Hope that for non-data registers that the upper 8 bits won't mess up when
			// sending data back to the dreamcast

			// Read all the data lines
			*selectedRegister = sio_hw->gpio_in & ATA_REGISTER_PIN_MASK;						// 16ns (4 cycles?)

			// .. process data while we wait for latch. 
			// Use second core?
			
			// wait for latch?
			while(gpio_get(MCU1_PIN_WRITE) == 0) { tight_loop_contents(); };					// 24ns (6 cycles)
		}

		// flip the mux back to control lines
		sio_hw->gpio_clr = 1ul << MCU1_PIN_MUX_SELECT;											// 12ns (3 cycles)

		// (this point takes about 172ns from the beginning of the do loop)
		// This means that there is about 128ns extra time to do something before the next read/write cycle
		// This is 32 cycles at 4ns per cycle
	}

	
	return 0;
}

void second_core_main() {
	while(1) {
		// Wait for data from core1 to be available
		// ... TODO figure out how to do this

		// register_index -> selected register 
		// selectedRegister -> register pointer

		// The command register is the only one that needs to be processed
		// !!!BSY bit must be set within 400ns, so if we need more time, this bit should be set
		if(register_index == SPI_COMMAND_REGISTER_INDEX) {
			switch (*selectedRegister) {
				case ATA_CMD_NOP:{
					// Command can be received when BSY bit is 1 
					// and device should terminate the command currently in execution
					break;
				}
				case ATA_CMD_SOFT_RESET: {
					break;
				}
				case ATA_CMD_PACKET_COMMAND: {
					// Process sega packet interface
					// ...
					break;
				}
				case ATA_CMD_IDENTIFY_DEVICE: {
					break;
				}
				case ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC: {
					break;
				}
				case ATA_CMD_SET_FEATURES: {
					break;
				}
			}
		}

		/*
		Switch case below....
		Best Case: 5 cycles.
		Worst Case: 33 cycles.
		Average Case: Approximately 19 cycles.
		--
		Remove register cases that don't need to be processed
		*/
		// switch(register_index) {
			// case SPI_COMMAND_REGISTER_INDEX: { break; }
			// case SPI_ATA_IO_REGISTER_INDEX: { break; }
			// case SPI_STATUS_REGISTER_INDEX: { break; } 
			// case SPI_ALTERNATE_STATUS_REGISTER_INDEX: { break; }
			// case SPI_DATA_REGISTER_INDEX: { break; }
			// case SPI_DEVICE_CONTROL_REGISTER_INDEX: { break; }
			// case SPI_DRIVE_SELECT_REGISTER_INDEX: { break; }  
			// case SPI_INTERRUPT_REASON_REGISTER_INDEX: { break; }
			// case SPI_SECTOR_COUNT_REGISTER_INDEX: { break; }
			// case SPI_SECTOR_NUMBER_REGISTER_INDEX: { break; }
			// case SPI_ERROR_REGISTER_INDEX: { break; }    
			// case SPI_FEATURES_REGISTER_INDEX: { break; }
			// case SPI_BYTE_COUNT_REGISTER_LOW_INDEX: { break; }
			// case SPI_BYTE_COUNT_REGISTER_HIGH_INDEX: { break; }
			// case SPI_REGISTER_COUNT: { break; }
		// }
	}
}
