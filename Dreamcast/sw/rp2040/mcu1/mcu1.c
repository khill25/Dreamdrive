/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/systick.h"

#include "mcu1_pins.h"
#include "shared.h"
#include "pio_uart/pio_uart.h"
#include "hardware/pio.h"
#include "sega_databus/sega_databus.h"

#include "ff.h" /* Obtains integer types */
#include "diskio.h" /* Declarations of disk functions */
#include "f_util.h"

#include "sega_packet_interface.h"
#include "sega_databus.pio.h"

#include "mcu_databus.pio.h"

uint8_t current_transfer_mode = SPI_SECTOR_COUNT_TRANSFER_MODE_PIO_DEFAULT;

void sdcard_read_test() {

}

void printNameOfRegister(uint32_t regIndex) {

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
		default: {
			printf("BAD INDEX: %x", regIndex); 
			break;
		}
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
        return SPI_DEVICE_CONTROL_REGISTER_INDEX; // write
    case 0x4E:
		return SPI_ALTERNATE_STATUS_REGISTER_INDEX; // read
    case 0x30:
        return SPI_DATA_REGISTER_INDEX; // write
    case 0x50:
        return SPI_DATA_REGISTER_INDEX; // read
    case 0x51:
		return SPI_ERROR_REGISTER_INDEX;
    case 0x31:
        return SPI_FEATURES_REGISTER_INDEX;
    case 0x32:
		return SPI_SECTOR_COUNT_REGISTER_INDEX;
	case 0x52:
		return SPI_INTERRUPT_REASON_REGISTER_INDEX;
    case 0x33:
        return SPI_SECTOR_NUMBER_REGISTER_INDEX;
    case 0x34:
        return SPI_BYTE_COUNT_REGISTER_LOW_INDEX;
    case 0x54:
        return SPI_BYTE_COUNT_REGISTER_LOW_INDEX;
    case 0x35:
        return SPI_BYTE_COUNT_REGISTER_HIGH_INDEX;
    case 0x55:
        return SPI_BYTE_COUNT_REGISTER_HIGH_INDEX;
    case 0x56:
        return SPI_DRIVE_SELECT_REGISTER_INDEX;
    case 0x36:
        return SPI_DRIVE_SELECT_REGISTER_INDEX;
    case 0x37:
		return SPI_COMMAND_REGISTER_INDEX; // write
    case 0x57:
        return SPI_STATUS_REGISTER_INDEX; // read
    default:
        // Handle unexpected index
        return SPI_REGISTER_COUNT;
	}
}

#define DEBUG_UART_BAUD_RATE 115200
#define CORE1_PROCESS_REGISTER_CMD 0x1
#define CORE1_CHIRP_CMD 0x2

void second_core_main();

// Map values to commands, start with all values loaded to invalid (register count)
uint16_t* registerIndex_map[128] = {0};
volatile uint16_t* status_register = 0;
volatile uint16_t* selectedRegister = 0;
volatile uint32_t register_index = SPI_REGISTER_COUNT;

volatile uint32_t writtenRegisters[10000] = {0};
volatile uint32_t writtenRegisterIndex = 0;

static inline uint16_t swap8(uint16_t value)
{
	// 0x1122 => 0x2211
	return (value << 8) | (value >> 8);
}

static inline uint32_t swap16(uint32_t value)
{
	// 0x11223344 => 0x33441122
	return (value << 16) | (value >> 16);
}
// TODO probably need some kind of value to indicate to core1 that we don't need to process ata stuff
// like if we are in DMA mode transfering data to the dreamcast

#define MCU1_DATABUS_READ_SM (0)
#define MCU1_DATABUS_WRITE_SM (1)

void setup_mcu_databus_read() {
	uint sm = MCU1_DATABUS_READ_SM;
	uint offset = pio_add_program(pio0, &mcu_databus_read_program);
	pio_sm_config c = mcu_databus_read_program_get_default_config(offset);

	// We want to input on pins 8-15
	sm_config_set_in_pins(&c, MCU1_DATABUS_D0);
	sm_config_set_set_pins(&c, MCU_DATABUS_DEVICE_SIGNAL_PIN, 1);

	sm_config_set_in_shift(&c, false, false, 8);

	pio_sm_set_pindirs_with_mask(pio0, sm, 0x30000, 0x3FF00);

	pio_sm_init(pio0, sm, offset, &c);
}

void setup_mcu_databus_write() {
	uint sm = MCU1_DATABUS_WRITE_SM;
	uint offset = pio_add_program(pio0, &mcu_databus_write_program);
	pio_sm_config c = mcu_databus_write_program_get_default_config(offset);

	sm_config_set_out_pins(&c, MCU1_DATABUS_D0, 8);
	sm_config_set_set_pins(&c, MCU_DATABUS_DEVICE_WRITE_PIN, 1);
	// We want to output on pins 8-15 on the set pin
	pio_sm_set_pindirs_with_mask(pio0, sm, 0x30000, 0x3FF00);

	pio_sm_init(pio0, sm, offset, &c);
}

void setup_mcu_databus() {
	for(int i = 8; i < 16; i++) {
		pio_gpio_init(pio0, i);
	}

	pio_gpio_init(pio0, MCU_DATABUS_DEVICE_SIGNAL_PIN);
	pio_gpio_init(pio0, MCU_DATABUS_DEVICE_WRITE_PIN);

	setup_mcu_databus_read();
	setup_mcu_databus_write();

	pio_sm_set_enabled(pio0, 0, true);
	pio_sm_set_enabled(pio0, 1, true);
}

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
	printf("MCU1- Init pins...\n");

	// init the control pins (7 pins)
	for (int i = 0; i < 7; i++) {
		gpio_init(i);
		gpio_set_dir(i, false); // set to input
	}

	gpio_init(MCU1_PIN_IORDY);
	gpio_set_dir(MCU1_PIN_IORDY, true);

	gpio_init(MCU_DATABUS_DEVICE_SIGNAL_PIN);
	gpio_set_dir(MCU_DATABUS_DEVICE_SIGNAL_PIN, true);

	gpio_init(MCU_DATABUS_DEVICE_WRITE_PIN);
	gpio_set_dir(MCU_DATABUS_DEVICE_WRITE_PIN, true);

	// Init the pio programs to read/write mcu databus
	setup_mcu_databus();

	multicore_launch_core1(second_core_main);

	volatile uint32_t pins = 0;

	printf("Setting up register map...");

	// This is used to quickly get the right register, read/write should be handled by whatever is doing the lookup
	// Register Index = Bits = W, R, CS1, CS0, A2, A1, A0 (most->least)
	registerIndex_map[0x4E] = &SPI_registers[SPI_STATUS_REGISTER_INDEX];//&SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]; // read
	registerIndex_map[0x2E] = &SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]; // write

	registerIndex_map[0x50] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // read
	registerIndex_map[0x30] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // write

	registerIndex_map[0x31] = &SPI_registers[SPI_FEATURES_REGISTER_INDEX]; // read
	registerIndex_map[0x51] = &SPI_registers[SPI_ERROR_REGISTER_INDEX]; // write

	registerIndex_map[0x52] = &SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX]; // read only
	registerIndex_map[0x53] = &SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX]; // read only

	registerIndex_map[0x54] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]; // read
	registerIndex_map[0x34] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]; // write

	registerIndex_map[0x55] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]; // read
	registerIndex_map[0x35] = &SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]; // write

	registerIndex_map[0x56] = &SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]; // read
	registerIndex_map[0x36] = &SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]; // write

	registerIndex_map[0x57] = &SPI_registers[SPI_STATUS_REGISTER_INDEX]; // read
	registerIndex_map[0x37] = &SPI_registers[SPI_COMMAND_REGISTER_INDEX]; // write

	// misc registers, these aren't in the gdrom doc but ARE ata registers
	registerIndex_map[0x32] = &SPI_registers[SPI_SECTOR_COUNT_REGISTER_INDEX]; // write

	// Setup a pointer to the status register
	status_register = &SPI_registers[SPI_STATUS_REGISTER_INDEX];

	SPI_registers[SPI_STATUS_REGISTER_INDEX] = 0x40; // set drive ready
	SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX] = 0xA0; // set drive select

	// For the rest of the values, just use a dump register
	for(int i = 0; i < 128; i++) {
		if (registerIndex_map[i] == 0) {
			registerIndex_map[i] = &SPI_registers[SPI_REGISTER_COUNT]; // Use the SPI_REGISTER_COUNT as a dump register
		}
	}
	
	printf("Dreamcast booting...\n");

	// The Dreamcast(something?) does a startup with the cd drive and it toggles all the control, read, and write lines.
	// Wait for that to be finished before we start out programs
	// while(1) {
	// 	pins = gpio_get_all();

	// 	// Was off and is now on
	// 	if(last_csMask == 0 && ((pins & cs_mask) == cs_mask)) {
	// 		break;
	// 	}

	// 	last_csMask = (pins & cs_mask);
	// }

	// volatile uint32_t testValue = 0;
	// while(1) {
	// 	// pio_sm_put_blocking(pio0, MCU1_DATABUS_WRITE_SM, 1); // signal pio we are writing to the bus	
	// 	// pio_sm_put_blocking(pio0, MCU1_DATABUS_WRITE_SM, testValue); // send register data to dreamcsat

	// 	// busy_wait_us(10);

	// 	pio_sm_put_blocking(pio0, MCU1_DATABUS_READ_SM, 0); // Signal pio we are reading bus
	// 	testValue = pio_sm_get_blocking(pio0, MCU1_DATABUS_READ_SM); // read 16bits

	// 	busy_wait_us(10);
	// }

	while(!gpio_get(MCU1_PIN_CS0)); // loop until the cs lines are active (really only useful when powering the board on before the console)

	printf("Dreamcast booted!\n");
	busy_wait_ms(1000); // TODO this is likely not needed? 

	volatile uint32_t readWriteLineValues = 0;
	// volatile uint32_t rawLineValues = 0;
	selectedRegister = registerIndex_map[SPI_REGISTER_COUNT];
	volatile uint8_t dreamcastWantsRead = 0;
	// 00 (0x0) - nothing
	// 01 (0x1) - read
	// 10 (0x2) - write
	// 11 (0x3) - nothing

	gpio_put(MCU1_PIN_IORDY, 0);

	while(1) {
		
		// Worst case loop is 172ns
		// Read/write are low for ~300ns
		// This leaves us with 128ns (32cycles @ 4ns)
		// TODO do we need to do anything with the register data???

		// Don't do anything until the control lines are ready
		do {
			readWriteLineValues = sio_hw->gpio_in & CS_PINS_MASK;								// 16ns (4 cycles)
		} while(readWriteLineValues == CS_PINS_MASK || readWriteLineValues == 0x00000);			// 12ns (3 cycles)

		register_index = (sio_hw->gpio_in & REGISTER_PIN_MASK);									// 16ns (4 cycles)

		// Signal is active low
		// 1 and 2 are the only valid value. Either read OR write is low, but not both high or both low
		do {
			readWriteLineValues = sio_hw->gpio_in & READ_WRITE_PIN_MASK;						// 16ns (4 cycles)
		} while(readWriteLineValues == READ_WRITE_PIN_MASK || readWriteLineValues == 0x00000);	// 12ns (3 cycles)

		// bit shift in read/write values to the control line values
		register_index = (register_index | readWriteLineValues);  	// 12ns (3 cycles)

		// get the pointer to the selected register
		selectedRegister = registerIndex_map[register_index]; 									// 24ns (6 cycles)
		writtenRegisters[writtenRegisterIndex++] = register_index;

		// Write data to dreamcast
		if (readWriteLineValues == READ_PIN_MASK) {				
			// pio_sm_put_blocking(pio0, MCU1_DATABUS_WRITE_SM, 1); // signal pio we are writing to the bus	
			pio0->txf[MCU1_DATABUS_WRITE_SM] = 1;
			// pio_sm_put_blocking(pio0, MCU1_DATABUS_WRITE_SM, *selectedRegister); // send register data to dreamcsat
			pio0->txf[MCU1_DATABUS_WRITE_SM] = swap8(*selectedRegister);

			writtenRegisters[writtenRegisterIndex++] = 0xAAAAAAAA;
			
			gpio_put(MCU1_PIN_IORDY, 1);
			// wait for latch?
			while(gpio_get(MCU1_PIN_READ) == 0) { tight_loop_contents(); };						// 24ns (6 cycles)

			gpio_put(MCU1_PIN_IORDY, 0);

		// Read data from dreamcast into register
		} else {
			// pio_sm_put_blocking(pio0, MCU1_DATABUS_READ_SM, 0); // Signal pio we are reading bus
			pio0->txf[MCU1_DATABUS_READ_SM] = 0;
			pins = pio_sm_get_blocking(pio0, MCU1_DATABUS_READ_SM); // read 16bits
			*selectedRegister = pins; // save the read value to the register
			
			// debug
			writtenRegisters[writtenRegisterIndex++] = pins;//((uint16_t)pins);
			
			multicore_fifo_push_blocking(register_index);										// 24ns (6 cycles)

			gpio_put(MCU1_PIN_IORDY, 1);
			// wait for latch?
			while(gpio_get(MCU1_PIN_WRITE) == 0) { tight_loop_contents(); };					// 24ns (6 cycles)

			gpio_put(MCU1_PIN_IORDY, 0);
		}

		// (this point takes about 172ns from the beginning of the do loop)
		// This means that there is about 128ns extra time to do something before the next read/write cycle
		// This is 32 cycles at 4ns per cycle

	}

	
	return 0;
}

uint32_t timetrack = 0;
bool hasChirped = false;
volatile uint32_t core0CData = 0;

void second_core_main() {
	printf("Core1 Online\n");
	while(1) {
		
		if(time_us_32() - timetrack > 18000000 && !hasChirped) {
			hasChirped = true;
			timetrack = time_us_32();
			printf("----------------------------------------\n");
			printf("Num Writes: %d\n", writtenRegisterIndex);
			printf("Written Registers:\n");
			int goodWrites = 0;
			// if (writtenRegisterIndex < 20) {
				for(int i = 0; i < writtenRegisterIndex; i++) {
					int codedRegisterIndex = registerIndexFromControlValue(writtenRegisters[i]);
					// if (codedRegisterIndex == SPI_REGISTER_COUNT) {
					// 	continue;
					// }
					// goodWrites++;
					printf("%d: ", i);
					printf("%x = ", writtenRegisters[i]);
					printNameOfRegister(codedRegisterIndex);
					printf("\n");

					// Dont print more than 100 in case the dreamcast gets stuck polling the alt status register
					if(i > 100) {
						break;
					}
				}
			// }
			// printf("Bad values: %d\n", writtenRegisterIndex - goodWrites);
			printf("----------------------------------------\n");
			printf("/n/n");
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			// printf("alt status: %x\n", SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]);
			// printf("device control: %x\n",SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]);
			// printf("data: %x\n",SPI_registers[SPI_DATA_REGISTER_INDEX]); 
			// printf("features: %x\n",SPI_registers[SPI_FEATURES_REGISTER_INDEX]);
			// printf("error: %x\n",SPI_registers[SPI_ERROR_REGISTER_INDEX]);
			// printf("interrupt: %x\n",SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX]);
			// printf("sector number: %x\n",SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX]);
			// printf("byte count low: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]); 
			// printf("byte count high: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]);
			// printf("drive select: %x\n",SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]);
			// printf("status: %x\n",SPI_registers[SPI_STATUS_REGISTER_INDEX]);
			// printf("cmd: %x\n",SPI_registers[SPI_COMMAND_REGISTER_INDEX]);
			// printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

			for(int i = 0; i < SPI_REGISTER_COUNT; i++) {
				printNameOfRegister(i);
				printf(" = %x\n", SPI_registers[i]);
			}

			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

		}

		// Wait for data from core1 to be available
		if(!multicore_fifo_rvalid()) {
			continue;
		}

		core0CData = multicore_fifo_pop_blocking();

		// writtenRegisters[writtenRegisterIndex++] = core0CData;
		

		// register_index -> selected register 
		// selectedRegister -> register pointer
		// The command register is the only one that needs to be processed (for now)
		if(core0CData == 0x37) {
			// !!!BSY bit must be set within 400ns, so if we need more time, this bit should be set
			// .. update status register
			// *status_register = 0x80; // BSY bit set

			// printf("cmd: %x\n", *selectedRegister);
			// // .. for debugging maybe print out all the registers
			// printf("alt status: %x\n", SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]);
			// printf("device control: %x\n",SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]);
			// printf("data: %x\n",SPI_registers[SPI_DATA_REGISTER_INDEX]); 
			// printf("features: %x\n",SPI_registers[SPI_FEATURES_REGISTER_INDEX]);
			// printf("error: %x\n",SPI_registers[SPI_ERROR_REGISTER_INDEX]);
			// printf("interrupt: %x\n",SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX]);
			// printf("sector number: %x\n",SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX]);
			// printf("byte count low: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]); 
			// printf("byte count high: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]);
			// printf("drive select: %x\n",SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]);
			// printf("status: %x\n",SPI_registers[SPI_STATUS_REGISTER_INDEX]);
			// printf("cmd: %x\n",SPI_registers[SPI_COMMAND_REGISTER_INDEX]);

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
			//...... un
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
