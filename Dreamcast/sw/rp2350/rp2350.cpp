/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/systick.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "rp2350_pins.h"
#include "hardware/pio.h"
#include "sega_packet_interface.h"

#include "ide_handling.pio.h"

#include "gdrom.h"

#include "sd_utils.h"

#include "gdrom_utils.h"

// #include "ff.h" /* Obtains integer types */
// #include "diskio.h" /* Declarations of disk functions */
// #include "f_util.h"

// List of important coded control line values for the register map
#define CODED_DATA_REGISTER_READ (0x50) // dreamcast is reading from the data register
#define CODED_DATA_REGISTER_WRITE (0x30) // dreacast is writing to the data register
#define CODED_STATUS_REGISTER_READ (0x57)
#define CODED_COMMAND_REGISTER_WRITE (0x37)

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

// IRQ does seem to be asserted high which is in contrast to the CS0, CS1, RD, and WR signals.
#define INTRQ_ASSERT 	(1)
#define INTRQ_DEASSERT 	(0)

void second_core_main();

// Map values to commands, start with all values loaded to invalid (register count)
uint16_t* registerIndex_map[128] = {0};
volatile uint16_t* status_register = 0;
volatile uint16_t* selectedRegister = 0;
volatile uint32_t register_index = SPI_REGISTER_COUNT;

volatile uint32_t writtenRegisters[1005] = {0};
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

#define IDE_WRITE_TO_HOST_SM 1 // read from register and write to dreamcast (this is a read pin low)
#define IDE_READ_FROM_HOST_SM 0 // write to register from dreamcast (this is a write pin low)

void setup_read_from_dreamcast() {
	uint sm = IDE_READ_FROM_HOST_SM;
	uint offset = pio_add_program(pio0, &mcu1_read_from_dreamcast_program);
	pio_sm_config c = mcu1_read_from_dreamcast_program_get_default_config(offset);

	sm_config_set_in_pins(&c, 0);
	sm_config_set_out_pins(&c, 0, 16);

	pio_sm_set_pindirs_with_mask(pio0, sm, 0x0000, 0xFFFF);

	sm_config_set_in_shift(&c, false, false, 16);

	pio_sm_init(pio0, sm, offset, &c);
}

void setup_write_to_dreamcast() {
	uint sm = IDE_WRITE_TO_HOST_SM;
	uint offset = pio_add_program(pio0, &mcu1_write_to_dreamcast_program);
	pio_sm_config c = mcu1_write_to_dreamcast_program_get_default_config(offset);

	// Input pins start at pin 16
	sm_config_set_in_pins(&c, 16);

	// Output pins are 0-15, but this is the low byte so start at pin 0
	sm_config_set_out_pins(&c, 0, 16);
	
	// Still set the initial pins to be read
	pio_sm_set_pindirs_with_mask(pio0, sm, 0x0000, 0xFFFF);
	
	sm_config_set_in_shift(&c, false, false, 16);

	pio_sm_init(pio0, sm, offset, &c);
}

int main(void) {
	// Set clock speed to 266MHz (3.76ns per cycle)
	const int freq_khz = 266000;
	// const int freq_khz = 336000;
	// vreg_set_voltage(VREG_VOLTAGE_1_25); // Usually needed for clocks over 266MHz
	bool clockWasSet = set_sys_clock_khz(freq_khz, false);

	// stdio_init_all();
	// gpio 44 is uart0 tx
	stdio_uart_init_full(uart0, DEBUG_UART_BAUD_RATE, 44, -1);

	printf("Clock of %uMhz was set: %u\n", freq_khz / 1000, clockWasSet);
	printf("MCU1- Init pins...\n");

	// init all pins used on for the Dreamcast IDE connection
	// This is from pin 0 to pin 33 for a total of 34 pins
	for (int i = 0; i <= PIN_WR; i++) {
		gpio_init(i);
		gpio_set_dir(i, false); // set to input
	}

	// Some pins that need to be set to output
	// TODO: Figure out which pins need to be set to output
	gpio_init(PIN_IORDY);
	gpio_set_dir(PIN_IORDY, true);
	gpio_pull_up(PIN_IORDY);

	gpio_init(PIN_DOPEN);
	gpio_set_dir(PIN_DOPEN, true);
	gpio_pull_down(PIN_DOPEN);

	gpio_init(PIN_INTRQ);
	gpio_set_dir(PIN_INTRQ, true);
	gpio_put(PIN_INTRQ, INTRQ_DEASSERT); // was 0

	gpio_init(PIN_DMARQ);
	gpio_set_dir(PIN_DMARQ, true); // output

	gpio_init(PIN_DMACK);
	gpio_set_dir(PIN_DMACK, false); // input

	// Setup and start the ide databus programs
	printf("Setting up PIO ide databus programs...\n");
	printf("\tiniting gpio for pio...");
	for(int i = 0; i < 16; i++) {
		gpio_init(i);
		gpio_set_function(i, GPIO_FUNC_PIO0);
		pio_gpio_init(pio0, i);
	}
	printf("DONE!\n\tSetting up programs...");
	setup_read_from_dreamcast();
	setup_write_to_dreamcast();
	printf("DONE!\n\tEnabling programs...");
	pio_sm_set_enabled(pio0, IDE_READ_FROM_HOST_SM, true);
	pio_sm_set_enabled(pio0, IDE_WRITE_TO_HOST_SM, true);
	printf("DONE!\n");

	multicore_launch_core1(second_core_main);

	volatile uint32_t pins = 0;

	printf("Setting up register map...");

	// This is used to quickly get the right register, read/write should be handled by whatever is doing the lookup
	// Register Index = Bits = W, R, CS1, CS0, A2, A1, A0 (most->least)
	registerIndex_map[0x4E] = &SPI_registers[SPI_STATUS_REGISTER_INDEX];//&SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]; // read
	registerIndex_map[0x2E] = &SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]; // write

	registerIndex_map[0x50] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // read
	registerIndex_map[0x30] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // write

	registerIndex_map[0x31] = &SPI_registers[SPI_FEATURES_REGISTER_INDEX]; // write
	registerIndex_map[0x51] = &SPI_registers[SPI_ERROR_REGISTER_INDEX]; // read

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

	SPI_registers[SPI_STATUS_REGISTER_INDEX] = 0b01000000; // set drive ready bit
	SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX] = 0xA0; // set drive select
	SPI_registers[SPI_SECTOR_COUNT_REGISTER_INDEX] = 0b01;
	SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x80; // 4 if apparently this is the code for a "data disc" and it occupies the top 4 bits of the byte

	// For the rest of the values, just use a dump register
	for(int i = 0; i < 128; i++) {
		if (registerIndex_map[i] == 0) {
			registerIndex_map[i] = &SPI_registers[SPI_REGISTER_COUNT]; // Use the SPI_REGISTER_COUNT as a dump register
		}
	}
	
	printf("Dreamcast booting...\n");

	// The Dreamcast(something?) does a startup with the cd drive and it toggles all the control, read, and write lines.
	// Not sure if this is real data or just a fun little startup sequence

	while(!gpio_get(PIN_CS0)); // loop until the cs lines are active (really only useful when powering the board on before the console)

	printf("Dreamcast booted!\n");

	busy_wait_ms(1000); // TODO this is likely not needed? 

	volatile uint32_t readWriteLineValues = 0;
	selectedRegister = registerIndex_map[SPI_REGISTER_COUNT];
	volatile uint8_t dreamcastWantsRead = 0;
	// 00 (0x0) - nothing
	// 01 (0x1) - read
	// 10 (0x2) - write
	// 11 (0x3) - nothing

	// By using the IORDY pin, we can slow down the control signaling.
	// This makes it a lot slower to read/write to the dreamcast but does at least work
	// TODO: Figure out how to do this without the IORDY pin to speed up the control processing
	gpio_put(PIN_IORDY, 0);

	while(1) {
		do {
			readWriteLineValues = sio_hw->gpio_in & CS_PINS_MASK;								// 16ns (4 cycles)
		} while(readWriteLineValues == CS_PINS_MASK || readWriteLineValues == 0x00000);			// 12ns (3 cycles)

		do {
			readWriteLineValues = sio_hw->gpio_in & READ_WRITE_PIN_MASK;
		} while(readWriteLineValues == READ_WRITE_PIN_MASK || readWriteLineValues == 0x00000);

		// Figure out the register index and get the pointer to the selected register
		register_index = (sio_hw->gpio_in & REGISTER_PIN_MASK) >> 16; // shift by 16 to offset the data pins (0-15)
		selectedRegister = registerIndex_map[register_index];

		if(writtenRegisterIndex < 1000) {
			writtenRegisters[writtenRegisterIndex++] = register_index;
		}

		// Read from register send to Dreamcast
		if ((register_index & BIT_SHIFTED_READ_PIN_MASK) == BIT_SHIFTED_READ_PIN_MASK) {
			pio0->txf[IDE_WRITE_TO_HOST_SM] = *selectedRegister;

			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0xAAAA;
				writtenRegisters[writtenRegisterIndex++] = *selectedRegister;
			}

			multicore_fifo_push_blocking(register_index);

			gpio_put(PIN_IORDY, 1);
			// wait for latch
			while(gpio_get(PIN_RD) == 0) { tight_loop_contents(); };
			
			// Send a signal to pio to let it go back to input
			pio0->txf[IDE_WRITE_TO_HOST_SM] = 0;
			
			gpio_put(PIN_IORDY, 0);

		// Write to Dreamcast from register
		} else {
			// Let pio know we are ready to read data
			pio0->txf[IDE_READ_FROM_HOST_SM] = 1;
			*selectedRegister = pio_sm_get_blocking(pio0, IDE_READ_FROM_HOST_SM);

			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0xBBBB;
				writtenRegisters[writtenRegisterIndex++] = *selectedRegister;
			}

			multicore_fifo_push_blocking(register_index);

			gpio_put(PIN_IORDY, 1);
			// wait for latch
			while(gpio_get(PIN_WR) == 0) { tight_loop_contents(); };
			gpio_put(PIN_IORDY, 0);
		}
	}

	return 0;
}

uint32_t timetrack = 0;
bool hasChirped = false;
volatile uint32_t core0CData = 0;
volatile uint16_t core0commandRegister = 0;
volatile uint16_t* spi_packet_register = 0;
volatile uint8_t spi_packet_word_count = 0;

#define DATA_MODE_IDLE 		(0)
#define DATA_MODE_SPI 		(1) // Sega SPI packet mode, processing their 12 byte packets
#define IDE_TRANSFER_MODE_PIO (0)
#define IDE_TRANSFER_MODE_DMA (1)
static uint8_t ide_current_mode = DATA_MODE_IDLE;
static uint8_t ide_current_transfer_mode = 0; // 0 = PIO, 1 = DMA

#define IO_MODE_IDLE 0
#define IO_MODE_WRITE 1
#define IO_MODE_READ 2
static uint8_t current_io_mode = 0;
// In other code bases, this is part of the state variable for data mode, but
// knowing which packet command is being processed is fine for now.
// There may be a point when we need to change to a better state machine.
// But that will likely require rethinking the whole flow.
static uint8_t current_io_packet_command = 0; // This is just the current SPI packet command we are processing, useful for sending canned responses vs actual data

// Bake the offset into the starting and ending positions
static uint32_t io_current_position = 0;
static uint32_t io_ending_position = 0;

void process_packet() {
	current_io_packet_command = SEGA_PACKET_CMD_REGISTER[0];
	ide_current_mode = DATA_MODE_IDLE;
	// Get and set transfer mode from the features register
	ide_current_transfer_mode = SPI_registers[SPI_FEATURES_REGISTER_INDEX] & 1;

	switch(current_io_packet_command) {
		case 0x70:
		case TEST_UNIT_SEGA_PACKET_CMD: {
			*status_register = 0x50; // only drive ready bit set
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
			SPI_registers[SPI_ERROR_REGISTER_INDEX] = 0x00; // no error
			SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x82;

			// Set interrupt bit and assert line
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);
			break;
		}
		case REQ_STAT_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x11111111;
			}
			break;
		}
		case REQ_MODE_SEGA_PACKET_CMD: {
			uint startingAddress = SEGA_PACKET_CMD_REGISTER[2];
			uint length = SEGA_PACKET_CMD_REGISTER[4];	

			// TODO icegdrom does this with 3 specific cases but nulldc just has a whole canned response array
			// and none of this specific address checking. Might be worth condensing this without any magic numbers
			if (startingAddress == 18 && length == 8) {
				uint halfLength = length/2; // Divide by two because we are sending 2 bytes at a time (16bit bus)
				
				// Setup the IO infos
				current_io_mode = IO_MODE_WRITE;
				io_current_position = startingAddress / 2;
				io_ending_position = (startingAddress / 2) + (halfLength);
				ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;
				
				// put first word in data register
				SPI_registers[SPI_DATA_REGISTER_INDEX] = swap8(reply_11[io_current_position++]);

				// Put the correct values in the registers
				SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
				SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0;
				SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = 8; // length is 8 bytes
				*status_register = 0x58; // DRQ = 1 BSY = 0 

				// set irq
				gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			} else if (startingAddress == 0 && length == 10) {
				if(writtenRegisterIndex < 1000) {
					writtenRegisters[writtenRegisterIndex++] = 0xDEADBEEF;
				}

			} else {
				// finish packet by setting an error
				if(writtenRegisterIndex < 1000) {
					writtenRegisters[writtenRegisterIndex++] = 0xDEADDEAD;
				}
			}

			break;
		}
		case SET_MODE_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x22222222;
			}
			break;
		}
		case REQ_ERROR_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x33333333;
			}
			break;
		}
		case GET_TOC_SEGA_PACKET_CMD: {
			// TOC is ALWAYS 408 bytes
			current_io_mode = IO_MODE_WRITE;
			io_current_position = 0;
			io_ending_position = 204; // 408 bytes / 2 bytes per word
			ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;

			DiskArea tocSelectBit = (SEGA_PACKET_CMD_REGISTER[1] & 0x1) ? DoubleDensity : SingleDensity;
			GetDriveToc((uint32_t*)(SEGA_PACKET_TOC_INFO), tocSelectBit);

			// put first word in data register
			SPI_registers[SPI_DATA_REGISTER_INDEX] = SEGA_PACKET_TOC_INFO_16[io_current_position++];

			// Put the correct values in the registers
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			 // length is 408 bytes which is 0x198 in Hex, so just magic number it
			SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0x1;
			SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = 0x98;
			*status_register = 0x58; // DRQ = 1 BSY = 0 

			// set irq
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);
			
			break;
		}
		case REQ_SES_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x55555555;
			}
			break;
		}
		case CD_OPEN_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x66666666;
			}
			break;
		}
		case CD_PLAY_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x77777777;
			}
			break;
		}
		case CD_SEEK_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x88888888;
			}
			break;
		}
		case CD_SCAN_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x99999999;
			}
			break;
		}
		case CD_READ_SEGA_PACKET_CMD: {
			current_io_mode = IO_MODE_WRITE;
			
			// Read and buffer the data
			gdrom_read_start(SEGA_PACKET_CMD_REGISTER, ide_current_transfer_mode);
			printf("current_io_mode: %u\n", current_io_mode);

			if (ide_current_transfer_mode == IDE_TRANSFER_MODE_PIO) {
				// Read the first word
				gdrom_read_consume_buffer(&SPI_registers[SPI_DATA_REGISTER_INDEX]);

				// Put the correct values in the registers
				SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
				SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = gdrom_read_bytes_remanining >> 8;
				SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = gdrom_read_bytes_remanining & 0xFF;
				*status_register = 0x58; // DRQ = 1 BSY = 0 

				// set irq
				gpio_put(PIN_INTRQ, INTRQ_ASSERT);
			} else {
				// Fall through to main loop and start the DMA transfer
				if(writtenRegisterIndex < 1000) {
					writtenRegisters[writtenRegisterIndex++] = 0xD8A10000;
				}
			}	

			break;
		}
		case CD_READ2_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x1234BBBB;
			}
			break;
		}
		case GET_SCD_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0xCCCCCCCC;
			}
			break;
		}
		case Code71_PACKET_CMD: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0x00000071;
			}
			break;
		}
		default: {
			if(writtenRegisterIndex < 1000) {
				writtenRegisters[writtenRegisterIndex++] = 0xDEAD0000 | current_io_packet_command;
			}
			break;
		}
	}
}

static inline void process_data_written() {
	// I think this method will ONLY be called when dreamcast is writing a SPI packet
	if(ide_current_mode == DATA_MODE_SPI) {
		
		spi_packet_register[spi_packet_word_count++] = SPI_registers[SPI_DATA_REGISTER_INDEX];

		// This is the last word of the packet, process it
		if (spi_packet_word_count >= 6) {
			ide_current_mode = DATA_MODE_IDLE;
			spi_packet_word_count = 0;

			process_packet();
		}
	} else {
		// So this else block is likely unneeded but will keep this here for debugging purposes
		if(writtenRegisterIndex < 1000) {
			writtenRegisters[writtenRegisterIndex++] = 0xEEEEEEEE;
		}
		// TODO might be better to just printf something instead?
	}
}

static uint32_t dmaTransfersCompleted = 0;
static uint8_t dmaDidTimeout = 0;
static uint32_t numDmackWaits = 0; // timeout
static bool gdrom_buffer_has_more_data = 0;
static inline void process_data_read() {
	// TODO
	// Dreamcast has read the data register. Put the next word in the register

	if(current_io_packet_command == REQ_MODE_SEGA_PACKET_CMD) {
		SPI_registers[SPI_DATA_REGISTER_INDEX] = swap8(reply_11[io_current_position++]); // put next word in data register

		if (io_current_position >= io_ending_position) {
			current_io_packet_command = 0;
			io_current_position = 0;
			io_ending_position = 0;
			current_io_mode = IO_MODE_IDLE;

			// Set the status register to indicate the data is finished
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
			*status_register = 0x50; // DRQ = 0 BSY = 0
			
			// Assert the IRQ line to announce we are finished
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);
		}
	} else if (current_io_packet_command == GET_TOC_SEGA_PACKET_CMD) {
		// TODO error handling
		// If the disc image we read doesn't have any TOC we should set error in status register

		// Start sending the TOC
		SPI_registers[SPI_DATA_REGISTER_INDEX] = swap8(SEGA_PACKET_TOC_INFO_16[io_current_position++]);
		if (io_current_position >= io_ending_position) {
			current_io_packet_command = 0;
			io_current_position = 0;
			io_ending_position = 0;
			current_io_mode = IO_MODE_IDLE;

			// Set the status register to indicate the data is finished
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
			*status_register = 0x50; // DRQ = 0 BSY = 0
			
			// Assert the IRQ line to announce we are finished
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);
		}

	} else if (current_io_packet_command == CD_READ_SEGA_PACKET_CMD) { 
		// read from the sd card and send data from the gdrom_read_buffer
		if (ide_current_transfer_mode == IDE_TRANSFER_MODE_DMA) {
			/*
			 *
			 * DMA Data Transfer Handshake in ATA/ATAPI
			 * 	The two key signals for DMA transfers are:

			 * 	*	DMARQ (DMA Request): Asserted by the device (GD-ROM) when it has data ready to transfer.
			 * 	*	DMACK (DMA Acknowledge): Asserted by the host (Dreamcast) when it is ready to receive data.
			 * 	This forms a handshake mechanism where:

			 * 	1.	GD-ROM sets DMARQ high when it has a word (16-bit) of data ready.
			 * 	2.	Dreamcast sets DMACK low when it is ready to accept data.
			 * 	3.	GD-ROM places data on the bus and waits for the host to acknowledge.
			 * 	4.	Dreamcast deasserts DMACK, signaling the drive to send the next word.
			 * 	5.	Steps 1-4 repeat for each word until the entire transfer is complete.
			 */
			// IMPORTANT: So the approach here is to tie up this core with the DMA transfer.
			// And presumably nothing is happening at the register level because there will
			// be no one to response to commands since this thread wont be popping the core fifo
			// IF this is a problem, we should just work the dma transfer into the main loop

			// if(writtenRegisterIndex < 1000) {
			// 	writtenRegisters[writtenRegisterIndex++] = 0xD8A00000;
			// }

			// do {
				gdrom_buffer_has_more_data = gdrom_read_consume_buffer(&SPI_registers[SPI_DATA_REGISTER_INDEX]); // read in another word
				

				// Put data on the bus
				pio0->txf[IDE_WRITE_TO_HOST_SM] = swap8(SPI_registers[SPI_DATA_REGISTER_INDEX]);

				// Signal we have data and wait for dreamcast to acknowledge
				gpio_put(PIN_DMARQ, 1);
				while(gpio_get(PIN_DMACK) == 1 && numDmackWaits < 10) { 
					numDmackWaits++;
					tight_loop_contents(); 
				}
				if (numDmackWaits >= 10) {
					dmaDidTimeout = 1;
				}
				numDmackWaits = 0;

				// Dreamcast has acknowledged, send the data, and wait for dreamcast to be ready for the next word
				gpio_put(PIN_DMARQ, 0);
				while(gpio_get(PIN_DMACK) == 0) { tight_loop_contents(); }

				// reset databus for next word
				pio0->txf[IDE_WRITE_TO_HOST_SM] = 0;

				dmaTransfersCompleted++;

			// } while(gdrom_buffer_has_more_data);

			// if(writtenRegisterIndex < 1000) {
			// 	writtenRegisters[writtenRegisterIndex++] = 0xD8A01111;
			// }

			if (dmaDidTimeout) {
				current_io_mode = IO_MODE_IDLE;
				// Set the status register to indicate the data is finished
				SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
				*status_register = 0x50; // DRQ = 0 BSY = 0

				// Do we need to assert the irq line for a read?
				gpio_put(PIN_INTRQ, INTRQ_ASSERT);

				if(writtenRegisterIndex < 1000) {
					writtenRegisters[writtenRegisterIndex++] = dmaTransfersCompleted;
					writtenRegisters[writtenRegisterIndex++] = 0xD8A07777;
					dmaTransfersCompleted = 0;
				}

				return;
			}
			
			if (!gdrom_buffer_has_more_data) {
				// All the data has been sent, signal the end of the transfer
				current_io_mode = IO_MODE_IDLE;
				// Set the status register to indicate the data is finished
				SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
				*status_register = 0x50; // DRQ = 0 BSY = 0

				// Do we need to assert the irq line for a read?
				gpio_put(PIN_INTRQ, INTRQ_ASSERT);

				if(writtenRegisterIndex < 1000) {
					writtenRegisters[writtenRegisterIndex++] = 0xD8A01111;
				}
			}

		// PIO
		} else {
			gdrom_buffer_has_more_data = gdrom_read_consume_buffer(&SPI_registers[SPI_DATA_REGISTER_INDEX]); // read in another word

			if (!gdrom_buffer_has_more_data) {
				current_io_mode = IO_MODE_IDLE;
				// Set the status register to indicate the data is finished
				SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
				*status_register = 0x50; // DRQ = 0 BSY = 0
				
				// Do we need to assert the irq line for a read?
				gpio_put(PIN_INTRQ, INTRQ_ASSERT);
			}
		}	

	} else {
		printf("\nunimplemented current_io_packet: %u\n", current_io_packet_command);
	}

}

void second_core_main() {
	printf("Core1 Online\n");

	printf("Opening default disc image...\n");
	gdrom_read_default_disc_image();
	printf("DONE!\n");

	spi_packet_register = (uint16_t*)(&SEGA_PACKET_CMD_REGISTER);

	while(1) {
		
		if(time_us_32() - timetrack > 60000000/*18000000*/ && !hasChirped) {
			hasChirped = true;
			timetrack = time_us_32();
			printf("----------------------------------------\n");
			printf("Num Writes: %d\n", writtenRegisterIndex);
			printf("Written Registers:\n");
			int goodWrites = 0;

			// uint32_t v0;
			// uint32_t v1;
			// uint32_t v2;
			// uint32_t loopUntilIndex = writtenRegisterIndex;
			// if (loopUntilIndex > 1500) {
			// 	loopUntilIndex = 1500;
			// }
			// uint8_t isPrintingCommandPacket = 0; // Revist to format command packets
			for(uint32_t i = 0; i < writtenRegisterIndex; i++) {

				// v0 = writtenRegisters[i];
				// v1 = writtenRegisters[i+1];
				// v2 = writtenRegisters[i+2];

				// if (v0 == 0xaaaa || v0 == 0xbbbb) {
				// 	printf("%x\t| %s |\t%x\n", writtenRegisters[i-1], ((v0 == 0xaaaa) ? "READ " : "WRITE"), writtenRegisters[i+1]);
				// 	i += 2;
				// } else {
				printf("%x\n", writtenRegisters[i]);
				// 	i++;
				// }
			}

			// for(int i = 0; i < SPI_REGISTER_COUNT; i++) {
			// 	printNameOfRegister(i);
			// 	printf(" = %x\n", SPI_registers[i]);
			// }

			printf("----------------------------------------\n");
			printf("/n/n");
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			printf("alt status: %x\n", SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]);
			printf("device control: %x\n",SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]);
			printf("data: %x\n",SPI_registers[SPI_DATA_REGISTER_INDEX]); 
			printf("features: %x\n",SPI_registers[SPI_FEATURES_REGISTER_INDEX]);
			printf("error: %x\n",SPI_registers[SPI_ERROR_REGISTER_INDEX]);
			printf("interrupt: %x\n",SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX]);
			printf("sector number: %x\n",SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX]);
			printf("byte count low: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]); 
			printf("byte count high: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]);
			printf("drive select: %x\n",SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]);
			printf("status: %x\n",SPI_registers[SPI_STATUS_REGISTER_INDEX]);
			printf("cmd: %x\n",SPI_registers[SPI_COMMAND_REGISTER_INDEX]);
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");


			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

		}

		// If we are sending data and in DMA mode
		if(ide_current_transfer_mode && current_io_mode == IO_MODE_WRITE) {
			process_data_read();
		}

		// Wait for data from core1 to be available
		if(!multicore_fifo_rvalid()) {
			continue;
		}

		// Get data
		core0CData = multicore_fifo_pop_blocking();

		// Host has read the status register
		if (core0CData == CODED_STATUS_REGISTER_READ) {
			gpio_put(PIN_INTRQ, INTRQ_DEASSERT); // negate the interrupt line
			continue;
		}

		if (core0CData == CODED_DATA_REGISTER_WRITE) {
			process_data_written();
			continue;
		}

		if (core0CData == CODED_DATA_REGISTER_READ) {
			if (current_io_mode == IO_MODE_WRITE) {
				process_data_read();
			}

			continue;
		}

		// register_index -> selected register 
		// selectedRegister -> register pointer
		// The command register is the only one that needs to be processed (for now)
		if(core0CData == CODED_COMMAND_REGISTER_WRITE) {

			// !!!BSY bit must be set within 400ns, so if we need more time, this bit should be set
			// .. update status register
			*status_register = *status_register | 0x80; // BSY bit set

			// We need to check the command register
			core0commandRegister = SPI_registers[SPI_COMMAND_REGISTER_INDEX];

			// If this is the packet command, it's the most important case
			if (core0commandRegister == ATA_CMD_PACKET_COMMAND) {
				ide_current_mode = DATA_MODE_SPI;

				SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0b01; // IO, CoD 
				*status_register = 0x58; // set DRQ bit and clear the busy bit
			
			// Otherwise just do a switch case to process the command
			} else {
				switch (*selectedRegister) {
					case ATA_CMD_NOP:{
						// Command can be received when BSY bit is 1 
						// and device should terminate the command currently in execution
						// printf(".");
						break;
					}
					case ATA_CMD_SOFT_RESET: {
						break;
					}
					// case ATA_CMD_PACKET_COMMAND: { // now handled in it's own if block
					// 	// Process sega packet interface
					// 	// ...
					// 	break;
					// }
					case ATA_CMD_IDENTIFY_DEVICE: {
						break;
					}
					case ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC: {
						break;
					}
					case ATA_CMD_SET_FEATURES: {
						printf("SET_FEATURES\n");
						break;
					}

					
				}
				// IMPORANT this is really important, if something changes status_register, this busy bit might not be needed
				// for now, only clear it after commands that aren't packet command as they don't do anything.
				// Clear BSY bit
				*status_register = *status_register ^ 0x80; 
			}
		}
	}
}

// Store the test functions here so we can get around to organizing them later
void test_gdrom_read() {
	#ifdef DEBUG_TEST
	busy_wait_ms(500);

	// Test the read command
	SEGA_PACKET_CMD_REGISTER[0] = 0x30;
	SEGA_PACKET_CMD_REGISTER[1] = 0x24;
	SEGA_PACKET_CMD_REGISTER[2] = 0x00;
	SEGA_PACKET_CMD_REGISTER[3] = 0xb0;
	SEGA_PACKET_CMD_REGISTER[4] = 0x5e;
	SEGA_PACKET_CMD_REGISTER[5] = 0x00;
	SEGA_PACKET_CMD_REGISTER[6] = 0x00;
	SEGA_PACKET_CMD_REGISTER[7] = 0x00;
	SEGA_PACKET_CMD_REGISTER[8] = 0x00;
	SEGA_PACKET_CMD_REGISTER[9] = 0x00;
	SEGA_PACKET_CMD_REGISTER[10] = 0x07;
	SEGA_PACKET_CMD_REGISTER[11] = 0x00;

	uint32_t endTime = 0;
	uint32_t startTime = time_us_32();
	gdrom_read_start(SEGA_PACKET_CMD_REGISTER, 0);
	endTime = time_us_32();

	printf("Read Time: %u\n", endTime - startTime);

	printf("GDROM Read Test\n");

	printf("SPI Command Packet:\t");
	for(int i = 0; i < 12; i++) {
		printf("(%u)%x ", i, SEGA_PACKET_CMD_REGISTER[i]);
	}
	printf("\n");

	printf("Sector Start: %u(0x%x), Sector Count: %u(0x%x), Sector Size: %u(0x%x)\n", gdrom_read_start_sector, gdrom_read_start_sector, gdrom_read_remaining_sectors, gdrom_read_remaining_sectors, gdrom_read_sector_size, gdrom_read_sector_size);
	printf("Data Buffer:");
	for (int i = 0; i < 32; i++) {
		if (i % 8 == 0) {
			printf("\n%d: ", i);
		}
		printf("%x ", gdrom_read_buffer[i]);
	}

	printf("\nData Register:");
	for (int i = 0; i < 32; i++) {
		gdrom_read_consume_buffer(&SPI_registers[SPI_DATA_REGISTER_INDEX]); // read in another word
		if (i % 8 == 0) {
			printf("\n%d: ", i);
		}
		printf("%x ", SPI_registers[SPI_DATA_REGISTER_INDEX]);
	}
	printf("\n\n");

	// printf("Data_Buffer[0]: %x\n", gdrom_read_buffer[0]);
	// printf("Data Register:  %x\n", SPI_registers[SPI_DATA_REGISTER_INDEX]);

	// Stall here
	while(1);;;
	#endif
}

