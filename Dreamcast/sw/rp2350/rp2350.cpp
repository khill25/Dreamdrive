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
volatile uint8_t core_command_buffer[256];
volatile uint8_t core0_buffer_index = 0;
volatile uint8_t core1_buffer_index = 0;

// IRQ does seem to be asserted high which is in contrast to the CS0, CS1, RD, and WR signals.
#define INTRQ_ASSERT 	(1)
#define INTRQ_DEASSERT 	(0)

void main_processing_loop();
void ide_register_controller_main();

// Map values to commands, start with all values loaded to invalid (register count)
uint16_t* registerIndex_map[128] = {0};
volatile uint16_t* status_register = 0;
volatile uint16_t* selectedRegister = 0;
volatile uint32_t register_index = SPI_REGISTER_COUNT;

volatile uint32_t writtenRegisters[10000] = {0};
volatile uint32_t writtenRegisterIndex = 0;

volatile uint16_t* rom = 0;

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

#define DMA_HANDLER_SM (1)
#define ATA_BUS_HANDLER_SM (0)

uint ata_bus_handler_offset = 0;
uint dma_bus_handler_offset = 0;

uint dma_bus_handler_dma_channel = 0;
void setup_dma_bus_handler() {
	uint sm = DMA_HANDLER_SM;
	dma_bus_handler_offset = pio_add_program(pio0, &dma_bus_handler_program);
	pio_sm_config c = dma_bus_handler_program_get_default_config(dma_bus_handler_offset);

	// Output pins are 0-15, but this is the low byte so start at pin 0
	sm_config_set_out_pins(&c, 0, 16);
	
	// Still set the initial pins to be output
	pio_sm_set_pindirs_with_mask(pio0, sm, 0x2200FFFF, 0x2200FFFF);
	
	sm_config_set_out_shift(&c, false, false, 16); // not sure if we need to do this
	sm_config_set_in_shift(&c, false, false, 16);

	sm_config_set_set_pins(&c, PIN_DMARQ, 1);

	sm_config_set_sideset_pins(&c, PIN_CD_SDAT);

	pio_sm_init(pio0, sm, dma_bus_handler_offset, &c);

	// setup dma unit
	// DMA Channels
	dma_bus_handler_dma_channel = dma_claim_unused_channel(true);

	// DMA1: Read index from FIFO and store it
	dma_channel_config c1 = dma_channel_get_default_config(dma_bus_handler_dma_channel);
	channel_config_set_transfer_data_size(&c1, DMA_SIZE_16);
	channel_config_set_read_increment(&c1, true);
	channel_config_set_write_increment(&c1, false);  // Store index in a fixed variable
	channel_config_set_dreq(&c1, pio_get_dreq(pio0, sm, true));

	dma_channel_configure(
		dma_bus_handler_dma_channel,
		&c1,
		&pio0->txf[sm],		 // Push values to sm 
		gdrom_read_buffer,  // Read from gdrom buffer
		1,                   // Single transfer (index)
		false                // Do not start yet
	);
}

uint8_t dma_bus_started = 0;
uint8_t ata_bus_started = 0;

void setup_ata_bus_handler() {
	PIO pio = pio0;
	uint sm = ATA_BUS_HANDLER_SM;
	ata_bus_handler_offset = pio_add_program(pio, &ata_bus_handler_program);
	pio_sm_config c = ata_bus_handler_program_get_default_config(ata_bus_handler_offset);

	// Input pins start at pin 0
	sm_config_set_in_pins(&c, 0);

	// Output pins are 0-15, but this is the low byte so start at pin 0
	sm_config_set_out_pins(&c, 0, 16);
	
	// Still set the initial pins to be read
	pio_sm_set_pindirs_with_mask(pio, sm, 0x800000, 0xFFFFFF);
	sm_config_set_jmp_pin(&c, PIN_WR);
	
	sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_out_shift(&c, true, false, 32);

	sm_config_set_sideset_pins(&c, PIN_IORDY);

	pio_sm_init(pio, sm, ata_bus_handler_offset, &c);

	ata_bus_started = 1;
}

void start_dma_bus_handler() {
	if (ata_bus_started) {
		pio_sm_set_enabled(pio0, ATA_BUS_HANDLER_SM, false);
		pio_remove_program(pio0, &ata_bus_handler_program, ata_bus_handler_offset);
		ata_bus_started = 0;
	}

	setup_dma_bus_handler();
	pio_sm_set_enabled(pio0, DMA_HANDLER_SM, true);

	dma_bus_started = 1;
}

void start_ata_bus_handler() {
	if (dma_bus_started) {
		pio_sm_set_enabled(pio0, DMA_HANDLER_SM, false);
		pio_remove_program(pio0, &dma_bus_handler_program, dma_bus_handler_offset);
		dma_bus_started = 0;
	}

	setup_ata_bus_handler();
	pio_sm_set_enabled(pio0, ATA_BUS_HANDLER_SM, true);
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
	// gpio_set_dir(PIN_IORDY, true);
	// gpio_pull_up(PIN_IORDY);

	gpio_init(PIN_DOPEN);
	gpio_set_dir(PIN_DOPEN, true);
	// gpio_pull_down(PIN_DOPEN);

	gpio_init(PIN_INTRQ);
	gpio_set_dir(PIN_INTRQ, true);
	gpio_put(PIN_INTRQ, INTRQ_DEASSERT); // was 0

	gpio_init(PIN_DMARQ);
	gpio_set_dir(PIN_DMARQ, true); // output

	gpio_init(PIN_DMACK);
	gpio_set_dir(PIN_DMACK, false); // input
	// gpio_pull_up(PIN_DMACK);

	// Setup and start the ide databus programs
	printf("Setting up PIO ide databus programs...\n");
	printf("\tiniting gpio for pio...");
	for(int i = 0; i <= PIN_WR; i++) {
		gpio_init(i);
		gpio_set_function(i, GPIO_FUNC_PIO0);
		pio_gpio_init(pio0, i);
	}
	gpio_set_function(PIN_IORDY, GPIO_FUNC_PIO0);
	pio_gpio_init(pio0, PIN_IORDY);

	// gpio_init(PIN_CS0);
	// gpio_set_function(PIN_CS0, GPIO_FUNC_SIO);
	// gpio_set_dir(PIN_CS0, false);

	// gpio_init(PIN_CS1);
	// gpio_set_function(PIN_CS1, GPIO_FUNC_SIO);
	// gpio_set_dir(PIN_CS1, false);

	gpio_init(PIN_RD);
	gpio_set_function(PIN_RD, GPIO_FUNC_SIO);
	gpio_set_dir(PIN_RD, false);

	gpio_init(PIN_WR);
	gpio_set_function(PIN_WR, GPIO_FUNC_SIO);
	gpio_set_dir(PIN_WR, false);

	// Setup dmarq line so it can be used by the dma handler state machine
	gpio_init(PIN_DMARQ);
	gpio_set_function(PIN_DMARQ, GPIO_FUNC_PIO0);
	pio_gpio_init(pio0, PIN_DMARQ);

	gpio_init(PIN_CD_SDAT);
	gpio_set_function(PIN_CD_SDAT, GPIO_FUNC_PIO0);
	pio_gpio_init(pio0, PIN_CD_SDAT);
	
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
	SPI_registers[SPI_SECTOR_COUNT_REGISTER_INDEX] = 0b00;
	SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x80; // GDROM is 0x80

	// For the rest of the values, just use a dump register
	for(int i = 0; i < 128; i++) {
		if (registerIndex_map[i] == 0) {
			registerIndex_map[i] = &SPI_registers[SPI_REGISTER_COUNT]; // Use the SPI_REGISTER_COUNT as a dump register
		}
	}

	printf("DONE!\n\tSetting up programs...");
	setup_ata_bus_handler();

	// start_dma_bus_handler();
	// printf("Sending num transfers\n");
	// pio_sm_put_blocking(pio0, DMA_HANDLER_SM, 20);
	// dma_channel_set_trans_count(dma_bus_handler_dma_channel, 20, true);
	// uint dmaFinished = pio_sm_get_blocking(pio0, DMA_HANDLER_SM);
	// printf("DMA finished!\n");

	// while(1);;

	printf("DONE!\n");

	// Launch the register loop on core 1
	multicore_launch_core1(ide_register_controller_main);

	main_processing_loop();

	printf("\nFATAL ERROR!! Main processing loop has exited\n");
	return 0;
}

volatile uint32_t readWriteLineValues = 0;
void __not_in_flash_func(process_ata_register_access)() {

	volatile uint32_t bigbessie = 0;
	pio_sm_set_enabled(pio0, 0, true);

	while(1) {

		readWriteLineValues = pio_sm_get_blocking(pio0, 0);

		register_index = readWriteLineValues & 0x7F;
		selectedRegister = registerIndex_map[register_index];

		// WRITE TO DREAMCAST (write == 1, read == 0)
		// read pin active low (so write is high)
		if ((readWriteLineValues & READ_WRITE_PIN_MASK) == READ_PIN_MASK) {

			// pio0->txf[0] = *selectedRegister;
			pio_sm_put_blocking(pio0, 0, *selectedRegister);

			if (writtenRegisterIndex < 5000 && register_index != 0x4e) {
				multicore_fifo_push_blocking(register_index);


				if (register_index != 0x50 && register_index != 0x53) {
					writtenRegisters[writtenRegisterIndex++] = 0xAAAA;
					writtenRegisters[writtenRegisterIndex++] = register_index;
					writtenRegisters[writtenRegisterIndex++] = *selectedRegister;
					writtenRegisters[writtenRegisterIndex++] = 0xDDDD;
				}
			}

		// READ FROM DREAMCAST (write == 0, read == 1)
		// write pin active low (so read is high)
		} else if ((readWriteLineValues & READ_WRITE_PIN_MASK) == WRITE_PIN_MASK) {
			if (writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0xBBBB;
				writtenRegisters[writtenRegisterIndex++] = register_index;
			}

			*selectedRegister = pio_sm_get_blocking(pio0, 0);

			if (writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = *selectedRegister;
				writtenRegisters[writtenRegisterIndex++] = 0xDDDD;
			}
			
			multicore_fifo_push_blocking(register_index);
			
		} 
		// else {
		// 	if (writtenRegisterIndex < 5000) {
		// 		writtenRegisters[writtenRegisterIndex++] = 0xCCCC;
		// 		writtenRegisters[writtenRegisterIndex++] = register_index;
		// 		writtenRegisters[writtenRegisterIndex++] = readWriteLineValues;
		// 		writtenRegisters[writtenRegisterIndex++] = 0xDDDD;
		// 	}
		// }
	}
}

void ide_register_controller_main() {


	uint32_t t = 0x30;
	uint32_t t1 = t & WRITE_PIN_MASK;
	uint32_t t2 = t & READ_PIN_MASK;
	uint32_t t3 = t & READ_WRITE_PIN_MASK;
	printf("%x, %x, %x\n", t1, t2, t3); // 20, 0, 20

	printf("Dreamcast booting...\n");

	// The Dreamcast(something?) does a startup with the cd drive and it toggles all the control, read, and write lines.
	// Not sure if this is real data or just a fun little startup sequence

	while(!gpio_get(PIN_CS0)); // loop until the cs lines are active (really only useful when powering the board on before the console)

	printf("Dreamcast booted!\n");

	busy_wait_ms(1000); // TODO this is likely not needed? 

	selectedRegister = registerIndex_map[SPI_REGISTER_COUNT];
	volatile uint8_t dreamcastWantsRead = 0;
	// 00 (0x0) - nothing
	// 01 (0x1) - read
	// 10 (0x2) - write
	// 11 (0x3) - nothing

	// By using the IORDY pin, we can slow down the control signaling.
	// This makes it a lot slower to read/write to the dreamcast but does at least work
	// TODO: Figure out how to do this without the IORDY pin to speed up the control processing
	gpio_put(PIN_IORDY, 1);

	process_ata_register_access();

	// while(1) {

	// 	do {
	// 		readWriteLineValues = sio_hw->gpio_in & CS_PINS_MASK;
	// 	} while(readWriteLineValues == CS_PINS_MASK || readWriteLineValues == 0x00000);			// 12ns (3 cycles)

	// 	do {
	// 		readWriteLineValues = sio_hw->gpio_in & READ_WRITE_PIN_MASK;
	// 	} while(readWriteLineValues == READ_WRITE_PIN_MASK || readWriteLineValues == 0x00000);

	// 	// Chatgpt suggested this, it seems to execute MUCH faster
	// 	// Something about branch prediction taking a long time and the compiler directive 
	// 	// fixes that
	// 	// do {
	// 	// 	readWriteLineValues = sio_hw->gpio_in & (CS_PINS_MASK | READ_WRITE_PIN_MASK);
	// 	// } while(__builtin_expect(readWriteLineValues == (CS_PINS_MASK | READ_WRITE_PIN_MASK) || readWriteLineValues == 0x00000, 0));
		
	// 	// gpio_put(PIN_IORDY, 0);

	// 	// Figure out the register index and get the pointer to the selected register
	// 	register_index = (sio_hw->gpio_in & REGISTER_PIN_MASK) >> 16; // shift by 16 to offset the data pins (0-15)
	// 	selectedRegister = registerIndex_map[register_index];

	// 	// Read from register send to Dreamcast
	// 	if ((register_index & BIT_SHIFTED_READ_PIN_MASK) == BIT_SHIFTED_READ_PIN_MASK) {
	// 		pio0->txf[IDE_WRITE_TO_HOST_SM] = *selectedRegister;

	// 		multicore_fifo_push_blocking(register_index);
	// 		// core_command_buffer[core0_buffer_index++] = register_index;

	// 		gpio_put(PIN_IORDY, 1);
	// 		// wait for latch
	// 		while(gpio_get(PIN_RD) == 0) { tight_loop_contents(); };
			
	// 		// Send a signal to pio to let it go back to input
	// 		pio0->txf[IDE_WRITE_TO_HOST_SM] = 0;
			
	// 		gpio_put(PIN_IORDY, 0);

	// 	// Write to Dreamcast from register
	// 	} else {
	// 		// Let pio know we are ready to read data
	// 		pio0->txf[IDE_READ_FROM_HOST_SM] = 1;
	// 		*selectedRegister = pio_sm_get_blocking(pio0, IDE_READ_FROM_HOST_SM);

	// 		if (register_index != 0x4e) {
	// 			multicore_fifo_push_blocking(register_index);
	// 		}
	// 		// core_command_buffer[core0_buffer_index++] = register_index;

	// 		gpio_put(PIN_IORDY, 1);
	// 		// wait for latch
	// 		while(gpio_get(PIN_WR) == 0) { tight_loop_contents(); };
	// 		gpio_put(PIN_IORDY, 0);
	// 	}
	// }

	printf("\nERROR!!! Register controller core main loop ended.\n");
}

uint32_t timetrack = 0;
bool hasChirped = false;
volatile uint32_t core0CData = 0;
volatile uint16_t core0commandRegister = 0;
volatile uint16_t* spi_packet_register = 0;
volatile uint8_t spi_packet_word_count = 0;
volatile uint16_t generic_data_buffer[64] = {0}; // Basic buffer to send data for things like req_ses, error, etc

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

static uint8_t command71Flag = 0;
static const uint8_t gdrom_version[] = "Rev 5.07";

// Bake the offset into the starting and ending positions
static uint32_t io_current_position = 0;
static uint32_t io_ending_position = 0;
volatile uint8_t secnr_poll_count = 0;
volatile uint8_t secnr_poll_next_status = 0x80;

void process_packet() {
	current_io_packet_command = SEGA_PACKET_CMD_REGISTER[0];
	ide_current_mode = DATA_MODE_IDLE;
	// Get and set transfer mode from the features register
	ide_current_transfer_mode = SPI_registers[SPI_FEATURES_REGISTER_INDEX] & 1;

	// printf("packet: ");
	// for(int i = 0; i < 12; i++) {
	// 	printf("%x ", SEGA_PACKET_CMD_REGISTER[i]);
	// }
	// printf("\n");

	if (writtenRegisterIndex < 5000) {
		writtenRegisters[writtenRegisterIndex++] = 0xDAAAAAAA;
		writtenRegisters[writtenRegisterIndex++] = SEGA_PACKET_CMD_REGISTER[0];
		writtenRegisters[writtenRegisterIndex++] = 0xAAAAAAAD;
	}

	switch(current_io_packet_command) {
		case 0x70:
		if(writtenRegisterIndex < 5000) {
			writtenRegisters[writtenRegisterIndex++] = 0x70FFFFFF;
		}
		case TEST_UNIT_SEGA_PACKET_CMD: {
			*status_register = 0x50; // only drive ready bit set
			SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x80;
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
			SPI_registers[SPI_ERROR_REGISTER_INDEX] = 0x00; // no error

			// Set interrupt bit and assert line
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);
			break;
		}
		case REQ_STAT_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x11111111;
			}

			// u8  *src = &rom[2188];
			// u8  *dst = buffer;
			// u32 size = buffer[4];

			// memcpy(dst, src, size);
			// count = size;
			// gdrom.busy &= 0x76;
			// gdrom.busy |= 0x08;
			// hwInt(0x0100);
			// break;

			current_io_mode = IO_MODE_WRITE;
			io_current_position = 2188 / 2;
			io_ending_position = (io_current_position + 4) / 2;
			ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;
			
			// put first word in data register
			SPI_registers[SPI_DATA_REGISTER_INDEX] = rom[io_current_position++];

			// Put the correct values in the registers
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0;
			SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = 4;
			*status_register = 0x58; // DRQ = 1 BSY = 0 

			// set irq
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			break;
		}
		case REQ_MODE_SEGA_PACKET_CMD: {
			uint startingAddress = SEGA_PACKET_CMD_REGISTER[2];
			uint length = SEGA_PACKET_CMD_REGISTER[4];
			uint halfLength = length/2; // Divide by two because we are sending 2 bytes at a time (16bit bus)
				
			// Setup the IO infos
			current_io_mode = IO_MODE_WRITE;
			io_current_position = (1976 + startingAddress) / 2; //startingAddress / 2;
			io_ending_position = io_current_position + halfLength;
			ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;
			
			// put first word in data register
			SPI_registers[SPI_DATA_REGISTER_INDEX] = rom[io_current_position++];

			// Put the correct values in the registers
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0;
			SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = length;
			*status_register = 0x58; // DRQ = 1 BSY = 0 

			// set irq
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			// // TODO icegdrom does this with 3 specific cases but nulldc just has a whole canned response array
			// // and none of this specific address checking. Might be worth condensing this without any magic numbers
			// if (startingAddress == 18 && length == 8) {
			// 	if(writtenRegisterIndex < 5000) {
			// 		writtenRegisters[writtenRegisterIndex++] = 0x011F18F8;
			// 	}
				
			// 	memcpy(&((uint8_t*)(generic_data_buffer))[0], gdrom_version, sizeof(gdrom_version));

			// 	// Setup the IO infos
			// 	current_io_mode = IO_MODE_WRITE;
			// 	io_current_position = 0;
			// 	io_ending_position = 4;
			// 	ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;
				
			// 	// put first word in data register
			// 	SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++];

			// 	// Put the correct values in the registers
			// 	SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			// 	SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0;
			// 	SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = 8; // length is 8 bytes
			// 	*status_register = 0x58; // DRQ = 1 BSY = 0 

			// 	// set irq
			// 	gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			// } else if (startingAddress == 0 && length == 10) {
			// 	if(writtenRegisterIndex < 5000) {
			// 		writtenRegisters[writtenRegisterIndex++] = 0x011F0F10;
			// 	}

			// 	memcpy(&((uint8_t*)(generic_data_buffer))[0], 0, 10);
				
			// 	// Setup the IO infos
			// 	current_io_mode = IO_MODE_WRITE;
			// 	io_current_position = 0;
			// 	io_ending_position = 5;
			// 	ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;
				
			// 	// put first word in data register
			// 	SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++];

			// 	// Put the correct values in the registers
			// 	SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			// 	SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0;
			// 	SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = 10;
			// 	*status_register = 0x58; // DRQ = 1 BSY = 0 

			// 	// set irq
			// 	gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			// } else {
			// 	// finish packet by setting an error
			// 	if(writtenRegisterIndex < 5000) {
			// 		writtenRegisters[writtenRegisterIndex++] = 0xDEADDEAD;
			// 	}
			// }

			break;
		}
		case SET_MODE_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x22222222;
			}

			uint startingAddress = SEGA_PACKET_CMD_REGISTER[2];
			uint length = SEGA_PACKET_CMD_REGISTER[4];	
			
			if (length == 0) {
				writtenRegisters[writtenRegisterIndex++] = 0x2222FFFF;
			} else {
				writtenRegisters[writtenRegisterIndex++] = 0x2222AAAA;
			}

			// Setup the IO infos
			current_io_mode = IO_MODE_READ;
			io_current_position = startingAddress / 2;
			io_ending_position = (startingAddress / 2) + (length / 2);
			ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;
			
			// We will be READING data into the reply 11 array

			// Put the correct values in the registers
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x00; // IO=0, CoD=0

			writtenRegisters[writtenRegisterIndex++] = SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX];
			writtenRegisters[writtenRegisterIndex++] = SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX];

			SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = length >> 8;
			SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = length & 0xFF;
			*status_register = 0x58; // DRQ = 1 BSY = 0 

			// set irq
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			break;
		}
		case REQ_ERROR_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x33333333;
			}
			break;
		}
		case GET_TOC_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x14FFFFFF;
			}
			// TOC is ALWAYS 408 bytes
			current_io_mode = IO_MODE_WRITE;
			io_current_position = 0;
			io_ending_position = 204; // 408 bytes / 2 bytes per word
			// ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;

			// printf("Read TOC - Double Density: %x:\n", (SEGA_PACKET_CMD_REGISTER[1] & 0x1));
			// for(int i = 0; i < 12; i++) {
			// 	printf("%x ", SEGA_PACKET_CMD_REGISTER[i]);
			// }
			// printf("\n");

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
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x55555555;
			}

			uint sessionNumber = SEGA_PACKET_CMD_REGISTER[2];
			uint length = SEGA_PACKET_CMD_REGISTER[4];	

			current_io_mode = IO_MODE_WRITE;
			io_current_position = 0;
			io_ending_position = 3; // req ses is always 6 bytes, 3 words
			ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;

			GetDriveSessionInfo((uint8_t*)(generic_data_buffer), sessionNumber);
			((uint8_t*)(generic_data_buffer))[0]= SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX];

			// put first word in data register
			SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++];

			// Put the correct values in the registers
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0;
			SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = 6; // length is 6 bytes
			*status_register = 0x58; // DRQ = 1 BSY = 0 

			// set irq
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			break;
		}
		case CD_OPEN_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x66666666;
			}
			break;
		}
		case CD_PLAY_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x77777777;
			}
			break;
		}
		case CD_SEEK_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x88888888;
			}
			break;
		}
		case CD_SCAN_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x99999999;
			}
			break;
		}
		case CD_READ_SEGA_PACKET_CMD: {
			current_io_mode = IO_MODE_WRITE;
			
			// Read and buffer the data
			gdrom_read_start(SEGA_PACKET_CMD_REGISTER, ide_current_transfer_mode);

			if (ide_current_transfer_mode == IDE_TRANSFER_MODE_PIO) {
				printf("cd_read pio\n");
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
				if(writtenRegisterIndex < 5000) {
					writtenRegisters[writtenRegisterIndex++] = 0xD8A10000;
				}
			}	

			break;
		}
		case CD_READ2_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x1234BBBB;
			}
			break;
		}
		case GET_SCD_SEGA_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0xCCCCCCCC;
			}

			uint8_t format = SEGA_PACKET_CMD_REGISTER[1] & 0xF;
			uint32_t lengthFromPacket = (SEGA_PACKET_CMD_REGISTER[3] << 8) | (SEGA_PACKET_CMD_REGISTER[4]);
			uint32_t length = lengthFromPacket;
			uint8_t* bufferPtr = (uint8_t*)(generic_data_buffer);
			uint8_t discStatus = SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] & 0xF;
			
			/*
			 * Data Format
			 * 0h = All subcode information is transferred as raw data, number of transfer bytes = 96
			 * 1h = Subcode Q data only, 12 bytes
			 * 2h = Media catalog number (UPC/Bar code)
			 * 3h = ISRC code International Standard Recording Code
			 * 4-Fh = reserved
			 */

			// Load the buffer with the correct data
			/*
			 * Byte 0    = reserved
			 * Byte 1    = status
			 * Byte 2,3  = subcode data length 100 = 64h
			 * Byte 4-99 = subcode
			 */

			bufferPtr[0] = 0x00;

			// TODO we aren't actually playing any audio data, but we should probably figure that out. 
			// For now the disc is in standby (sector number = 0x82), so this will return no audio status (0x15)
			if (discStatus == REQ_STAT_INFO_STATUS_PAUSE) {
				bufferPtr[1] = 0x12;
			} else if (discStatus == REQ_STAT_INFO_STATUS_STANDBY) {
				bufferPtr[1] = 0x13;
			} else if (discStatus == REQ_STAT_INFO_STATUS_PLAY) {
				bufferPtr[1] = 0x11;
			} else {
				bufferPtr[1] = 0x15; // no audio status information
			}

			bufferPtr[1] = 0x15;

			if (format == 0) {
				length = 100;
				bufferPtr[2] = 0;
				bufferPtr[3] = 100; // 0x64
				// copy subchannel array?
				memcpy(&bufferPtr[4], q_subchannel, 96);
			} else if (format == 1) {
				length = 0xE; // 14 bytes
				// Data length MSB (0 = 0x0)
				bufferPtr[2] = 0;
				// Data length LSB (14 = 0xE)
				bufferPtr[3] = 0xE;
				// Control(top 4 bits)/ADR (bottom 4 bits)
				bufferPtr[4] = (4<<4) | (1);
				// Copy the rest from nulldc, icegdrom's version looks really complicated :|
				//5-13	DATA-Q
				uint8_t* data_q = &bufferPtr[5-1];
				//-When ADR = 1
				//Byte	Description
				//1	TNO
				data_q[1]=1;//Track number .. duno whats it :P gota parse toc xD ;p
				//2	X
				data_q[2]=1;//gap #1 (main track)
				//3-5	Elapsed FAD within track
				//u32 FAD_el=cdda.CurrAddr.FAD-cdda.StartAddr.FAD;
				data_q[3]=0;//(u8)(FAD_el>>16);
				data_q[4]=0;//(u8)(FAD_el>>8);
				data_q[5]=0;//(u8)(FAD_el>>0);
				//6	0	0	0	0	0	0	0	0
				data_q[6]=0;//
				//7-9	-> seems to be FAD
				data_q[7]=0;//(u8)(cdda.CurrAddr.FAD>>16);
				data_q[8]=0x0;//(u8)(cdda.CurrAddr.FAD>>8);
				data_q[9]=0x96;//(u8)(cdda.CurrAddr.FAD>>0);

			} else {
				// Unsupported, abort!
				printf("SCD format: %x, unsupported\n", format);
			}

			current_io_mode = IO_MODE_WRITE;
			io_current_position = 0;
			io_ending_position = length/2;
			ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;

			// put first word in data register
			SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++];

			// Put the correct values in the registers
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = length >> 8;
			SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = length & 0xFF;
			*status_register = 0x58; // DRQ = 1 BSY = 0 

			// set irq
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			break;
		}
		case Code71_PACKET_CMD: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x71000000 | SEGA_PACKET_CMD_REGISTER[1];
			}

			uint32_t countData71 = 0;
			if (!command71Flag) {
				// Copy the first 0x3f4 bytes of the rom to the buffer
				io_current_position = 0;
				io_ending_position = 0x3f4/2;
				countData71 = 0x3f4;
				command71Flag = 1;

			} else {
				// Copy the next 0x3c0 bytes of the rom to the buffer
				io_current_position = 0x3f8;
				countData71 = 0x3c0;
				io_ending_position = 0x3c0/2;
				command71Flag = 0;
			}

			current_io_mode = IO_MODE_WRITE;
			ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;

			SPI_registers[SPI_DATA_REGISTER_INDEX] = rom[io_current_position++];

			SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x82;
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = countData71 >> 8;
			SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = countData71 && 0xFF; 
			*status_register = 0x58; // DRQ = 1 BSY = 0

			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			// memcpy(&((uint8_t*)generic_data_buffer)[0], cmd71_reply, sizeof(cmd71_reply));

			// current_io_mode = IO_MODE_WRITE;
			// io_current_position = 0;
			// io_ending_position = 2;//sizeof(generic_data_buffer)/2;
			// ide_current_transfer_mode = IDE_TRANSFER_MODE_PIO;

			// SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++];
		

			// // SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x84;
			// SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x82;
			// SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x02; // IO=1, CoD=0
			// SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX] = 0;
			// SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX] = 4; 
			// *status_register = 0x58; // DRQ = 1 BSY = 0

			// // set irq
			// gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			break;
		}
		default: {
			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0xDEAD0000 | current_io_packet_command;
			}
			break;
		}
	}
}

static inline void process_data_written() {

	if (current_io_mode == IO_MODE_READ) {

		if (current_io_packet_command == SET_MODE_SEGA_PACKET_CMD) {

			reply_11[io_current_position++] = SPI_registers[SPI_DATA_REGISTER_INDEX];

			if (io_current_position >= io_ending_position) {
				// for(int i = 0; i < 16; i++) {
				// 	printf("0x%02X ", reply_11[i]);
				// }
				// printf("\n");
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

		}

		return;
	}

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
		if(writtenRegisterIndex < 5000) {
			writtenRegisters[writtenRegisterIndex++] = 0xEEEEEEEE;
		}
		printf("!!error process_data_written- ide_current_mode: %x\n", ide_current_mode);
		for(int i = 0; i < 12; i++) {
			printf("%x ", SEGA_PACKET_CMD_REGISTER[i]);
		}
		printf("\n");
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

	if (current_io_packet_command == REQ_STAT_SEGA_PACKET_CMD) {
		SPI_registers[SPI_DATA_REGISTER_INDEX] = rom[io_current_position++];

		if (io_current_position >= io_ending_position) {
			if (writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x1000FFFF;
			}

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

	} else if(current_io_packet_command == REQ_MODE_SEGA_PACKET_CMD) {
		// SPI_registers[SPI_DATA_REGISTER_INDEX] = reply_11[io_current_position++]; // put next word in data register
		// SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++];
		SPI_registers[SPI_DATA_REGISTER_INDEX] = rom[io_current_position++];

		if (io_current_position >= io_ending_position) {
			if (writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x1100FFFF;
			}

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
		SPI_registers[SPI_DATA_REGISTER_INDEX] = SEGA_PACKET_TOC_INFO_16[io_current_position++];
		if (io_current_position >= io_ending_position) {
			if (writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x1400FFFF;
			}
			// printf("TOC finished\n");
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

	} else if (current_io_packet_command == REQ_SES_SEGA_PACKET_CMD) {
		SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++]; // put next word in data register

		if (io_current_position >= io_ending_position) {
			if (writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x1500FFFF;
			}
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

			// if(writtenRegisterIndex < 5000) {
			// 	writtenRegisters[writtenRegisterIndex++] = 0xD8A00000;
			// }

			
			// So the DMA transfer handshake is once
			////after the dmack line asserts( low) the words are read via a read pin strobe
			////


			// We can only hold 32 sectors of data at a time. If we need more then we need to fetch it
			uint16_t numTransfers = gdrom_read_buffer_size / 2; // We are transferring num words not sectors... todo fix this > 32 ? 32 : numTotalTransfers;

			printf("Starting DMA. Num word transfers: %u\n", numTransfers);

			start_dma_bus_handler();

			printf("Sending num transfers\n");
			// Push number of transfers 
			// Transfer count MINUS 2, 
			// once for the inital word that is loaded on "setup" in the program
			// and once more to skip a final jmp loop since it will take the loop if x is non zero BEFORE decrement
			pio_sm_put_blocking(pio0, DMA_HANDLER_SM, numTransfers-1);

			// And start dma
			dma_channel_set_trans_count(dma_bus_handler_dma_channel, numTransfers+1, true);

			// start dma
			volatile uint dmaFinished = 0; // throw away value to read the finished signal from PIO

			do {
				// gdrom_buffer_has_more_data = gdrom_read_consume_buffer(&SPI_registers[SPI_DATA_REGISTER_INDEX]); // read in another word
				// Wait for num transfers to finish
				
				dmaFinished = pio_sm_get_blocking(pio0, DMA_HANDLER_SM);
				

				if (gdrom_read_remaining_sectors > 0) {
					// Fetch more data
					gdrom_fill_read_buffer(); 

					numTransfers = gdrom_read_buffer_size / 2;
					printf("Refilling DMA buffer, Num word transfers: %u\n", numTransfers);
					
					// Push number of transfers
					pio_sm_put_blocking(pio0, DMA_HANDLER_SM, numTransfers-1);

					// Restart the dma
					dma_channel_set_trans_count(dma_bus_handler_dma_channel, numTransfers+1, true);
				} else {
					// If we are finished, then we should disable the dma state machine and restart the ata handler.
					pio_sm_set_enabled(pio0, DMA_HANDLER_SM, false);
					start_ata_bus_handler(); // stop the dma bus handler and start the ata one
				}

			} while(gdrom_read_remaining_sectors > 0);

			printf("\nDMA finished\n");
		
			// All the data has been sent, signal the end of the transfer
			current_io_mode = IO_MODE_IDLE;
			// Set the status register to indicate the data is finished
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
			*status_register = 0x50; // DRQ = 0 BSY = 0

			// Do we need to assert the irq line for a read?
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0xD8A01111;
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

	} else if (current_io_packet_command == CD_PLAY_SEGA_PACKET_CMD) {
		printf("CD_PLAY_SEGA_PACKET_CMD\n");
	} else if (current_io_packet_command == CD_SEEK_SEGA_PACKET_CMD) {
		printf("CD_SEEK_SEGA_PACKET_CMD\n");
	} else if (current_io_packet_command == CD_SCAN_SEGA_PACKET_CMD) {
		printf("CD_SCAN_SEGA_PACKET_CMD\n");
	} else if (current_io_packet_command == CD_READ2_SEGA_PACKET_CMD) {
		printf("CD_READ2_SEGA_PACKET_CMD\n");
	} else if (current_io_packet_command == GET_SCD_SEGA_PACKET_CMD) {
		SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++]; // put next word in data register

		if (io_current_position >= io_ending_position) {

			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x4000FFFF;
			}

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

	} else if (current_io_packet_command == Code71_PACKET_CMD) {
		// SPI_registers[SPI_DATA_REGISTER_INDEX] = generic_data_buffer[io_current_position++];
		SPI_registers[SPI_DATA_REGISTER_INDEX] = rom[io_current_position++];

		if (io_current_position >= io_ending_position) {
			current_io_packet_command = 0;
			io_current_position = 0;
			io_ending_position = 0;
			current_io_mode = IO_MODE_IDLE;

			// Set the status register to indicate the data is finished
			// SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = 0x80;
			SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x03; // IO=1, CoD=1
			*status_register = 0x50; // DRQ = 0 BSY = 0
			
			// Assert the IRQ line to announce we are finished
			gpio_put(PIN_INTRQ, INTRQ_ASSERT);

			if(writtenRegisterIndex < 5000) {
				writtenRegisters[writtenRegisterIndex++] = 0x71FFFFFF;
			}
		}

	} else {
		printf("\nunimplemented current_io_packet: %u\n", current_io_packet_command);
		printf("Resetting all variables\n\n");
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

}

void main_processing_loop() {
	printf("Opening default disc image...\n");
	gdrom_read_default_disc_image();
	printf("DONE!\n");


	// // Test the read command
	// SEGA_PACKET_CMD_REGISTER[0] = 0x30;
	// SEGA_PACKET_CMD_REGISTER[1] = 0x24;
	// SEGA_PACKET_CMD_REGISTER[2] = 0x00;
	// SEGA_PACKET_CMD_REGISTER[3] = 0xb0;
	// SEGA_PACKET_CMD_REGISTER[4] = 0x5e;
	// SEGA_PACKET_CMD_REGISTER[5] = 0x00;
	// SEGA_PACKET_CMD_REGISTER[6] = 0x00;
	// SEGA_PACKET_CMD_REGISTER[7] = 0x00;
	// SEGA_PACKET_CMD_REGISTER[8] = 0x00;
	// SEGA_PACKET_CMD_REGISTER[9] = 0x00;
	// SEGA_PACKET_CMD_REGISTER[10] = 0x07;
	// SEGA_PACKET_CMD_REGISTER[11] = 0x00;

	// uint32_t endTime = 0;
	// uint32_t startTime = time_us_32();
	// gdrom_read_start(SEGA_PACKET_CMD_REGISTER, 0);
	// endTime = time_us_32();

	// printf("Read Time: %u\n", endTime - startTime);

	// printf("GDROM Read Test\n");

	// printf("SPI Command Packet:\t");
	// for(int i = 0; i < 12; i++) {
	// 	printf("(%u)%x ", i, SEGA_PACKET_CMD_REGISTER[i]);
	// }
	// printf("\n");

	// printf("Sector Start: %u(0x%x), Sector Count: %u(0x%x), Sector Size: %u(0x%x)\n", gdrom_read_start_sector, gdrom_read_start_sector, gdrom_read_remaining_sectors, gdrom_read_remaining_sectors, gdrom_read_sector_size, gdrom_read_sector_size);
	// printf("Data Buffer:");
	// for (int i = 0; i < 32; i++) {
	// 	if (i % 8 == 0) {
	// 		printf("\n%d: ", i);
	// 	}
	// 	printf("%x ", gdrom_read_buffer[i]);
	// }

	// printf("\nData Register:");
	// for (int i = 0; i < 32; i++) {
	// 	gdrom_read_consume_buffer(&SPI_registers[SPI_DATA_REGISTER_INDEX]); // read in another word
	// 	if (i % 8 == 0) {
	// 		printf("\n%d: ", i);
	// 	}
	// 	printf("%x ", SPI_registers[SPI_DATA_REGISTER_INDEX]);
	// }
	// printf("\n\n");

	// printf("TOC dump...\n");
	// GetDriveToc((uint32_t*)(SEGA_PACKET_TOC_INFO), SingleDensity);
	// for(int i = 0; i < 408; i++) {
	// 	if (i % 8 == 0) {
	// 		printf("\n");
	// 	}
	// 	printf("%x ", SEGA_PACKET_TOC_INFO[i]);
	// }
	// printf("DONE!\n");

	rom = (uint16_t*)(gdrom_rom_data);

	spi_packet_register = (uint16_t*)(&SEGA_PACKET_CMD_REGISTER);

	while(1) {
		
		if(time_us_32() - timetrack > 10000000/*60000000*//*18000000*/ && !hasChirped) {
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
			
			// Allow this to dump data every minute
			writtenRegisterIndex = 0;
			hasChirped = false;

			// for(int i = 0; i < SPI_REGISTER_COUNT; i++) {
			// 	printNameOfRegister(i);
			// 	printf(" = %x\n", SPI_registers[i]);
			// }

			printf("status: %x, alt_status: %x\n", *registerIndex_map[0x57], *registerIndex_map[0x4e]);

			printf("----------------------------------------\n");
			printf("/n/n");
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			printf("alt status: %x\n", SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]);
			printf("status: %x\n",SPI_registers[SPI_STATUS_REGISTER_INDEX]);
			printf("device control: %x\n",SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]);
			printf("data: %x\n",SPI_registers[SPI_DATA_REGISTER_INDEX]); 
			printf("features: %x\n",SPI_registers[SPI_FEATURES_REGISTER_INDEX]);
			printf("error: %x\n",SPI_registers[SPI_ERROR_REGISTER_INDEX]);
			printf("interrupt: %x\n",SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX]);
			printf("sector number: %x\n",SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX]);
			printf("byte count low: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_LOW_INDEX]); 
			printf("byte count high: %x\n",SPI_registers[SPI_BYTE_COUNT_REGISTER_HIGH_INDEX]);
			printf("drive select: %x\n",SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX]);
			printf("cmd: %x\n",SPI_registers[SPI_COMMAND_REGISTER_INDEX]);
			printf("sector count: %x\n", SPI_registers[SPI_SECTOR_COUNT_REGISTER_INDEX]);
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");


			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

		}

		// If we are sending data and in DMA mode
		if(ide_current_transfer_mode && current_io_mode == IO_MODE_WRITE) {
			process_data_read();
		}

		// // Wait for data from core1 to be available
		if(!multicore_fifo_rvalid()) {
			continue;
		}

		// // Get data
		core0CData = multicore_fifo_pop_blocking();

		// if (core1_buffer_index == core0_buffer_index) {
		// 	continue;
		// }

		// if (core1_buffer_index != core0_buffer_index) {
		// 	core0CData = core_command_buffer[core1_buffer_index++];
		// }

		// Please skip all the stupid alt status register reads
		// if(writtenRegisterIndex < 5000 && core0CData != 0x4e) {
		// 	writtenRegisters[writtenRegisterIndex++] = core0CData;

		// 	if (core0CData == 0x53) {
		// 		writtenRegisters[writtenRegisterIndex++] = 0x53FFFFFF;
		// 		secnr_poll_count++;
		// 		if (secnr_poll_count > 10) {
		// 			secnr_poll_next_status = 0x80 + 1;
		// 			if (secnr_poll_next_status == 0x86) {
		// 				secnr_poll_next_status = 0x80;
		// 			}

		// 			writtenRegisters[writtenRegisterIndex++] = secnr_poll_next_status;
		// 			SPI_registers[SPI_SECTOR_NUMBER_REGISTER_INDEX] = secnr_poll_next_status;
		// 			writtenRegisters[writtenRegisterIndex++] = 0x53AAAAAA;
		// 		}
		// 	}
		// }

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
			// This else block seems to be causing a lot of problems
			// I only see it appear after a 0x11 packet command
			// And it should appear for any of the other commands in process_data_read
			// since we write the last word THEN pull the IRQ, and then the DC should read the final
			// word and then clear the irq.
			// This doesn't appear to be working as expected and I have no idea why.....
			// else {
			// 	// printf("ERROR register READ current_io_mode incorrect: %x | %x, %u, %u, dma:%u\n", current_io_mode, current_io_packet_command, io_current_position, io_ending_position, ide_current_transfer_mode);
			// 	if(writtenRegisterIndex < 5000) {
			// 		writtenRegisters[writtenRegisterIndex++] = 0xDEADDDDD;
			// 	}
			// }

			continue;
		}

		// register_index -> selected register 
		// selectedRegister -> register pointer
		// The command register is the only one that needs to be processed (for now)
		if(core0CData == CODED_COMMAND_REGISTER_WRITE) {

			// !!!BSY bit must be set within 400ns, so if we need more time, this bit should be set
			// .. update status register
			// *status_register = *status_register | 0x80; // BSY bit set

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
						printf(".\n");

						SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x01; // IO=0, CoD=0
						*status_register = 0x50; // DRQ = 0 BSY = 0
		
						// Assert the IRQ line to announce we are finished
						gpio_put(PIN_INTRQ, INTRQ_ASSERT);

						/*
						The task file register is initialized as follows.
						Status = 00h, 
						Error = 01h, 
						Sector Count = 01h, 
						Sector Number = 01h, 
						Cylinder Low = 14h, 
						Cylinder High = EBh, 
						Drive/Head = 00h
						BSY = 0 
						following after any reset indicates that 
						the task file register is already initialized for the host.
						*/
						break;
					}
					case ATA_CMD_SOFT_RESET: {
						printf("soft_reset\n");
						break;
					}
					// case ATA_CMD_PACKET_COMMAND: { // now handled in it's own if block
					// 	// Process sega packet interface
					// 	// ...
					// 	break;
					// }
					case ATA_CMD_IDENTIFY_DEVICE: {
						printf("identify_defice\n");
						break;
					}
					case ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC: {
						printf("device_diagnostics\n");
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
				// *status_register = *status_register ^ 0x80; 
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

