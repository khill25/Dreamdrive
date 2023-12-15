/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include "sega_packet_interface.h"

uint16_t ATA_task_file_register[ATA_TFR_REGISTER_COUNT] = {
	0x00,
	0x01,
	0x01,
	0x01,
	0x14,
	0xEB,
	0x00,
};

uint16_t SPI_registers[SPI_REGISTER_COUNT+1];
uint8_t SEGA_PACKET_CMD_REGISTER[12] = {0};
uint8_t SEGA_PACKET_TOC_INFO[408] = {0};

void SPI_issue_device_reset(uint8_t reset_type) {
	switch(reset_type) {
		case POWER_ON_OR_HARDWARE_RESET:
			break;
		case SPI_SOFT_RESET:
			break;
		case ATA_SRST_RESET:
			break;
		default:
			break;
	}
}

/*
 * Return the selected register based on signal line values and if passed in, sets the selected register index pointer as a second return value
 * Returns the value of the register, if the value returned is equal to the register count, then no register was returned
 *
 * 00xxx 1x - Data bus high imped
 * 010xx 1x - Data bus high imped
 * 0110x 1x - Data bus high imped
 *
 * 01110 10 - (0xE) Alternate status register (read)
 * 01110 01 - (0xE) Device Control Register (write)
 *
 * 10000 10 - (0x10) Data (read)
 * 10000 01 - (0x10) Data (write)
 *
 * 10001 10 - (0x11) Error Register (read)
 * 10001 01 - (0x11) Features Register (write)
 *
 * 10010 1x - (0x12) Interrupt Reason Register (read)
 *
 * 10011 1x - (0x13) Sector Number (read)
 *
 * 10100 10 - (0x14) Byte Count Register bits 0-7 (read)
 * 10100 01 - (0x14) Byte Count Register bits 0-7 (write)
 *
 * 10101 10 - (0x15) Byte Count Register bits 8-15 (read)
 * 10101 01 - (0x15) Byte Count Register bits 8-15 (write)
 *
 * 10110 10 - (0x16) Drive Select Register (read)
 * 10110 01 - (0x16) Drive Select Register (write)
 *
 * 10111 10 - (0x17) Status Register (read)
 * 10111 01 - (0x17) Command Register (write)
 *
 * 11 xxx xx - Invalid address
 *
 */


/*
 * IO and CoD are from the Interrupt reason register
 * DRQ is from status register bit 3
 */
uint8_t SPI_interrupt_reason(bool io, bool drq, bool cod) {
	uint8_t reason = (io << 2 | drq << 1 | cod);
	return reason;
}

void SPI_set_BSY(bool isBusy) {
	// uint8_t* status_register = SPI_registers[SPI_STATUS_REGISTER_INDEX];
	// uint8_t* alternate_status_register = SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX];

	// (&status_register) |= isBusy << SPI_STATUS_BSY;
	// (&alternate_status_register) |= isBusy << SPI_ALTERNATE_STATUS_BSY;
}

void SPI_set_DRQ(bool isDataReady) {
	// uint8_t* status_register = SPI_registers[SPI_STATUS_REGISTER_INDEX];
	// uint8_t* alternate_status_register = SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX];

	// (&status_register) |= isBusy << SPI_STATUS_DRQ;
	// (&alternate_status_register) |= isBusy << SPI_ALTERNATE_STATUS_DRQ;
}

void SPI_assert_INTRQ(bool valueHigh) {
	// if valueHigh = 1, pull high
	// if valueHigh = 0, pull low
}

// Handles calling the right functions based on the command function
void SPI_execute_cmd() {
	// Access command register
	uint16_t commandRegister = SPI_registers[SPI_COMMAND_REGISTER_INDEX];
	switch (commandRegister) {
		case ATA_CMD_NOP:
			break;
		case ATA_CMD_SOFT_RESET:
			break;
		case ATA_CMD_PACKET_COMMAND:
			break;
		case ATA_CMD_IDENTIFY_DEVICE:
			break;
		case ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC:
			break;
		case ATA_CMD_SET_FEATURES:
			break;
	}
}

/*
 * Host is reading the status register. It appears that we always want to deassert INTRQ AFTER we send the data?
 */
void SPI_read_status_register() {
	// TODO: Send status register
	SPI_assert_INTRQ(false);
}

// Only applies to ATA_CMD_IDENTIFY_DEVICE command
void SPI_execute_ata_data_cmd() {
/*
I think this is only valid for ATA_CMD_IDENTIFY_DEVICE commands
Command flow
	* host polls status/alt status registers until BSY = 0
	* host writes values to registers
	* host writes command to command register
	* Device sets BSY = 1
	* When data is ready:
		* DRQ = 1 (if no error), BSY = 0, Assert INTRQ
	* Host polls for the above^ conditions via alternate status register
	* Host reads out and saves status register
		* When status register is read out, negate INTRQ (pull low)
	* If DRQ = 1, host reads and transfers one block from Data Register
	* After data block transfered:
		* If more blocks required, BSY = 1 and repeat last step
		* If error, clear DRQ
		* After final block, clear DRQ
*/
	// Starting to execute command
	SPI_set_BSY(true);

	// Access command register
	uint16_t* commandRegister = &SPI_registers[SPI_COMMAND_REGISTER_INDEX];
	// command should be ATA_CMD_IDENTIFY_DEVICE

	// Fetch data
	//...

	// Data ready!
	SPI_set_BSY(false);
	SPI_assert_INTRQ(true); // TODO: PIO mode = assert every block, DMA once transfer is finished
	SPI_set_DRQ(true);

	// If there are more blocks, repeat

	// TODO: this will have to be done in two steps so this function can return and wait for
	// the status register to be read...
	//...
	//... Wait for the host to read the status register, then deassert INTRQ
	//...
	SPI_assert_INTRQ(false);
}

// Applies to NOP, SOFT_RESET, EXECUTE_DEVICE_DIAGNOSTIC, SET_FEATURES
void SPI_execute_ata_non_data_cmd() {
/*
non data command flow
	* host polls status/alt status register until BSY = 0
	* Host writes params to registers
	* Host writes command to command register
	* Device sets BSY = 1
	* Once cmd is finished BSY = 1 and assert INTRQ
*/

	 // Starting to execute command
	SPI_set_BSY(true);

	// Access command register
	//...

	// Do CMD
	//...

	// Finished!
	SPI_set_BSY(false);
	SPI_assert_INTRQ(true);
}

// SPI Packet Command for PIO Data to host
/*
Valid for these SPI Packet commands
 * REQ_STAT
 * REQ_MODE
 * REQ_ERROR
 * GET_TOC
 * REQ_SES
 * CD_READ
 * CD_READ2
 * GET_SCD
 */
void SPI_execute_packet_cmd_pio_data1() {
	/*
	 * host polls until BSY=0 and DRQ=0, then sets data into some registers
	 * Host writes packet command code (0xA0 = ATA_CMD_PACKET_COMMAND) to the command register
	 * Device sets BSY = 1 within 400ns
	 * Device sets CoD bit and clears the IO bit
		* DRQ bit must be valid before making BSY = 0
		* DRQ = 1, BSY = 0 so host will send packet data
	 * Host polls to verify DRQ bit... so device sets DRQ=1??? TODO unclear?
	 * Host will send packet data once DRQ = 1??? (I think this is right) send to (SEGA_PACKET_CMD_REGISTER)
	 * After 12th byte written device sets
		* DRQ = 0
		* BSY = 1
		* FEATURES and BYTE_COUNT registers contain params
	 * Device prepars data?
	 *** Device writes
		* Num bytes to be read in BYTE_COUNT register
		* IO = 1, CoD = 0
		* DRQ = 1, BSY = 0
		* Assert INTRQ
	 * Host checks INTRQ and the DRQ bit to determine if it can send more command
		* DRQ = 0 - Device has executed command
		* DRQ = 1 - Host must read data (distinct byte numbers in BYTE_COUNT register) via DATA_REGISTER
		* When host reads status register deassert INTRQ
	 * Device sets DRQ = 0 if host sent entire data else BSY=1 and repeat "***" step
	 * When device is ready to send status
		* Write final status to status register
		* CoD = 1, IO=1, DRDY=1 (before deasserting INTRQ), then BSY=0, DRQ=0... Assert INTRQ???
	 * Host reads status register if DRQ=0 && INTRQ=0(deasserted)
		* If CHECK bit is set, read command completion status from the ERROR register
	*
	*
	* DRQ is used by device to indicate it's ready for data transfer
		*  It is cleared after the last data byte has been sent
	* If data transmission amount = 0 in the command, DRQ=0 (executed command success?) and termination is okay
	 */
}

/*
 * Valid for
 * SET_MODE
 */
void SPI_execute_packet_cmd_pio_data2() {

}

/*
 * Valid for
 * CD_READ
 * CD_READ2
 */
void SPI_execute_packet_cmd_dma_read() {
/*
 * Host polls for BSY=0, DRQ=0 and sets data in FEATURES and BYTE_COUNT registers
 * Host writes packet command code to command register (0xA0 = ATA_CMD_PACKET_COMMAND)
 * Device sets BSY=1 within 400ns and prepares for command packet transfer
 * When device is ready, set CoD bit and clear IO bit
 * Host polls DRQ then writes 12byte command to data register (SEGA_PACKET_CMD_REGISTER)
 * Device
	* Sets DRQ = 0 after 12th byte is written
	* BSY = 1
	* Read FEATURES and BYTE_COUNT registers
	* Make preparations to send data
 *** Device sends data packets using DMARQ/DMACK
	* Packets must be managed in their own buffers?
	* Device asserts DMARQ and host will respond with DMACK once it's ready for more data
	* TODO: HOW MUCH DATA BEFORE ASSERTING DMARQ?????
 * Once finished with data, device sets final status (IO=0, CoD=0, DRDY=1, BSY=0, DRQ=0) to STATUS register
 *
 *
 * If data transmission amount is 0 in the command, DMARQ/DMACK is not carried out in step "***"
 * but INTRQ is enabled and termination is okay
 */
}

/*
 * Valid for
 * TEST_UNIT
 * CD_OPEN
 * CD_PLAY
 * CD_SEEK
 * CD_SCAN
 */
void SPI_execute_packet_cmd_non_data() {
/*
 * Host polls for BSY=0, DRQ=0 and sets data in FEATURES and BYTE_COUNT registers
 * Host writes packet command code to command register (0xA0 = ATA_CMD_PACKET_COMMAND)
 * Device sets BSY=1 within 400ns and prepares for command packet transfer
 * When device is ready, set CoD bit and clear IO bit
 * Host polls DRQ then writes 12byte command to data register (SEGA_PACKET_CMD_REGISTER)
 * Device set BSY=1 and executes command
 * Device sets status (IO=0, CoD=0, DRDY=1, BSY=0, DRQ=0) ... assert INTRQ???
 */
}
