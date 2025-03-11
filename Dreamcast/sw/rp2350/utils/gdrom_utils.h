#pragma once

#include "common.h"

/*
 * read data from the sd card using params from the SPI command packets
 * This could probably just be part of sega_packet_interface.c/h but we can evaluate that later
 * 
 */

#define MAX_SECTOR_BYTE_SIZE (2352) // This is the size of a sector on the disc
#define MAX_BUFFERED_SECTORS (32)   // This is the max number of sectors to buffer at a time
#define MAX_BUFFER_SIZE (MAX_SECTOR_BYTE_SIZE * MAX_BUFFERED_SECTORS)
extern uint8_t gdrom_read_buffer[MAX_BUFFER_SIZE]; // This is about 73KB of ram
extern uint16_t* gdrom_16bit_buffer_ptr;

extern uint8_t gdrom_read_data_select_value;
extern uint8_t gdrom_read_expected_data_type;
extern uint8_t gdrom_read_data_parameter_type; // 0 = FAD or 1 =MSF

extern uint32_t gdrom_read_start_sector;
extern uint32_t gdrom_read_read_sectors; // Used by fill buffer to offset another preload
extern uint32_t gdrom_read_remaining_sectors;
extern uint32_t gdrom_read_sector_size;
extern uint32_t gdrom_read_buffer_index;
extern uint32_t gdrom_read_buffer_size; // how much data is actually in here

extern uint32_t gdrom_read_current_byte_count;
extern uint32_t gdrom_read_bytes_remanining_in_sector;
extern uint32_t gdrom_read_bytes_remanining;

// Not sure I need to keep track of any of this as it's done in the nulldc code
// enum cd_read_data_select_type {
//     CD_READ_HEADER = 0,
//     CD_READ_SUBHEADER = 1,
//     CD_READ_DATA = 2,
//     CD_READ_OTHER = 3 // all bytes of the sector are output (2352)

//     // If data type is CD_DA, this field is ignored and transfers 2352 bytes
// };

// enum cd_read_expected_data_type {
//     CD_READ_DATA_TYPE_ANY = 0b000,
//     CD_READ_DATA_TYPE_CD_DA = 0b001,
//     CD_READ_DATA_TYPE_MODE_1 = 0b010,
//     CD_READ_DATA_TYPE_MODE_2 = 0b011,
//     CD_READ_DATA_TYPE_MODE_2_FORM_1 = 0b100,
//     CD_READ_DATA_TYPE_MODE_2_FORM_2 = 0b101,
//     CD_READ_DATA_TYPE_MODE_2_NON_XA = 0b110
// };

// struct {
//     cd_read_data_select_type dataSelect;
//     cd_read_expected_data_type expectedDataType;
//     bool isFAD;
//     uint32_t startAddress;
//     uint32_t transferLength;
// } cd_read_params;

// struct {
//     uint32_t sectorSize;
//     uint32_t sectorStart;
//     uint32_t sectorEnd;
//     uint32_t currentSector;

//     uint32_t currentBufferPosition; // in bytes
    
// } cd_read_state;

// Read and process the disc image
void gdrom_read_default_disc_image();

// Call at the start of a cd_read command to setup state variables and init the read buffer
void gdrom_read_start(uint8_t* packet, bool isDMA);

// Pass in the ata data register to write the data to
// Returns false if there is no more data to read
// Returns true if the buffer still has requested data remaining
bool gdrom_read_consume_buffer(uint16_t* toBuffer);

// Read sectors and put them in the buffer
// Another helper method that probably doesn't need to be called directly
// Unless we swap to reading a sector at a time
void gdrom_read_sectors(uint8_t* buffer, uint32_t sector, uint32_t sectorCount, uint32_t sectorSize);

// Used as a helper function to fill the read buffer with data from the disc
// Probably dont need to call this method as it's called from gdrom_read_start
void gdrom_fill_read_buffer();