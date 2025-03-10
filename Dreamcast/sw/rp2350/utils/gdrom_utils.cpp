/*
 * gdrom_utils.c
 * 
 * Utility functions to convert between CD read info and the iso on the sd card
 * IE Using the params from a SPI command, get the data from the iso image
 */

 /*
gdi file contents

-- Looks like
num tracks
track x, x, x, x, filename, x
....

3
1 0 4 2352 track01.bin 0
2 600 0 2352 track02.raw 0
3 45000 4 2352 track03.bin 0

*/

#include "pico/stdlib.h"
#include "gdrom_utils.h"
#include "gdi.h"
#include "sd_utils.h"

#define GDROM_BYTES_PER_READ (2)

Disc* current_disc;
FIL* track_files;
FATFS ddrdc_fs;

uint8_t gdrom_read_buffer[MAX_BUFFER_SIZE];
uint16_t* gdrom_16bit_buffer_ptr = (uint16_t*)gdrom_read_buffer;

uint8_t gdrom_read_data_select_value = 0;
uint8_t gdrom_read_expected_data_type = 0;
uint8_t gdrom_read_data_parameter_type = 0;

uint32_t gdrom_read_buffer_index = 0;
uint32_t gdrom_read_buffer_size = 0;

uint32_t gdrom_read_read_sectors = 0;
uint32_t gdrom_read_start_sector = 0;
uint32_t gdrom_read_remaining_sectors = 0;
uint32_t gdrom_read_sector_size = 0;
uint32_t gdrom_read_original_num_sectors_to_read = 0;

uint32_t gdrom_read_current_byte_count = 0;
uint32_t gdrom_read_bytes_remanining_in_sector = 0;
uint32_t gdrom_read_bytes_remanining = 0;

// Helper functions //////////////////////////////////////////////

// If needed convert the stating address into MSF format
uint32_t _gdrom_read_get_FAD(uint8_t* data, uint8_t msf) {
    if (msf) {
        return (uint32_t)((data[0]*60*75) + (data[1]*75) + (data[2]));
    }

    return (uint32_t)((data[0] << 16) | (data[1] << 8) | (data[2]));
}

uint32_t _gdrom_get_sector_type(uint8_t dataSelect, uint8_t expectedDataType, uint8_t isFAD) {
    
    // data select bits 
    // 3 = header       0b1000
    // 2 = subheader    0b0100
    // 1 = data         0b0010
    // 0 = other        0b0001
    
    // Header, subheader, and data bites are all set AND the expected data type is Mode 2 Form 1
    if(((dataSelect & 0b1110) == 0b1110) && (expectedDataType == 0b011)) {
        // printf("Sector size set to 2340 Bytes \tHeader/Subheader/Data, Mode 2\n");
        return 2340;

    // If data bit with any other data select bits are set and not mode 2
    } else if (((dataSelect & 0b1101) != 0) && ((dataSelect & 0b0010) == 0)) {
        // Nulldc doesn't support anything other than mode2?
        printf("ERROR: Add more cd read settings: %x\n", dataSelect);
        return 0;
    }
	
    // printf("Sector size set to 2048 Bytes\n");
	return 2048;
}

// Main functions ////////////////////////////////////////////////

// Use the crazy taxi image as our default image until we create a menu loader that we will load on startup.
char default_disc_image_path[] = "/crazytaxi/Crazy Taxi v1.004 (1999)(Sega)(US)[!][6S].gdi";
void gdrom_read_default_disc_image() {
    current_disc = gdi_parse(default_disc_image_path);
    if (current_disc == NULL) {
        printf("Failed to load default disc image\n");
    }
}

// Parses the data from the read command packet and sets up the state variables for the read
void gdrom_read_start(uint8_t* packet, bool isDMA) {

    // Zero out all the values on a fresh read start / helps if a dma timed out
    gdrom_read_read_sectors = 0;
    gdrom_read_buffer_size = 0;
    gdrom_read_buffer_index = 0;
    gdrom_read_buffer_size = 0;
    gdrom_read_bytes_remanining_in_sector = 0;
    gdrom_read_bytes_remanining = 0;
    gdrom_read_current_byte_count = 0;
    gdrom_read_bytes_remanining_in_sector = 0;
    gdrom_read_bytes_remanining = 0;

    gdrom_read_data_select_value = (packet[1] & 0xFF00) >> 8;
    gdrom_read_expected_data_type = (packet[1] & 0xE) >> 1;
    gdrom_read_data_parameter_type = (packet[1] & 0x1);

    gdrom_read_sector_size = _gdrom_get_sector_type(gdrom_read_data_select_value, gdrom_read_expected_data_type, gdrom_read_data_parameter_type);
    gdrom_read_start_sector = _gdrom_read_get_FAD(&packet[2], gdrom_read_data_parameter_type);
    // printf("Start Sector: %u\n", gdrom_read_start_sector);
    gdrom_read_remaining_sectors = (packet[8] << 16) | (packet[9] << 8) | (packet[10]);
    
    gdrom_read_original_num_sectors_to_read = gdrom_read_remaining_sectors;

    gdrom_read_bytes_remanining = gdrom_read_remaining_sectors * gdrom_read_sector_size;

    // Setup a 16bit version of the buffer
    gdrom_16bit_buffer_ptr = (uint16_t*)gdrom_read_buffer;

    if (isDMA) {
        // Fill the buffer
        gdrom_fill_read_buffer();
    } else {
        // Fill a single sector? 
        // TODO fill buffer with the number of sectors to read.
        gdrom_fill_read_buffer();
    }

    if (current_disc->type == GdRom && gdrom_read_start_sector == 45150 && gdrom_read_original_num_sectors_to_read == 7) {
        printf("Special buffer patching for sector 45150 with sector count of 7\n");
        PatchRegion_0(gdrom_read_buffer, gdrom_read_sector_size);
        PatchRegion_6(gdrom_read_buffer + 2048 * 6, gdrom_read_sector_size);
    } else {
        printf("Disc type: %d, startSector: %u, numSectorsToRead: %u\n", current_disc->type, gdrom_read_start_sector, gdrom_read_original_num_sectors_to_read);
    }
}

// Updates state variables for micro updates related to actually sending data over the 16-bit bus
bool gdrom_read_consume_buffer(uint16_t* toBuffer) {
    gdrom_read_bytes_remanining_in_sector -= GDROM_BYTES_PER_READ;
    gdrom_read_bytes_remanining -= GDROM_BYTES_PER_READ;

    // Bytes will be swapped correctly by reading a 16bit value from the 8bit buffer
    toBuffer[0] = gdrom_16bit_buffer_ptr[gdrom_read_buffer_index];
    gdrom_read_buffer_index += GDROM_BYTES_PER_READ; // Increment buffer index AFTER reading data to caller

    // If we have read all the bytes in the sector, reset the bytes remanining in sector
    if (gdrom_read_bytes_remanining_in_sector == 0) {
        gdrom_read_bytes_remanining_in_sector = gdrom_read_sector_size;
    }

    // If we reach the end of the buffer...
    if (gdrom_read_buffer_index >= gdrom_read_buffer_size) {
        
        // and we have more sectors, buffer them
        if (gdrom_read_remaining_sectors > 0) {
            gdrom_fill_read_buffer();

        // No more sectors to read
        } else {
            printf("Data finished!\n");
            // Zero out all the state variables    
            gdrom_read_buffer_size = 0;
            gdrom_read_buffer_index = 0;
            gdrom_read_buffer_size = 0;
            gdrom_read_bytes_remanining_in_sector = 0;
            gdrom_read_bytes_remanining = 0;
            return false; // Finished reading all the data!!
        }
    }

    return true; // more data still needs to be read
}

void gdrom_read_sectors(uint8_t* buffer, uint32_t sector, uint32_t sectorCount, uint32_t sectorSize) {

    current_disc->ReadSectors(sector, sectorCount, buffer, sectorSize);

    // TODO figure out what this patching is actually used for, might not be needed?
    // cross check with icegdrom code to see if their tools do anything like this
    // to the disc image conversions.
    if (current_disc->type == GdRom && sector == 45150 && sectorCount == 7) {
        PatchRegion_0(buffer, sectorSize);
        PatchRegion_6(buffer + 2048 * 6, sectorSize);
    }
}

// Fills the buffer up to a max of 32 sectors at a time, refill buffer as needed
void gdrom_fill_read_buffer() {
    uint32_t count = (gdrom_read_remaining_sectors > 32) ? 32 : gdrom_read_remaining_sectors;

	gdrom_read_buffer_index = 0;
	gdrom_read_buffer_size = count * gdrom_read_sector_size;

    // Calcuate the start offset by adding gdrom_read_read_sectors to start sector
	current_disc->ReadSectors(gdrom_read_start_sector+gdrom_read_read_sectors, count, gdrom_read_buffer, gdrom_read_sector_size);

	gdrom_read_read_sectors+=count;
	gdrom_read_remaining_sectors-=count;
}