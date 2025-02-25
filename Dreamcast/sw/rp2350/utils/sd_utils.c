/*
 * sd_utils.c
 * 
 * Utility functions for the SD card
 */

 #include <stdio.h>

#include "pico/stdlib.h"

#include "f_util.h"
#include "ff.h"
#include "hw_config.h"

 #include "sd_utils.h"

// TODO
// * Create a read buffer.... so we can dma? How do we read this from main to send to DC?
// * Probably want to hold on to the file handle pointer so we don't have to open it every time

// We have 520KB of total memory
// Sectors are 2048 bytes (I think?)
// SD card sectors are 512 bytes so we can read 4 sectors at a time into a single gdrom sector buffer
// nulldc uses a 32 sector buffer 
#define MAX_SECTOR_BYTE_SIZE (2336) // This is a weird number, but also includes dummy bytes
#define MAX_BUFFERED_SECTORS (32)
#define MAX_BUFFER_SIZE (MAX_SECTOR_BYTE_SIZE * MAX_BUFFERED_SECTORS)
uint8_t sd_read_buffer[MAX_BUFFER_SIZE]; // Place to put read data


/*
//Buffer for sector reads [ dma ]
struct 
{
	u32 cache_index;
	u32 cache_size;
	u8 cache[2352*32];	//up to 32 sectors
} read_buff;
*/

/*
Sector size
mode 1 = 2048 bytes
mode 2 = 2336 bytes
mode 2 form 1 = 2048 bytes
mode 2 form 2 = 2324 bytes
mode 2 non-xa = 2336 bytes

Extra info...
When Mode 2 has been specified, a Sub Header is also automatically transferred when data is specified in Data Select. 
Also, when the data is Form 1: 
    28 bytes are added as dummy bytes for the transmission 
When the data is Form 2: 
    4 bytes are added as dummy bytes.
It should be noted that because sector type check is not performed the error correction function does not work for the CD-ROM.
*/

 // Setup the sd card (and create a file handle?)
 void sdcard_init() {
    // Copied from dreamdrive64 codebase
    // sd_card_t *pSD = sd_get_by_num(0);
	// FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	// if (FR_OK != fr) {
	// 	panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	// }

	// printf("\n\n---- read /%s -----\n", verify_data_filename);

	// fr = f_open(&verify_data_fil, verify_data_filename, FA_OPEN_EXISTING | FA_READ);
	// if (FR_OK != fr && FR_EXIST != fr) {
	// 	panic("f_open(%s) error: %s (%d)\n", verify_data_filename, FRESULT_str(fr), fr);
	// }

	// FILINFO filinfo;
	// fr = f_stat(verify_data_filename, &filinfo);
	// printf("%s [size=%llu]\n", filinfo.fname, filinfo.fsize);
 }

 // This code was copied from dreamdrive64. Removed anything relating to PSRAM since 
 // we aren't doing anything with that here. 
 // We can likely write to some kind of buffer 
 void sdcard_read_test(char* filename) {
    sd_card_t *pSD = sd_get_by_num(0);
	FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	if (FR_OK != fr) {
		panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	}

    sleep_ms(10);
    printf("Mounted!\n");

	FIL fil;

	printf("\n\n---- read /%s -----\n", filename);

    printf("Open file...\n");
	fr = f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ);
	if (FR_OK != fr && FR_EXIST != fr) {
		panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
	}

	FILINFO filinfo;
	fr = f_stat(filename, &filinfo);
	printf("%s [size=%llu]\n", filinfo.fname, filinfo.fsize);

    
	int len = 0;
	int total = 0;
    volatile bool isFirstRead = true;
	uint64_t t0 = to_us_since_boot(get_absolute_time());
	do {
        fr = f_read(&fil, buf, sizeof(buf), &len);        
        // write to psram..... not useful here so removed
        // do something with the read data.... so write to a data buffer or something?
		
        total += len;
	} while (len > 0);

	uint64_t t1 = to_us_since_boot(get_absolute_time());
	uint32_t delta = (t1 - t0) / 1000;
	uint32_t kBps = (uint32_t) ((float)(total / 1024.0f) / (float)(delta / 1000.0f));

	printf("Read %d bytes in %d ms (%d kB/s)\n\n\n", total, delta, kBps);

	fr = f_close(&fil);
	if (FR_OK != fr) {
		printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
	}
	printf("---- read file done -----\n\n\n");

 }