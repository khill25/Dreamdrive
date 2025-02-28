/*
 * sd_utils.c
 * 
 * Utility functions for the SD card
 */

 #include <stdio.h>
#include "pico/stdlib.h"
#include "sd_utils.h"

// TODO
// * Create a read buffer.... so we can dma? How do we read this from main to send to DC?
// * Probably want to hold on to the file handle pointer so we don't have to open it every time

// We have 520KB of total memory
// Sectors are 2048 bytes (I think?)
// SD card sectors are 512 bytes so we can read 4 sectors at a time into a single gdrom sector buffer
// nulldc uses a 32 sector buffer 



/*
//Buffer for sector reads [ dma ]
struct 
{
	uint32_t cache_index;
	uint32_t cache_size;
	uint8_t cache[2352*32];	//up to 32 sectors
} read_buff;
*/

/*
Sector size
mode 1 = 2048 bytes
mode 2 = 2336 bytes + 4 bytes as dummy??
mode 2 form 1 = 2048 bytes
mode 2 form 2 = 2324 bytes
mode 2 non-xa = 2336 bytes

Extra info...
MODE1:
    This mode consists of 2048-byte User Data a 4-byte error detection flag area 
    and a 276-byte error correction area for flag error control.

When Mode 2 has been specified, a Sub Header is also automatically transferred when data is specified in Data Select. 
Also, when the data is Form 1: 
    28 bytes are added as dummy bytes for the transmission 
When the data is Form 2: 
    4 bytes are added as dummy bytes.
It should be noted that because sector type check is not performed the error correction function does not work for the CD-ROM.
*/

const char* const test_file_path = "/crazytaxi/Crazy Taxi v1.004 (1999)(Sega)(US)[!][6S].gdi";

// /* SDIO Interface */
// static sd_sdio_if_t sdio_if = {
//     /*
//     Pins CLK_gpio, D1_gpio, D2_gpio, and D3_gpio are at offsets from pin D0_gpio.
//     The offsets are determined by sd_driver\SDIO\rp2040_sdio.pio.
//         CLK_gpio = (D0_gpio + SDIO_CLK_PIN_D0_OFFSET) % 32;
//         As of this writing, SDIO_CLK_PIN_D0_OFFSET is 30,
//             which is -2 in mod32 arithmetic, so:
//         CLK_gpio = D0_gpio -2.
//         D1_gpio = D0_gpio + 1;
//         D2_gpio = D0_gpio + 2;
//         D3_gpio = D0_gpio + 3;
//     */
//     .SDIO_PIO = pio1,
//     .CMD_gpio = 35,
//     .D0_gpio = 36,
//     .baud_rate = 125 * 1000 * 1000 / 10 // 12.5MHz?
// };

// /* Hardware Configuration of the SD Card socket "object" */
// static sd_card_t sd_card = {.type = SD_IF_SDIO, .sdio_if_p = &sdio_if};

// /**
//  * @brief Get the number of SD cards.
//  *
//  * @return The number of SD cards, which is 1 in this case.
//  */
// size_t sd_get_num() { return 1; }

// /**
//  * @brief Get a pointer to an SD card object by its number.
//  *
//  * @param[in] num The number of the SD card to get.
//  *
//  * @return A pointer to the SD card object, or @c NULL if the number is invalid.
//  */
// sd_card_t* sd_get_by_num(size_t num) {
//     printf("Getting SD card by number\n");
//     if (0 == num) {
//         // The number 0 is a valid SD card number.
//         // Return a pointer to the sd_card object.
//         return &sd_card;
//     } else {
//         // The number is invalid. Return @c NULL.
//         return NULL;
//     }
// }

 // Setup the sd card (and create a file handle?)
 void sdcard_init() {

     // See FatFs - Generic FAT Filesystem Module, "Application Interface",
    // http://elm-chan.org/fsw/ff/00index_e.html
    // sd_init_driver();

    // FATFS fs;
    // FRESULT fr = f_mount(&fs, "", 1);
    // if (FR_OK != fr) {
    //     panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    //     return;
    // }

    // FIL fil;
    // const char* const filename = "filename.txt";
    // fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    // if (FR_OK != fr && FR_EXIST != fr) {
    //     panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    //     return -1;
    // }

    // if (f_printf(&fil, "Hello, world!\n") < 0) {
    //     printf("f_printf failed\n");
    // }

    // fr = f_close(&fil);
    // if (FR_OK != fr) {
    //     printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    // }

    // f_unmount("");
 }

 // This code was copied from dreamdrive64. Removed anything relating to PSRAM since 
 // we aren't doing anything with that here. 
 // We can likely write to some kind of buffer 
 void sdcard_read_test() {

    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }

    FIL fil;
    // const char* const filename = test_file_path;
    fr = f_open(&fil, test_file_path, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        panic("f_open(%s) error: %s (%d)\n", test_file_path, FRESULT_str(fr), fr);
        return;
    }

    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }

    // sd_card_t *pSD = sd_get_by_num(0);
	// FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
	// if (FR_OK != fr) {
	// 	panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
	// }

    // sleep_ms(10);
    // printf("Mounted!\n");

	// FIL fil;

	// printf("\n\n---- read /%s -----\n", filename);

    // printf("Open file...\n");
	// fr = f_open(&fil, filename, FA_OPEN_EXISTING | FA_READ);
	// if (FR_OK != fr && FR_EXIST != fr) {
	// 	panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
	// }

	// FILINFO filinfo;
	// fr = f_stat(filename, &filinfo);
	// printf("%s [size=%llu]\n", filinfo.fname, filinfo.fsize);

    
	// int len = 0;
	// int total = 0;
    // volatile bool isFirstRead = true;
	// uint64_t t0 = to_us_since_boot(get_absolute_time());
	// do {
    //     fr = f_read(&fil, buf, sizeof(buf), &len);        
    //     // write to psram..... not useful here so removed
    //     // do something with the read data.... so write to a data buffer or something?
		
    //     total += len;
	// } while (len > 0);

	// uint64_t t1 = to_us_since_boot(get_absolute_time());
	// uint32_t delta = (t1 - t0) / 1000;
	// uint32_t kBps = (uint32_t) ((float)(total / 1024.0f) / (float)(delta / 1000.0f));

	// printf("Read %d bytes in %d ms (%d kB/s)\n\n\n", total, delta, kBps);

	// fr = f_close(&fil);
	// if (FR_OK != fr) {
	// 	printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
	// }
	// printf("---- read file done -----\n\n\n");

 }