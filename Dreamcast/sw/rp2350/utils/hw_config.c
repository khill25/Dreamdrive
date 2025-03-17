
/* hw_config.c
Copyright 2021 Carl John Kugler III

Licensed under the Apache License, Version 2.0 (the License); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an AS IS BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
*/

/*
This file should be tailored to match the hardware design.

See
https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/tree/main#customizing-for-the-hardware-configuration
*/

#include "hw_config.h"

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
//         //     .SDIO_PIO = pio1,
// //     .CMD_gpio = 35,
// //     .D0_gpio = 36,
//     */
//    .SDIO_PIO = pio1,
//     // .CLK_gpio = 34,
//     .CMD_gpio = 35,
//     .D0_gpio = 36,
//     // .D1_gpio = 37,
//     // .D2_gpio = 38,
//     // .D3_gpio = 39,
//     .baud_rate = 266 * 1000 * 1000 / 6  // 20833333 Hz
// };

// /* Hardware Configuration of the SD Card socket "object" */
// static sd_card_t sd_card = {
//     .type = SD_IF_SDIO, 
//     .sdio_if_p = &sdio_if
// };

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
//     if (0 == num) {
//         // The number 0 is a valid SD card number.
//         // Return a pointer to the sd_card object.
//         return &sd_card;
//     } else {
//         // The number is invalid. Return @c NULL.
//         return NULL;
//     }
// }

/* Configuration of hardware SPI object */
static spi_t spi = {
    .hw_inst = spi0,  // SPI component
    .sck_gpio = 34,    // GPIO number (not Pico pin number)
    .mosi_gpio = 35,
    .miso_gpio = 36,
    // .baud_rate = 266 * 1000 * 1000 / 2  // 66MHz (Not even sure if it will go that fast)
    // .baud_rate = 125 * 1000 * 1000 / 8  // 15625000 Hz
    //.baud_rate = 125 * 1000 * 1000 / 6  // 20833333 Hz
    // .baud_rate = 125 * 1000 * 1000 / 4  // 31250000 Hz
    .baud_rate = 125 * 1000 * 1000 / 2  // 62500000 Hz
};

/* SPI Interface */
static sd_spi_if_t spi_if = {
    .spi = &spi,  // Pointer to the SPI driving this card
    .ss_gpio = 39  // The SPI slave select GPIO for this SD card
};


/* Configuration of the SD Card socket object */
static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if  // Pointer to the SPI interface driving this card
};

/* ********************************************************************** */

size_t sd_get_num() { return 1; }

/**
 * @brief Get a pointer to an SD card object by its number.
 *
 * @param[in] num The number of the SD card to get.
 *
 * @return A pointer to the SD card object, or @c NULL if the number is invalid.
 */
sd_card_t *sd_get_by_num(size_t num) {
    if (0 == num) {
        // The number 0 is a valid SD card number.
        // Return a pointer to the sd_card object.
        return &sd_card;
    } else {
        // The number is invalid. Return @c NULL.
        return NULL;
    }
}

/* [] END OF FILE */
