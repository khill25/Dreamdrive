/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/dport_access.h"

#include "sega_packet_interface.h"

// 8 Control lines (including IORDY since it seems useful and sort of needed?)
#define CONTROL_LINE_MASK       (0x7F0)
#define CS_PINS_MASK            (0x180)
#define CONTROL_REG_PINS_MASK   (0x1F0)
#define READ_WRITE_PIN_MASK     (0x600)
#define A0_PIN              (4)
#define A1_PIN              (5)
#define A2_PIN              (6)
#define CS0_PIN             (7)
#define CS1_PIN             (8)
#define RD_PIN              (9)
#define WR_PIN              (10)

#define IORDY_PIN_MASK      (0x4000000000)
#define IORDY_PIN           (38)

// 8 lines for the interconnect
#define DATA_PIN_MASK       (0x7F800) // shift 11 to do easy data inout
#define D0_PIN              (11)
#define D1_PIN              (12)
#define D2_PIN              (13)
#define D3_PIN              (14)
#define D4_PIN              (15)
#define D5_PIN              (16)
#define D6_PIN              (17)
#define D7_PIN              (18)

// Interconnect bus. 
// 8 bit bidirectional bus
// 3 control lines
// 11 total lines
#define MCU_INTERCONNECT_BUS_MASK       (0x7F800)
#define MCU_INTERCONNECT_PIN_MASK       (0x200003)
#define MCU_IRQ_LOW_PIN                 (1)
#define MCU_IRQ_HIGH_PIN                (2)
#define MCU_INTERCONNECT_DIRECTION_PIN  (21)

/*
1 output
2 output
4-10 input
11-18 bidirectional

0x7F803 output mask
0x7FFF0 input mask
*/

#define INPUT_PIN_MASK      (0x7FFF0)
#define OUTPUT_PIN_MASK     (0x7F803)

// MCU1
// 8 data lines
// 8 dreamcast control lines
// 3 interconnect control lines
// 3 for irq, dmarq, dmack
// 5 lines for cd audio
// 5 for sd card (3 if using spi)
// 32 total pins?

// MCU2 (mux)
// 8 Interconnect lines
// 3 interconnect contol lines
// 16 data lines
// 27 pins


// Using gpio 4,5,6,7,8,9,10
// Connected to 
// a0, a1, a2, cs0, cs1, rd, wr

uint32_t SPI_registers[SPI_REGISTER_COUNT+1];
uint32_t* registerIndex_map[128] = {0};
volatile uint32_t* status_register = 0;
volatile uint32_t* selectedRegister = 0;
volatile uint32_t register_index = SPI_REGISTER_COUNT;
volatile uint32_t writtenRegisters[10000] = {0};
volatile uint32_t writtenRegisterIndex = 0;

void app_main(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = CONTROL_LINE_MASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_config_t io_conf1 = {};
    io_conf1.intr_type = GPIO_INTR_DISABLE;
    io_conf1.mode = GPIO_MODE_OUTPUT;
    io_conf1.pin_bit_mask = IORDY_PIN_MASK;
    io_conf1.pull_down_en = 0;
    io_conf1.pull_up_en = 0;
    gpio_config(&io_conf1);

    gpio_config_t io_databus_conf = {};
    io_databus_conf.intr_type = GPIO_INTR_DISABLE;
    io_databus_conf.mode = GPIO_MODE_OUTPUT;
    io_databus_conf.pin_bit_mask = DATA_PIN_MASK;
    io_databus_conf.pull_down_en = 0;
    io_databus_conf.pull_up_en = 0;
    gpio_config(&io_databus_conf);

    gpio_config_t io_interconnect_conf = {};
    io_interconnect_conf.intr_type = GPIO_INTR_DISABLE;
    io_interconnect_conf.mode = GPIO_MODE_OUTPUT;
    io_interconnect_conf.pin_bit_mask =  MCU_INTERCONNECT_PIN_MASK;
    io_interconnect_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_interconnect_conf.pull_up_en = 0;
    gpio_config(&io_interconnect_conf);
    //GPIO.enable_w1ts = output
    //GPIO.enable_w1tc = input

    gpio_set_level(MCU_INTERCONNECT_DIRECTION_PIN, 0);
    gpio_set_level(MCU_IRQ_LOW_PIN, 0);
    gpio_set_level(MCU_IRQ_HIGH_PIN, 0);

    volatile uint32_t readWriteLineValues = 0;
    volatile uint32_t register_index = 0;
    volatile uint32_t csLineValue = 0;

    // This is used to quickly get the right register, read/write should be handled by whatever is doing the lookup
	// Register Index = Bits = W, R, CS1, CS0, A2, A1, A0 (most->least)
	registerIndex_map[0x4E] = &SPI_registers[SPI_STATUS_REGISTER_INDEX];//&SPI_registers[SPI_ALTERNATE_STATUS_REGISTER_INDEX]; // read
	registerIndex_map[0x2E] = &SPI_registers[SPI_DEVICE_CONTROL_REGISTER_INDEX]; // write

	registerIndex_map[0x50] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // read
	registerIndex_map[0x30] = &SPI_registers[SPI_DATA_REGISTER_INDEX]; // write

	registerIndex_map[0x31] = &SPI_registers[SPI_FEATURES_REGISTER_INDEX]; // read
	registerIndex_map[0x51] = &SPI_registers[SPI_ERROR_REGISTER_INDEX]; // write

    registerIndex_map[0x32] = &SPI_registers[SPI_SECTOR_COUNT_REGISTER_INDEX]; // write

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

    /*
    I was using the sega packet interface docs which don't include some of the registers that ARE VALID
    making me think I was crazy
    */

	// Setup a pointer to the status register
	status_register = &SPI_registers[SPI_STATUS_REGISTER_INDEX];

	SPI_registers[SPI_STATUS_REGISTER_INDEX] = 0x40; //0x48 is DRDY and DRQ bits, 0x40 is just DRDY
	SPI_registers[SPI_DRIVE_SELECT_REGISTER_INDEX] = 0xA0; // set drive select
    SPI_registers[SPI_INTERRUPT_REASON_REGISTER_INDEX] = 0x00; // 

	// For the rest of the values, just use a dump register
	for(int i = 0; i < 128; i++) {
		if (registerIndex_map[i] == 0) {
			registerIndex_map[i] = &SPI_registers[SPI_REGISTER_COUNT]; // Use the SPI_REGISTER_COUNT as a dump register
		}
	}

    vTaskDelay(4000 / portTICK_PERIOD_MS);
    printf("Boot dreamcast...\n");
    
    while(!gpio_get_level(CS0_PIN)){
        // busy wait
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }; // loop until the cs lines are active (really only useful when powering the board on before the console)

	printf("Dreamcast booted!\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Pull signal low to indicate we are ready to read/write
    gpio_set_level(IORDY_PIN, 0);

    // Set pins back to input
    // GPIO.enable_w1tc = DATA_PIN_MASK;

    // while(1) {
    for(int i = 0; i < 10; i++) {

		// Don't do anything until the control lines are ready
		do {
            csLineValue = REG_READ(GPIO_IN_REG) & CS_PINS_MASK;
		} while(csLineValue == CS_PINS_MASK || csLineValue == 0x00000);

        // register_index = REG_READ(GPIO_IN_REG) & CONTROL_REG_PINS_MASK;												

		// Signal is active low
		// 1 and 2 are the only valid value. Either read OR write is low, but not both high or both low
		do {
			readWriteLineValues = REG_READ(GPIO_IN_REG) & READ_WRITE_PIN_MASK;						
		} while(readWriteLineValues == READ_WRITE_PIN_MASK || readWriteLineValues == 0x0);

        // register_index = (register_index >> 4) | (csLineValue >> 4) | (readWriteLineValues >> 4);
        register_index = (REG_READ(GPIO_IN_REG) & CONTROL_LINE_MASK) >> 4;
        
        // get the pointer to the selected register
        selectedRegister = registerIndex_map[register_index];
		writtenRegisters[writtenRegisterIndex++] = register_index;

		// READ - SEND data to dreamcast
		if (readWriteLineValues == 0x400) {
            // Change gpio to output													
            GPIO.enable_w1ts = DATA_PIN_MASK;

            // Set register values on lines
            writtenRegisters[writtenRegisterIndex++] = 0xFFFF;

            //shift in the high 8 bits
            gpio_set_level(MCU_IRQ_LOW_PIN, 1);
            GPIO.out_w1ts = ((*selectedRegister >> 8) << D0_PIN);
            for(int s = 0; s < 100; s++) { __asm__ __volatile__ ("nop"); }
            gpio_set_level(MCU_IRQ_LOW_PIN, 0);

            // then shift low 8 bits
            gpio_set_level(MCU_IRQ_LOW_PIN, 1);
            GPIO.out_w1ts = ((*selectedRegister) << D0_PIN);
            for(int s = 0; s < 100; s++) { __asm__ __volatile__ ("nop"); }
            gpio_set_level(MCU_IRQ_LOW_PIN, 0);

            // Set pins back to input
			GPIO.enable_w1tc = DATA_PIN_MASK;

            gpio_set_level(IORDY_PIN, 1);

			// wait for latch?
			while(gpio_get_level(RD_PIN) == 0) { ; };
			
            gpio_set_level(IORDY_PIN, 0);

		// WRITE - GET data from dreamcast into register
		} else {

            // set mcu interconnect to read from the dreamcast
            // gpio_set_level(MCU_INTERCONNECT_DIRECTION_PIN, 1);
            
            // Wait until the mcu pulls this pin high
            // while(gpio_get_level(MCU_IRQ_HIGH_PIN) == 0) { ; };
            
            // shift in the low 8 bits
            gpio_set_level(MCU_IRQ_HIGH_PIN, 1);
            for(int s = 0; s < 100; s++) { __asm__ __volatile__ ("nop"); }
            *selectedRegister = (REG_READ(GPIO_IN_REG) & DATA_PIN_MASK) >> D0_PIN;
            gpio_set_level(MCU_IRQ_HIGH_PIN, 0);

            //shift in the high 8 bits
            gpio_set_level(MCU_IRQ_HIGH_PIN, 1);
            for(int s = 0; s < 100; s++) { __asm__ __volatile__ ("nop"); }
            *selectedRegister = (*selectedRegister) | (((REG_READ(GPIO_IN_REG) & DATA_PIN_MASK) >> D0_PIN) << 8);
            gpio_set_level(MCU_IRQ_HIGH_PIN, 0);
            

			// .. process data while we wait for latch. 
            //printf("\n%lx\t%lx\n", register_index, *selectedRegister);
            writtenRegisters[writtenRegisterIndex++] = *selectedRegister;
            
            // gpio_set_level(MCU_INTERCONNECT_DIRECTION_PIN, 0);

            gpio_set_level(IORDY_PIN, 1);

			// wait for latch?
			while(gpio_get_level(WR_PIN) == 0) { ; };

            gpio_set_level(IORDY_PIN, 0);
		}

        // gpio_set_level(IORDY_PIN, 0);

	}

    printf("Control line values:\n");
    for(int i = 0; i < writtenRegisterIndex; i++) {
        printf("%lx\n", writtenRegisters[i]);
    }
}
