/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Konrad Beckmann
 * Copyright (c) 2022 Kaili Hill
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/clocks.h"
#include "hardware/structs/systick.h"
#include "hardware/vreg.h"

#include "FreeRTOS.h"
#include "task.h"

// #include "esp32_task.h"
#include "n64_cic.h"
#include "git_info.h"
// #include "led_task.h"
#include "reset_reason.h"
#include "pins_mcu2.h"
#include "utils.h"
#include "gpio_helper.h"

#include "sdcard/internal_sd_card.h"
#include "pio_uart/pio_uart.h"
#include "psram.h"
#include "qspi_helper.h"

#include "ff.h"
#include <string.h>

#define UART0_BAUD_RATE  (115200)

// Priority 0 = lowest, 31 = highest
// Use same priority to force round-robin scheduling
#define LED_TASK_PRIORITY     (tskIDLE_PRIORITY + 1UL)
#define ESP32_TASK_PRIORITY   (tskIDLE_PRIORITY + 1UL)
#define MAIN_TASK_PRIORITY    (tskIDLE_PRIORITY + 1UL)

static StaticTask_t main_task;
static StaticTask_t led_task;
static StaticTask_t esp32_task;

#define MAIN_TASK_STACK_SIZE (1024)
#define LED_TASK_STACK_SIZE (1024)
#define ESP32_TASK_STACK_SIZE (1024)

static StackType_t main_task_stack[MAIN_TASK_STACK_SIZE];
static StackType_t led_task_stack[LED_TASK_STACK_SIZE];
static StackType_t esp32_task_stack[ESP32_TASK_STACK_SIZE];

static const gpio_config_t mcu2_gpio_config[] = {
	{PIN_UART0_TX, GPIO_OUT, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_UART},
	{PIN_UART0_RX, GPIO_OUT, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_UART},

	// TODO: Configure as PIO0 probably?
	{PIN_SD_CLK, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_SD_CMD, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_SD_DAT0_UART1_TX, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_SD_DAT1_UART1_RX, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_SD_DAT2, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_SD_DAT3, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},

	// TODO: Configure as PIO1 probably?
	{PIN_ESP32_D0, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_ESP32_D1, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_ESP32_D2, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_ESP32_D3, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_ESP32_CS, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_ESP32_SCK, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},

	// Demux should be configured as inputs without pulls until we lock the bus
	{PIN_DEMUX_A0, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_DEMUX_A1, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_DEMUX_A2, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},
	{PIN_DEMUX_IE, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},

	// MCU1 RUN/RESETn pin
	{PIN_MCU1_RUN, GPIO_OUT, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},

	// WS2812 RGB LED
	{PIN_LED, GPIO_OUT, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},

	// Configure as a Clock Output clk_gpout0 after GPIO config
	{PIN_MCU2_GPIO21, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_2MA, GPIO_FUNC_SIO},

	// N64 signals
	{PIN_N64_COLD_RESET, GPIO_IN, false, false, true, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},	// Pulled down
	{PIN_N64_NMI, GPIO_IN, false, true, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},	// Pulled up, open drain
	{PIN_CIC_DIO, GPIO_IN, false, true, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},	// Pulled up
	{PIN_CIC_DCLK, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_SIO},

	// Configure as PIO that implements UART becase of the way the pins from MCU1 are connected to MCU2
	{PIN_SPI1_SCK, GPIO_IN, true, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO1}, 
	//{PIN_SPI1_TX, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO1}, // not using
	{PIN_SPI1_RX, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO1},
	{PIN_SPI1_CS, GPIO_IN, false, false, false, GPIO_DRIVE_STRENGTH_4MA, GPIO_FUNC_PIO1},
};

void main_task_entry(__unused void *params)
{
	int count = 0;
	printf("MCU2 Main Entry\n");

	// Make sure that ssi hardware is disabled before starting mcu1
	// qspi_disable();
	// ssi_hw->ssienr = 0;
	// qspi_oeover_disable();

	// test_read_psram("Resident Evil 2 (USA) (Rev 1).z64");
	// test_read_psram("Pokemon Stadium 2 (USA).z64");
	// test_read_psram("GoldenEye 007 (U) [!].z64");

	// load_new_rom("GoldenEye 007 (U) [!].z64");
	// load_new_rom("Donkey Kong 64 (U) [!].z64");
	// load_new_rom("Legend of Zelda, The - Majora's Mask (U) [!].z64");
	// load_new_rom("Perfect Dark (U) (V1.1) [!].z64");
	// load_new_rom("Resident Evil 2 (USA) (Rev 1).z64");
	// load_new_rom("Pokemon Stadium 2 (USA).z64");
	// load_new_rom("1080[en,jp].z64");
	// load_new_rom("Legend of Zelda, The - Ocarina of Time (U) (V1.2) [!].z64");
	
	vTaskDelay(100);
	printf("Booting MCU1...\n");
	gpio_put(PIN_MCU1_RUN, 1);

	printf("Mounting SD Card...");
	mount_sd();
	printf("Finished!\n");

	// Setup PIO UART
	printf("Initing MCU1<->MCU2 serial bridge...");
	pio_uart_init(PIN_SPI1_CS, PIN_SPI1_RX);
	printf("Finshed!\n");

	// mcu2_setup_verify_rom_data(); // opens the file into some global variables

	// Random test stuff, leave in for now as still heavily debugging
	// vTaskDelay(5000);
	//ddr64_load_new_rom_command("Doom 64 (USA) (Rev 1).z64");
	// load_new_rom("Doom 64 (USA) (Rev 1).z64");
	// load_new_rom("GoldenEye 007 (U) [!].z64");
	// load_new_rom("Super Mario 64 (USA).z64");

	printf("Starting main MCU2 loop\n");

	volatile uint32_t t = 0;
	volatile uint32_t t2 = 0;
	uint32_t totalBytesSinceLastPeriod = 0;
	bool isFirstVerifyDataLoop = true;
	
	while (true) {
		tight_loop_contents();

		// process the buffer look for cmd data
		mcu2_process_rx_buffer();

		if(sendDataReady) {
			send_sd_card_data();
		}

		if (startRomLoad && !romLoading) {
			romLoading = true;
			load_selected_rom();
			romLoading = false;
			startRomLoad = false;
		}

		if (start_saveEeepromData) {
			start_saveEeepromData = false;
			start_eeprom_sd_save();
		}

		if (is_verifying_rom_data_from_mcu1) {
			is_verifying_rom_data_from_mcu1 = false;
			mcu2_verify_sent_rom_data();

			if (isFirstVerifyDataLoop) {
				isFirstVerifyDataLoop = false;
				verifyDataTime = time_us_32();
			}
		}

		// NMI is pulses when the reset button is pressed.
		// Doesn't appear to toggle state until the button is released?
		// if (gpio_get(PIN_N64_NMI) != lastNMIState && justForcedCICReset == false) {
		// 	bool nowNMIState = gpio_get(PIN_N64_NMI);
		// 	printf("NMI changed state. Was: %d, now: %d\n", lastNMIState, nowNMIState);
		// 	lastNMIState = nowNMIState;
		// 	// Reset the cic with updated info
		// 	force_restart_cic = true;
		// 	justForcedCICReset = true;
		// }

		// Tick every second
		if(time_us_32() - t > 1000000) {
			t = time_us_32();
			t2++;	
			// if (t2 % 30 == 0) {
			// 	uint32_t totalDataInLastPeriod = 512 * totalSectorsRead;
			// 	uint32_t kBps = (uint32_t) ((float)(totalDataInLastPeriod / 1024.0f) / (float)(totalTimeOfSendData_ms / 1000.0f));
    		// 	printf("Sent %d bytes in %d ms (%d kB/s)\n", totalDataInLastPeriod, totalTimeOfSendData_ms, kBps);
			// }

			// if (t2 == 2) {
			// 	printf("Starting inter_mcu_comms test...\n");
			// 	inter_mcu_comms_test();
			// }

			// if (t2 % 10 == 0 && t2 != 0) {
			// 	justForcedCICReset = false;
			// }
		}
	}
}

void mcu2_core1_entry(void)
{
	printf("[Core1] CIC Starting\n");

	while (1) {
		n64_cic_run(PIN_N64_COLD_RESET, PIN_CIC_DCLK, PIN_CIC_DIO);

		// n64_cic_run returns when N64_CR goes low, i.e.
		// user presses the reset button, or the N64 loses power.

		// TODO: Perform actions when this happens. Commit RAM to flash/SD etc.

		printf("[Core1] CIC Restarting\n");
	}
}

void vLaunch(void)
{
	xTaskCreateStatic(main_task_entry, "Main", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, main_task_stack, &main_task);
	// xTaskCreateStatic(led_task_entry, "LED", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, led_task_stack, &led_task);
	// Disable the esp32 right now to avoid adding any additional variables to debug
	//xTaskCreateStatic(esp32_task_entry, "ESP32", ESP32_TASK_STACK_SIZE, NULL, ESP32_TASK_PRIORITY, esp32_task_stack, &esp32_task);

	// Start the tasks and timer.
	vTaskStartScheduler();
}

void mcu2_main(void)
{
	// const int freq_khz = 133000;
	// const int freq_khz = 166000;
	// const int freq_khz = 200000;
	// const int freq_khz = 210000;
	// const int freq_khz = 250000;
	// const int freq_khz = 266000;
	// NOTE: For speeds above 266MHz voltage must be increased.
	// const int freq_khz = 300000;
	// const int freq_khz = 332000;
	// const int freq_khz = 360000;
	// const int freq_khz = 384000;
	// const int freq_khz = 400000;

	// IMPORTANT: For the serial comms between mcus to work properly 
	// both mcus must be run at the same clk speed or have the pio divder set accordingly

	// Note that this might call set_sys_clock_pll,
	// which might set clk_peri to 48 MHz
	// vreg_set_voltage(VREG_VOLTAGE_1_25);
	// bool clockWasSet = set_sys_clock_khz(freq_khz, false);

	// Init async UART on pin 0/1
	// stdio_async_uart_init_full(DEBUG_UART, DEBUG_UART_BAUD_RATE, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
	stdio_uart_init_full(DEBUG_UART, DEBUG_UART_BAUD_RATE, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
	gpio_configure(mcu2_gpio_config, ARRAY_SIZE(mcu2_gpio_config));

	set_demux_mcu_variables(PIN_DEMUX_A0, PIN_DEMUX_A1, PIN_DEMUX_A2, PIN_DEMUX_IE);

	// printf("MCU2: Was%s able to set clock to %d MHz\n", clockWasSet ? "" : " not", freq_khz/1000);

	// Enable a 12MHz clock output on GPIO21 / clk_gpout0
	clock_gpio_init(PIN_MCU2_GPIO21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 1);

	// printf("\n\n----------------------------------------\n");
	// printf("PicoCart64 MCU2 Boot (git rev %08x)\r\n", GIT_REV);
	// printf("Reset reason: 0x%08lX\n", get_reset_reason());
	// printf("clk_sys: %d Hz\n", clock_get_hz(clk_sys));
	// printf("clk_peri: %d Hz\n", clock_get_hz(clk_peri));
	// printf("----------------------------------------\n\n");

	// for (int fkhz = 125000; fkhz < 400000;) {
	// 	uint vco, postdiv1, postdiv2;
	// 	if (check_sys_clock_khz(fkhz, &vco, &postdiv1, &postdiv2)) {
	// 		// set_sys_clock_pll(vco, postdiv1, postdiv2);
			
	// 	} else {
	// 		printf("%dMhz NOT available\n", fkhz/1000);
	// 	}
	// 	fkhz += 1000;
	// }

	// printf("\n\n");
	// uint32_t timeBuffer[32];
	// systick_hw->csr = 0x5;
    // systick_hw->rvr = 0x00FFFFFF;
	// for(int i = 0; i < 32; i++) {
	// 	uint32_t startTime_ns = systick_hw->cvr;
	// 	timeBuffer[i] = startTime_ns;
	// }

	// for(int i = 0; i < 32; i++) {
	// 	printf("%d\n", timeBuffer[i]);	
	// }

	// printf("\n\n");

	// gpio_init(PIN_DEMUX_A0);
	// gpio_set_dir(PIN_DEMUX_A0, true);
	// gpio_put(PIN_DEMUX_A0, 1);

	multicore_launch_core1(mcu2_core1_entry);

	// Start FreeRTOS on Core0
	vLaunch();

	while (true) {
		// Never reached
	}
}
