/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

#define MCU2_CONTROL_LINE_PIN_MASK (0xFFFF)
#define MCU2_REGISTER_SELECT_PIN_MASK (0x1F)
#define MCU2_REGISTER_AND_RD_WR_SELECT_PIN_MASK (0x7F)

// CD Audio Lines, Dreamcast custom sound processor
#define MCU2_PIN_CD_CLK             (0) // Main Clock 33.8688MHz clock signal for AICA
#define MCU2_PIN_CDLRCK             (1)  // Left/right clock, digital audio left/right discriminat signal
#define MCU2_PIN_CD_BCK             (2) // Clock for fetching digital audio
#define MCU2_PIN_CD_SDAT            (3) // Digital Audio data
#define MCU2_PIN_CD_EMPH            (4) // Emphasis - ?? Low on power on, reset, playback stopped, lid open, and no disc

#define MCU2_PIN_PIO_COMMS_D0       (5) // TX
#define MCU2_PIN_PIO_COMMS_D1       (6) // RX

// CURRENTLY these pins aren't connected to anything... NEXT BOARD - > please at least break them out
#define MCU2_PIN_PIO_0				(7)
#define MCU2_PIN_PIO_1				(8)
#define MCU2_PIN_PIO_2				(9)
#define MCU2_PIN_PIO_3				(10)
#define MCU2_PIN_PIO_4				(11)
#define MCU2_PIN_PIO_5				(12)
#define MCU2_PIN_PIO_6				(13)
#define MCU2_PIN_PIO_7				(14)

#define MCU2_PIN_IORDY              (15)

#define MCU2_PIN_UART_TX            (16)
#define MCU2_PIN_UART_RX            (17)
