/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

// TODO now that I understand the control lines better
// it would be nice to keep them together. 
// CD audio pins grouped
// THEN the clock line for the custom sound chip (AICA)

#define MCU2_CONTROL_LINE_PIN_MASK (0xFFFF)
#define MCU2_REGISTER_SELECT_PIN_MASK (0x1F)

// Control lines
#define MCU2_PIN_A0                 (0)
#define MCU2_PIN_A1                 (1)
#define MCU2_PIN_A2                 (2)
#define MCU2_PIN_IDE_CS0            (3)
#define MCU2_PIN_IDE_CS1            (4)
// #define MCU2_PIN_READ               (5) // moved to MCU1 pin 28
// #define MCU2_PIN_WRITE              (6) // moved to MCU1 pin 29
#define MCU2_PIN_IORDY              (7)
#define MCU2_PIN_INTRQ              (8)
#define MCU2_PIN_DMARQ              (12)
#define MCU2_PIN_DMACK              (13)

// Dreamcast custom sound processor 
#define MCU2_PIN_CD_CLK             (10) // Main Clock 33.8688MHz clock signal for AICA

// CD Audio
#define MCU2_PIN_CDLRCK             (9)  // Left/right clock, digital audio left/right discriminat signal
#define MCU2_PIN_CD_EMPH            (11) // Emphasis - ?? Low on power on, reset, playback stopped, lid open, and no disc
#define MCU2_PIN_CD_SDAT            (14) // Digital Audio data
#define MCU2_PIN_CD_BCK             (15) // Clock for fetching digital audio

#define MCU2_PIN_UART_TX            (16)
#define MCU2_PIN_UART_RX            (17)

#define MCU2_PIN_PIO_COMMS_CTRL1    (22)
#define MCU2_PIN_PIO_COMMS_CTRL2    (24)
#define MCU2_PIN_PIO_COMMS_D0       (26)
#define MCU2_PIN_PIO_COMMS_D1       (27)
#define MCU2_PIN_PIO_COMMS_D2       (28)
#define MCU2_PIN_PIO_COMMS_D3       (29)

// TODO define which pins are input, output, or both
