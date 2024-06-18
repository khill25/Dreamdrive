/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

// Control lines (share the same pins as data 0-4, controlled via mux select = high)
#define MCU1_PIN_A0					(0)
#define MCU1_PIN_A1					(1)
#define MCU1_PIN_A2					(2)
#define MCU1_PIN_CS0				(3)
#define MCU1_PIN_CS1				(4)

// Data lines (d0-4 shared with control lines, controlled via mux select = low)
#define MCU1_PIN_D0                 (0)
#define MCU1_PIN_D1                 (1)
#define MCU1_PIN_D2                 (2)
#define MCU1_PIN_D3                 (3)
#define MCU1_PIN_D4                 (4)
#define MCU1_PIN_D5                 (5)
#define MCU1_PIN_D6                 (6)
#define MCU1_PIN_D7                 (7)
#define MCU1_PIN_D8                 (8)
#define MCU1_PIN_D9                 (9)
#define MCU1_PIN_D10                (10)
#define MCU1_PIN_D11                (11)
#define MCU1_PIN_D12                (12)
#define MCU1_PIN_D13                (13)
#define MCU1_PIN_D14                (14)
#define MCU1_PIN_D15                (15)

// READ/WRITE
#define MCU1_PIN_READ				(16)
#define MCU1_PIN_WRITE				(17)

// Signal lines
#define MCU1_PIN_INTRQ				(18)
#define MCU1_PIN_DMACK				(19)
#define MCU1_PIN_DMARQ				(20)

// Redine of the pins that are connected to the mux
#define MCU1_PIN_MUX_0				(0)
#define MCU1_PIN_MUX_1				(1)
#define MCU1_PIN_MUX_2				(2)
#define MCU1_PIN_MUX_3				(3)
#define MCU1_PIN_MUX_4				(4)

// SD Card
#define MCU1_PIN_SD_CLK             (21)
#define MCU1_PIN_SD_CMD             (22)
#define MCU1_PIN_SD_D0              (23)
#define MCU1_PIN_SD_D1              (24)
#define MCU1_PIN_SD_D2              (25)
#define MCU1_PIN_SD_D3              (26)

// Mux select, LOW = D0-4, HIGH = Control lines
#define MCU1_PIN_MUX_SELECT         (27)

// UART comms to MCU2
#define MCU1_PIN_PIO_COMMS_D0       (28) // RX
#define MCU1_PIN_PIO_COMMS_D1       (29) // TX


// Pins 16 and 17 are used for read and write
#define READ_WRITE_PIN_MASK         (0x30000)
// Pins 3,4 and 16,17 are used for CS0, CS1, read and write
#define READ_WRITE_CS_PIN_MASK      (0x30018)
// Only pins 3,4 
#define CS_PINS_MASK                (0x18)

// This is a 16 bit value for pins 0-15
// The 16 bit value is only used when accessing the data register
// All other registers are 8 bit
#define ATA_REGISTER_PIN_MASK       (0xFFFF)
