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
#define MCU1_PIN_D0                 (8)
#define MCU1_PIN_D1                 (9)
#define MCU1_PIN_D2                 (10)
#define MCU1_PIN_D3                 (11)
#define MCU1_PIN_D4                 (12)
#define MCU1_PIN_D5                 (13)
#define MCU1_PIN_D6                 (14)
#define MCU1_PIN_D7                 (15)
// #define MCU1_PIN_D8                 (8)
// #define MCU1_PIN_D9                 (9)
// #define MCU1_PIN_D10                (10)
// #define MCU1_PIN_D11                (11)
// #define MCU1_PIN_D12                (12)
// #define MCU1_PIN_D13                (13)
// #define MCU1_PIN_D14                (14)
// #define MCU1_PIN_D15                (15)

// READ/WRITE
#define MCU1_PIN_READ				(5)
#define MCU1_PIN_WRITE				(6)
#define MCU1_PIN_IORDY              (7)

// // Signal lines
// #define MCU1_PIN_INTRQ				(18)
// #define MCU1_PIN_DMACK				(19)
// #define MCU1_PIN_DMARQ				(20)

// // Redine of the pins that are connected to the mux
// #define MCU1_PIN_MUX_0				(0)
// #define MCU1_PIN_MUX_1				(1)
// #define MCU1_PIN_MUX_2				(2)
// #define MCU1_PIN_MUX_3				(3)
// #define MCU1_PIN_MUX_4				(4)

// SD Card
#define MCU1_PIN_SD_CLK             (21)
#define MCU1_PIN_SD_CMD             (22)
#define MCU1_PIN_SD_D0              (23)
#define MCU1_PIN_SD_D1              (24)
#define MCU1_PIN_SD_D2              (25)
#define MCU1_PIN_SD_D3              (26)

// gpio 26 is used to tell other rp2040 that we want to 
// read/write to the dreamcast
// gpio 27 is used in conjuction with 26 when we are strobing
// data in and out of the pins
#define MCU1_DATABUS_D0               (8)
#define MCU_DATABUS_DEVICE_SIGNAL_PIN (16)
#define MCU_DATABUS_DEVICE_WRITE_PIN  (17)

// UART comms to MCU2
#define MCU1_PIN_PIO_COMMS_D0       (28) // RX
#define MCU1_PIN_PIO_COMMS_D1       (29) // TX


#define READ_WRITE_PIN_MASK         (0x60)
#define READ_PIN_MASK               (0x40)
#define WRITE_PIN_MASK              (0x20)
#define READ_WRITE_CS_PIN_MASK      (0x78)
#define CS_PINS_MASK                (0x18)

#define REGISTER_PIN_MASK           (0x1F)

