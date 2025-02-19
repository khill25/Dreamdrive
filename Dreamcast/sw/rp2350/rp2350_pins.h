/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

#define DATABUS_D0 0  // D0-D15 on GPIO 0-15
#define PIN_A0 16
#define PIN_A1 17
#define PIN_A2 18
#define PIN_CS0 19
#define PIN_CS1 20
#define PIN_RD 21
#define PIN_WR 22
#define PIN_IORDY 23
#define PIN_INTRQ 24
#define PIN_DMARQ 25
#define PIN_DMACK 26
#define PIN_CD_LRCK 27
#define PIN_CD_BCK 28
#define PIN_CD_SDAT 29
#define PIN_CDDA_CLK 30
#define PIN_CD_EMPH 31
#define PIN_DOPEN 32
#define PIN_GD_RST 33
#define PIN_SD_CLK 34
#define PIN_SD_CMD 35
#define PIN_SD_D0 36
#define PIN_SD_D1 37
#define PIN_SD_D2 38
#define PIN_SD_D3 39

// Update masks for the new pin layout
#define CS_PINS_MASK (0x180000)
#define READ_WRITE_PIN_MASK (0x600000)
#define READ_PIN_MASK (0x400000)
#define WRITE_PIN_MASK (0x200000)
#define REGISTER_PIN_MASK (0x7F0000)
#define BIT_SHIFTED_READ_PIN_MASK (0x40)
