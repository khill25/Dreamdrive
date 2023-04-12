/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

/*
 * SPI(Sega Packet Interface) commands are 11 bytes
 */

// Byte 0 is the command code
// Byte 1-11 are parameters or empty
uint8_t SEGA_PACKET_CMD[11] = {0};

#define TEST_UNIT_SEGA_PACKET_CMD       0x00
#define REQ_STAT_SEGA_PACKET_CMD        0x10
#define REQ_MODE_SEGA_PACKET_CMD        0x11
#define SET_MODE_SEGA_PACKET_CMD        0x12
#define REQ_ERROR_SEGA_PACKET_CMD       0x13
#define GET_TOC_SEGA_PACKET_CMD         0x14
#define REQ_SES_SEGA_PACKET_CMD         0x15
#define CD_OPEN_SEGA_PACKET_CMD         0x16
#define CD_PLAY_SEGA_PACKET_CMD         0x20
#define CD_SEEK_SEGA_PACKET_CMD         0x21
#define CD_SCAN_SEGA_PACKET_CMD         0x22
#define CD_READ_SEGA_PACKET_CMD         0x30
#define CD_READ2_SEGA_PACKET_CMD        0x31
#define GET_SCD_SEGA_PACKET_CMD         0x40

uint8_t SEGA_PACKET_CMD_TEST_UNIT[] = {
    TEST_UNIT_SEGA_PACKET_CMD,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

uint8_t SEGA_PACKET_CMD_REQ_STAT[] = {
    REQ_STAT_SEGA_PACKET_CMD,
    0x00,
    0x00, // byte[2] - starting address, always even
    0x00,
    0x00, // byte[4] - allocation length
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

/*************************************/
/*********** REQ STAT INFO ***********/
/*************************************/
// Req mode status bits are placed in the lower 4 bits of the first byte.
#define REQ_STAT_INFO_STATUS_BUSY        0x00 // State transition
#define REQ_STAT_INFO_STATUS_PAUSE       0x01 
#define REQ_STAT_INFO_STATUS_STANDBY     0x02 // drive stop
#define REQ_STAT_INFO_STATUS_PLAY        0x03
#define REQ_STAT_INFO_STATUS_SEEK        0x04
#define REQ_STAT_INFO_STATUS_SCAN        0x05
#define REQ_STAT_INFO_STATUS_OPEN        0x06 // Tray is open
#define REQ_STAT_INFO_STATUS_NODISC      0x07
#define REQ_STAT_INFO_STATUS_RETRY       0x08
#define REQ_STAT_INFO_STATUS_ERROR       0x09 // Reading of disc TOC failed (state does not allow access)

// Req mode disc format = byte 1, upper 4 bits
#define REQ_STAT_INFO_DISC_FORMAT_CDDA           0x00
#define REQ_STAT_INFO_DISC_FORMAT_CDROM          0x01
#define REQ_STAT_INFO_DISC_FORMAT_CDROM_XA       0x02
#define REQ_STAT_INFO_DISC_FORMAT_CDI            0x03
#define REQ_STAT_INFO_DISC_FORMAT_GDROM          0x04

// Req mode Repeat count = byte 1, lower 4 bits.
// 0x0 to 0xE, 0xF becomes unlimited repeat

// Req mode Frame Address (FAD), bytes 5-7.
// Based on status. Home position = 0x96

// 9 Bytes total
uint8_t SEGA_PACKET_REQ_STAT_INFO[] = {
    0x00, // STATUS in lower 4 bits (no command in )
    0x00, // Upper 4 bits = Disc format, Lower 4 bits = Repeat Count
    0x00, // Upper 4 bits = Address, lower 4 = Control, NOTE: "Control address byte of subcode Q (first byte)" (whatever this means...)
    0x00, // TNO, Subcode Q track number (binary value, not BCD)
    0x00, // X, Subcode Q index number (binary value, not BCD)
    0x00, // FAD = Frame Address, (based on subcode Q for CD-DA and header information for CD-ROM)
    0x00, // FAD
    0x00, // FAD
    0x00, // Max Read Error Retry Times, cleared to 0 when read
    0x00,
};

// Gets CD block mode information
uint8_t SEGA_PACKET_CMD_REQ_MODE[] = {
    REQ_MODE_SEGA_PACKET_CMD,
    0x00,
    0x00, // byte[2] - starting address, always even
    0x00,
    0x00, // byte[4] - allocation length
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

/**********************************************/
/*********** REQ MODE HARDWARE INFO ***********/
/**********************************************/
uint8_t SEGA_PACKET_REQ_MODE_HARDWARE_INFO[32] = { 0
// Describe the structure of the packet here
// since there are many bytes
};

