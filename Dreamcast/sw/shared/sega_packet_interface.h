/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

/*
 * SPI(Sega Packet Interface) commands are 12 bytes
 */

// Byte 0 is the command code
// Byte 1-11 are parameters or empty
uint8_t SEGA_PACKET_CMD[12] = {0};

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

uint8_t SEGA_PACKET_CMD_TEST_UNIT[12] = {
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
    0x00
};

uint8_t SEGA_PACKET_CMD_REQ_STAT[12] = {
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
    0x00
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

// 10 Bytes total
uint8_t SEGA_PACKET_REQ_STAT_INFO[10] = {
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
uint8_t SEGA_PACKET_CMD_REQ_MODE[12] = {
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

// Sets CD block mode information
uint8_t SEGA_PACKET_CMD_SET_MODE[12] = {
    SET_MODE_SEGA_PACKET_CMD,
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

/*************************************************/
/*********** REQ/SET MODE HARDWARE INFO **********/
/*************************************************/

#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_MAX         0x0
#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_STANDARD    0x1
#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_2X          0x2
#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_4X          0x3
#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_6X          0x4
#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_8X          0x5
#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_10X         0x6
#define REQ_MODE_HARDWARE_INFO_CDROM_SPEED_12X         0x7

uint8_t SEGA_PACKET_REQ_MODE_HARDWARE_INFO[32] = { 0
/*
 * Byte 0-1         0x00
 * Byte 2           CD-ROM Speed
 * Byte 3           0x00
 * Byte 4-5         Standby Time, 0 = unlimited, value 0x1 to 0xFFFF
 * Byte 6           
 *      bit 5 = Read Continuous, 1 = playback without delay, data stream may contain errors
 *      bit 4 = ECC, 1 = perform retry if an error occurs. Valid only in mode 1 or Mode 2 Form 1
 *      bit 3 = Read Retry, 1 = retry on error
 *      bit 0 = Form2 Read Retry, 1 = read retry for Mode 2 Form 2
 * Byte 7-8         0x00
 * Byte 9           Read Retry Times, Same sector read retry, 0 = no retry, default is 0x8
 * Byte 10-17       Drive Information (ASCII), Name of drive, READ ONLY
 * Byte 18-25       System Version (ASCII), CD block version, READ ONLY
 * Byte 26-31       System Date (ASCII), Updated date of CD block, READ ONLY
 */
};

uint8_t SEGA_PACKET_CMD_REQ_ERROR[] = {
    REQ_ERROR_SEGA_PACKET_CMD,
    0x00,
    0x00,
    0x00,
    0x00, // byte[4] - allocation length
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

uint8_t SEGA_PACKET_REQ_ERROR_INFO[10] = { 0
/*
 * Byte 0, 0xF0 (upper 4 bits are 1s)
 * Byte 1, 0x0
 * Byte 2, bit 3-0, Sense Key
 * Byte 4-7, Command specific information, if not defined by a command, the FAD where the error occured is reported 
 * Byte 8, Additional Sense Code - TODO Appendix I
 * Byte 9, Additional Sense Code Qualifier, TODO Appendix I
 */
};

uint8_t SEGA_PACKET_CMD_GET_TOC[] = {
    GET_TOC_SEGA_PACKET_CMD,
    0x00, // byte[1, bit 0, Select. 0 = Single density TOC info, 1 = Double density TOC info
    0x00,
    0x00, // byte[3] - Allocation Length (MSB)
    0x00, // byte[4] - Allocation Length (LSB)
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

#define SEGA_PACKET_ADR_CODE_NO_SUB_Q                   0x0
#define SEGA_PACKET_ADR_CODE_SUB_Q_CURR_POS             0x1
#define SEGA_PACKET_ADR_CODE_SUB_Q_MEDIA_CATALOG_NUM    0x2
#define SEGA_PACKET_ADR_CODE_SUB_Q_ISRC_CODE            0x3

uint8_t SEGA_PACKET_TOC_INFO[408] = { 0
/*
 * 0-3, Track 1 information *1
 * 4-7, Track 2 information
 * n-n+3, Track n information
 * 396-399, Start track information *2
 * 400-403, End track information *3
 * 404-407, Lead-out information *4
 */
};

/*
Bit 0, 
    0 = Audio data without pre-emphasis (CD-DA) At-once recorded track (CD-ROM)
    1 = Audio data with pre-emphasis (CD-DA) Packet-recorded track (CD-ROM

Bit 1, 
    0 = Digital copy prohibited 
    1 = Digital copy allowed

Bit 2, 
    0 = Audio track 
    1 = Data track

Bit 3, 
    0 = 2-channel audio 
    1 = 4-channel audio
*/

uint8_t TOC_TRACK_INFO_1[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, FAD for track 1 start (MSB)
 * Byte 2, FAD for track 1 start
 * Byte 3, FAD for track 1 start (LSB)
 */
};

uint8_t TOC_TRACK_INFO_2[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, Start track number
 */
};

uint8_t TOC_TRACK_INFO_3[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, End track number
 */
};

uint8_t TOC_TRACK_INFO_1[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, FAD for lead-out start (MSB)
 * Byte 2, FAD for lead-out start
 * Byte 3, FAD for lead-out start (LSB)
 */
};
