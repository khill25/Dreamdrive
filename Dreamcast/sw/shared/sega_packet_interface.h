/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

#include "pico/stdlib.h"

#define POWER_ON_OR_HARDWARE_RESET  0
#define SPI_SOFT_RESET              1
#define ATA_SRST_RESET              2

/* ATA Stuff */
// Commands
#define ATA_CMD_NOP                         (0x00)
#define ATA_CMD_SOFT_RESET                  (0x08)
#define ATA_CMD_PACKET_COMMAND              (0xA0)
#define ATA_CMD_IDENTIFY_DEVICE             (0xA1)
#define ATA_CMD_EXECUTE_DEVICE_DIAGNOSTIC   (0x90)
#define ATA_CMD_SET_FEATURES                (0xEF)

typedef enum ATA_TASK_FILE_REGISTER_INDEX {
    ATA_TFR_STATUS = 0,
    ATA_TFR_ERROR,
    ATA_TFR_SECTOR_COUNT,
    ATA_TFR_SECTOR_NUMBER,
    ATA_TFR_CYLINDER_LOW,
    ATA_TFR_CYLINDER_HIGH,
    ATA_TFR_DRIVE_HEAD,
    ATA_TFR_REGISTER_COUNT,
} ATA_TASK_FILE_REGISTER_INDEX;
extern uint8_t ATA_task_file_register[ATA_TFR_REGISTER_COUNT];

/* Sega Packet Interface stuff */
typedef enum SPI_REGISTER_INDEX {
    SPI_ATA_IO_REGISTER_INDEX = 0       ,
    SPI_STATUS_REGISTER_INDEX           ,
    SPI_ALTERNATE_STATUS_REGISTER_INDEX ,
    SPI_COMMAND_REGISTER_INDEX         ,
    SPI_BYTE_COUNT_REGISTER_LOW_INDEX      , // Low bits
    SPI_BYTE_COUNT_REGISTER_HIGH_INDEX      , // high bits
    SPI_DATA_REGISTER_INDEX             , // use `SPI_data_register` to access this register
    SPI_DEVICE_CONTROL_REGISTER_INDEX   ,
    SPI_DRIVE_SELECT_REGISTER_INDEX     ,// ATA Drive/Head register
    SPI_ERROR_REGISTER_INDEX            ,
    SPI_FEATURES_REGISTER_INDEX         ,
    SPI_INTERRUPT_REASON_REGISTER_INDEX , // Read only
    SPI_SECTOR_COUNT_REGISTER_INDEX     , // Write only
    SPI_SECTOR_NUMBER_REGISTER_INDEX    , // ATA Sector Number Register
    SPI_REGISTER_COUNT // 14 = (0xE)
} SPI_REGISTER_INDEX;
extern uint8_t SPI_registers[SPI_REGISTER_COUNT];
extern uint16_t SPI_data_register; // since this is the only 16 bit register...

/*
 * Bit index enums for various registers that have specific bit indexing
 */
typedef enum {
    SPI_STATUS_CHECK            = 0,
    SPI_STATUS_Reserved         = 1,
    SPI_STATUS_CORR             = 2,
    SPI_STATUS_DRQ              = 3, // Data request
    SPI_STATUS_DSC              = 4, // Data seek
    SPI_STATUS_DF               = 5, // Drive fault
    SPI_STATUS_DRDY             = 6, // Drive ready (able to respond to ATA command)
    SPI_STATUS_BSY              = 7,
} SPI_STATUS_REGISTER;

typedef enum {
    SPI_ALTERNATE_STATUS_CHECK            = 0,
    SPI_ALTERNATE_STATUS_Reserved         = 1,
    SPI_ALTERNATE_STATUS_CORR             = 2,
    SPI_ALTERNATE_STATUS_DRQ              = 3,
    SPI_ALTERNATE_STATUS_DSC              = 4,
    SPI_ALTERNATE_STATUS_DF               = 5,
    SPI_ALTERNATE_STATUS_DRDY             = 6,
    SPI_ALTERNATE_STATUS_BSY              = 7,
} SPI_ALTERNATE_STATUS_REGISTER;

typedef enum {
    SPI_DEVICE_CONTROL_Zero = 0, // always 0?
    SPI_DEVICE_CONTROL_nIEN = 1, // Sets up interrupt for the host
    SPI_DEVICE_CONTROL_SRST = 2, // Software reset from the host
    SPI_DEVICE_CONTROL_One  = 3, // always 1?
    // rest of the bits are reserved
} SPI_DEVICE_CONTROL_REGISTER;

typedef enum {
    // LUN = Logical unit number, 4 bits
    SPI_DRIVE_SELECT_LUN_LSB    = 0,
    SPI_DRIVE_SELECT_Zero       = 4, // always 0?
    SPI_DRIVE_SELECT_One        = 5,
    // 6 is reserved
    SPI_DRIVE_SELECT_One2       = 7,
} SPI_DRIVE_SELECT_REGISTER;

typedef enum {
    SPI_ERROR_ILI              = 0, // Command length is not correct
    SPI_ERROR_EMOF             = 1, // End of media detected
    SPI_ERROR_ABRT             = 2, // Drive is not ready and command is invalid
    SPI_ERROR_MCR              = 3, // Media change request, media may have been ejected
    SPI_ERROR_SENSE_KEY_LSB    = 4, // Sense key LSB
} SPI_ERROR_REGISTER;
#define SPI_SENSE_KEY_NO_SENSE                  (0x0)
#define SPI_SENSE_KEY_RECOVERED_ERROR           (0x1)
#define SPI_SENSE_KEY_NO_SENSE_NOT_READY        (0x2)
#define SPI_SENSE_KEY_NO_SENSE_MEDIUM_ERROR     (0x3)
#define SPI_SENSE_KEY_NO_SENSE_HARDWARE_ERROR   (0x4)
#define SPI_SENSE_KEY_NO_SENSE_ILLEGAL_REQUEST  (0x5)
#define SPI_SENSE_KEY_NO_SENSE_UNIT_ATTENTION   (0x6)
#define SPI_SENSE_KEY_NO_SENSE_DATA_PROTECT     (0x7)
#define SPI_SENSE_KEY_NO_SENSE_ABORTED_COMMAND  (0xB)

typedef enum {
    SPI_FEATURES_DMA_OR_FEATURE_NUMBER_LSB  = 0, // 1 = DMA, 0 = PIO
    SPI_FEATURES_FEATURE_NUMBER_MSB         = 6, // Feature number is 7 bits, bits 0-6
    SPI_FEATURES_FEATURE_SET_CLEAR          = 7, // set = 1, clear = 0
} SPI_FEATURES_REGISTER;
#define SPI_FEATURES_REGISTER_SET_MODE  (0x83) // 0x83: Set = 1, Feature Number = 3, Transfer mode is then specified in the Sector Count Register

typedef enum {
    SPI_INTERRUPT_REASON_CoD  = 0, // 0 = data, 1 = command
    SPI_INTERRUPT_REASON_IO = 1
} SPI_INTERRUPT_REASON_REGISTER;
#define SPI_INTERRUPT_REASON_CMD_PACKET_RDY_TO_RX   (0x3) // 011b Command packet can be received
#define SPI_INTERRUPT_REASON_MSG_RDY_TO_TX          (0x7) // 111b Message can be sent from device to host
#define SPI_INTERRUPT_REASON_DATA_RDY_TO_TX         (0x6) // 110b Data can be sent to host
#define SPI_INTERRUPT_REASON_DATA_RDY_TO_RX         (0x2) // 010b Data can be received from host
#define SPI_INTERRUPT_REASON_STATUS_COMPLETION      (0x5) // 101b Status register completion status

typedef enum {
    SPI_SECTOR_COUNT_MODE_VALUE_LSB = 0,
    SPI_SECTOR_COUNT_TRANSFER_MODE_LSB = 4,
} SPI_SECTOR_COUNT_REGISTER;    
#define SPI_SECTOR_COUNT_TRANSFER_MODE_PIO_DEFAULT      (0x0)  // 0000000xb 
#define SPI_SECTOR_COUNT_TRANSFER_MODE_PIO_FLOW_CONTROL (0x08) // 00001xxxb 
#define SPI_SECTOR_COUNT_TRANSFER_MODE_SINGLE_WORD_DMA  (0x10) // 00010xxxb 
#define SPI_SECTOR_COUNT_TRANSFER_MODE_MULTI_WORD_DMA   (0x40) // 00100xxxb 
#define SPI_SECTOR_COUNT_TRANSFER_MODE_PSEUDO_DMA       (0x18) // 00011xxxb apparently reserved?

typedef enum {
    SPI_SECTOR_NUMBER_STATUS_LSB = 0, // 4 bits
    SPI_SECTOR_NUMBER_DISC_FORMAT_LSB = 4, // 4 bits
} SPI_SECTOR_NUMBER_REGISTER;

/*
 * SPI(Sega Packet Interface) commands are 12 bytes
 */

// Byte 0 is the command code
// Byte 1-11 are parameters or empty
extern uint8_t SEGA_PACKET_CMD_REGISTER[12];

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

// uint8_t SEGA_PACKET_CMD_TEST_UNIT[12] = {
//     TEST_UNIT_SEGA_PACKET_CMD,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00
// };

// uint8_t SEGA_PACKET_CMD_REQ_STAT[12] = {
//     REQ_STAT_SEGA_PACKET_CMD,
//     0x00,
//     0x00, // byte[2] - starting address, always even
//     0x00,
//     0x00, // byte[4] - allocation length
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00
// };

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
// uint8_t SEGA_PACKET_REQ_STAT_INFO[10] = {
//     0x00, // STATUS in lower 4 bits (no command in )
//     0x00, // Upper 4 bits = Disc format, Lower 4 bits = Repeat Count
//     0x00, // Upper 4 bits = Address, lower 4 = Control, NOTE: "Control address byte of subcode Q (first byte)" (whatever this means...)
//     0x00, // TNO, Subcode Q track number (binary value, not BCD)
//     0x00, // X, Subcode Q index number (binary value, not BCD)
//     0x00, // FAD = Frame Address, (based on subcode Q for CD-DA and header information for CD-ROM)
//     0x00, // FAD
//     0x00, // FAD
//     0x00, // Max Read Error Retry Times, cleared to 0 when read
//     0x00,
// };

// Gets CD block mode information
// uint8_t SEGA_PACKET_CMD_REQ_MODE[12] = {
//     REQ_MODE_SEGA_PACKET_CMD,
//     0x00,
//     0x00, // byte[2] - starting address, always even
//     0x00,
//     0x00, // byte[4] - allocation length
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
// };

// Sets CD block mode information
// uint8_t SEGA_PACKET_CMD_SET_MODE[12] = {
//     SET_MODE_SEGA_PACKET_CMD,
//     0x00,
//     0x00, // byte[2] - starting address, always even
//     0x00,
//     0x00, // byte[4] - allocation length
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
// };

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

extern uint8_t SEGA_PACKET_REQ_MODE_HARDWARE_INFO[32];
/* 
= { 0

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
 
};
*/

// uint8_t SEGA_PACKET_CMD_REQ_ERROR[] = {
//     REQ_ERROR_SEGA_PACKET_CMD,
//     0x00,
//     0x00,
//     0x00,
//     0x00, // byte[4] - allocation length
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
// };

// uint8_t SEGA_PACKET_REQ_ERROR_INFO[10] = { 0
// /*
//  * Byte 0, 0xF0 (upper 4 bits are 1s)
//  * Byte 1, 0x0
//  * Byte 2, bit 3-0, Sense Key
//  * Byte 4-7, Command specific information, if not defined by a command, the FAD where the error occured is reported 
//  * Byte 8, Additional Sense Code - TODO Appendix I
//  * Byte 9, Additional Sense Code Qualifier, TODO Appendix I
//  */
// };

// uint8_t SEGA_PACKET_CMD_GET_TOC[] = {
//     GET_TOC_SEGA_PACKET_CMD,
//     0x00, // byte[1, bit 0, Select. 0 = Single density TOC info, 1 = Double density TOC info
//     0x00,
//     0x00, // byte[3] - Allocation Length (MSB)
//     0x00, // byte[4] - Allocation Length (LSB)
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
//     0x00,
// };

#define SEGA_PACKET_ADR_CODE_NO_SUB_Q                   0x0
#define SEGA_PACKET_ADR_CODE_SUB_Q_CURR_POS             0x1
#define SEGA_PACKET_ADR_CODE_SUB_Q_MEDIA_CATALOG_NUM    0x2
#define SEGA_PACKET_ADR_CODE_SUB_Q_ISRC_CODE            0x3

extern uint8_t SEGA_PACKET_TOC_INFO[408];
// = { 0
/*
 * 0-3, Track 1 information *1
 * 4-7, Track 2 information
 * n-n+3, Track n information
 * 396-399, Start track information *2
 * 400-403, End track information *3
 * 404-407, Lead-out information *4
 */
// };

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

// uint8_t TOC_TRACK_INFO_1[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, FAD for track 1 start (MSB)
 * Byte 2, FAD for track 1 start
 * Byte 3, FAD for track 1 start (LSB)
 */
// };

// uint8_t TOC_TRACK_INFO_2[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, Start track number
 */
// };

// uint8_t TOC_TRACK_INFO_3[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, End track number
 */
// };

// uint8_t TOC_TRACK_INFO_1[4] = { 0
/*
 * Byte 0, upper 4 bits = Control, lower 4 bits = ADR
 * Byte 1, FAD for lead-out start (MSB)
 * Byte 2, FAD for lead-out start
 * Byte 3, FAD for lead-out start (LSB)
 */
// };


bool SPI_select_register(bool cs0, bool cs1, bool da2, bool da1, bool da0, bool dior, bool diow, uint8_t* ret_register, uint8_t* ret_registerIndex);
void SPI_set_BSY(bool isBusy);
void SPI_set_DRQ(bool isDataReady);
void SPI_assert_INTRQ(bool valueHigh);