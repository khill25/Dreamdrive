#pragma once

/* Sega Packet Interface stuff */
typedef enum SPI_REGISTER_INDEX {
/* 0 */    SPI_ATA_IO_REGISTER_INDEX = 0       , // TODO what is this register??? I think this isn't a real register
/* 1 */    SPI_STATUS_REGISTER_INDEX           ,
/* 2 */    SPI_ALTERNATE_STATUS_REGISTER_INDEX ,
/* 3 */    SPI_COMMAND_REGISTER_INDEX         ,
/* 4 */    SPI_BYTE_COUNT_REGISTER_LOW_INDEX      , // Low bits
/* 5 */    SPI_BYTE_COUNT_REGISTER_HIGH_INDEX      , // high bits
/* 6 */    SPI_DATA_REGISTER_INDEX             , 
/* 7 */    SPI_DEVICE_CONTROL_REGISTER_INDEX   ,
/* 8 */    SPI_DRIVE_SELECT_REGISTER_INDEX     ,// ATA Drive/Head register
/* 9 */    SPI_ERROR_REGISTER_INDEX            ,
/* 10*/    SPI_FEATURES_REGISTER_INDEX         ,
/* 11*/    SPI_INTERRUPT_REASON_REGISTER_INDEX , // Read only
/* 12*/    SPI_SECTOR_COUNT_REGISTER_INDEX     , // Write only
/* 13*/    SPI_SECTOR_NUMBER_REGISTER_INDEX    , // ATA Sector Number Register

// In the ata spec, these registers are used but named different
// sector count = interrupt reason (not sure why this is a totally different name and function)
// cylinder low = byte count low
// cylinder high = byte count high
// drive/head = drive select

/* 14*/    SPI_REGISTER_COUNT // 14 = (0xE)
} SPI_REGISTER_INDEX;
// The registers are all 8 bit except the data register. Just use uint16_t for all of them
extern uint32_t SPI_registers[SPI_REGISTER_COUNT+1];