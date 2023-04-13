/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

#define MCU1 1
#define MCU2 2

extern volatile int current_mcu;

// TODO there will likely be other variables and it might make sense
// to use a single variable and bit shift? 

// When waiting for the control line command to send data back
extern volatile bool mcu1_is_waiting_for_control_line_return;
extern volatile bool mcu1_control_line_data_ready;

// flag for MCU2 to fetch the control lines latest update
extern volatile bool mcu2_fetch_control_lines;

// Overview of a dreamlink command
// CMD_START byte 1, CMD_START byte 2
// CMD (1 byte)
// Num bytes (2 bytes)
// Data (num bytes)
#define COMMAND_HEADER_LENGTH 3
#define COMMAND_START_BYTE_0 (0xBE)
#define COMMAND_START_BYTE_1 (0xEF)

// Commands
#define DREAMLINK_CMD_GET_CONTROL_LINE      (0x01)
#define DREAMLINK_CMD_SET_CONTROL_LINE      (0x02)
#define DREAMLINK_CMD_SEND_CONTROL_LINE     (0x03)

void process_dreamlink_data();
void dreamlink_get_control_lines_cmd();
void dreamlink_set_control_lines_cmd(bool intrq, bool dmarq);
void dreamlink_send_control_line_data_cmd(uint16_t controlLines);
