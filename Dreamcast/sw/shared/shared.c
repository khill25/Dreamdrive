/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "shared.h"
#include "serial_bridge.h"

#include "mcu2_pins.h"

#define DEBUG_PRINT_INTERLINK 1

volatile int current_mcu = 0;

// MCU1 state variables
// volatile uint16_t mcu1_received_control_line_data_buffer[32] = {0};
// volatile uint8_t mcu1_received_control_line_data_buffer_head = 0;
// volatile uint8_t mcu1_received_control_line_data_buffer_tail = 0;
volatile bool mcu1_is_waiting_for_control_line_return = false; 
volatile bool mcu1_control_line_data_ready = false;
volatile uint16_t mcu1_received_control_line_data = 0;

// MCU2 state variables
volatile bool mcu2_fetch_control_lines;

// Dreamlink buffer and state variables
volatile uint8_t cmd_buffer[16];
volatile int cmd_buffer_index = 0;
volatile bool is_reading_cmd_header = false;
volatile uint16_t cmd_num_bytes_to_read = 0;

volatile uint16_t data_buffer[512];
volatile bool is_receiving_data = false;
volatile int data_buffer_index = 0;
volatile bool should_process_cmd_buffer = false;

volatile bool mayHaveStart = false; // janky variable to track when to actually process command data
void process_dreamlink_buffer_helper(char ch) {
        #if DEBUG_PRINT_INTERLINK == 1
        printf("%02x ", ch);
        #endif

        if (is_receiving_data) {
            ((uint8_t*)(data_buffer))[data_buffer_index] = ch;
        
            data_buffer_index++;

            // We have recieved all the data we expect, we can signal to start processing it
            if (data_buffer_index >= cmd_num_bytes_to_read) {
                should_process_cmd_buffer = true;
                data_buffer_index = 0;
            }
        } else if (is_reading_cmd_header) {
            cmd_buffer[cmd_buffer_index++] = ch;

            if (cmd_buffer_index >= COMMAND_HEADER_LENGTH) {
                cmd_num_bytes_to_read = (cmd_buffer[1] << 8) | cmd_buffer[2];

                is_reading_cmd_header = false;
                is_receiving_data = true;
                cmd_buffer_index = 0;
            }
        } else if (ch == COMMAND_START_BYTE_0 && !is_receiving_data) {
            mayHaveStart = true;
        } else if (ch == COMMAND_START_BYTE_1 && mayHaveStart && !is_receiving_data) {
            is_reading_cmd_header = true;
        }

        if (should_process_cmd_buffer) {
            should_process_cmd_buffer = false;
            // process what was sent
            char command = cmd_buffer[0];

            if (command == DREAMLINK_CMD_GET_CONTROL_LINE) {
                printf("Get control line info");
                // If MCU2 gets this command, it should send the last known control line state
                mcu2_fetch_control_lines = true;

            } else if (command == DREAMLINK_CMD_SET_CONTROL_LINE) {
                // Should be received by MCU2 only... As MCU1 is going to assert/deassert the INTRQ or DMARQ lines
                // Do it here, should be quick enough

            } else if (command == DREAMLINK_CMD_SEND_CONTROL_LINE) {
                // Should be received by MCU1 only... 
                // MCU2 will just send the control lines instead of MCU1 needing to poll??
                uint8_t* buf8 = (uint8_t*)data_buffer;
                mcu1_received_control_line_data = (buf8[1] << 8) | buf8[2]; //  because we need to dump the 0 padding
                mcu1_control_line_data_ready = true;
            }
            else {
                // not supported yet
                printf("\nUnknown command: %x\n", command);
            }

            data_buffer_index = 0;
            mayHaveStart = false;
            is_receiving_data = false;
            cmd_num_bytes_to_read = 0;
            #if DEBUG_PRINT_INTERLINK == 1
            printf("\n");
            #endif
        }
}
void process_dreamlink_buffer() {
    while (interconnect_rx_buffer_has_data()) {
        // TODO: might want to use 8bit data for dreamlink/interconnect instead of 16 bits
        // Need to see more use cases before changing...
        uint16_t halfWord = interconnect_rx_get();
        
        process_dreamlink_buffer_helper((uint8_t)(halfWord >> 8));
        process_dreamlink_buffer_helper((uint8_t)halfWord);
    }
}

// Sends an async request to fetch the last control pin values
void dreamlink_get_control_lines_cmd() {
    mcu1_is_waiting_for_control_line_return = true;
    uint8_t buf[] = { 
        COMMAND_START_BYTE_0, 
        COMMAND_START_BYTE_1, 
        DREAMLINK_CMD_GET_CONTROL_LINE, 
        0x0, 
        0x0,
        };
    interconnect_tx(buf, sizeof(buf));
}

void dreamlink_set_control_lines_cmd(bool intrq, bool dmarq) {
    uint8_t value = intrq << 1 | dmarq;
    uint8_t buf[] = { 
        COMMAND_START_BYTE_0, 
        COMMAND_START_BYTE_1,
        DREAMLINK_CMD_SET_CONTROL_LINE, 
        0x0, 
        0x1, // 1 byte
        value 
        };
    interconnect_tx(buf, sizeof(buf));
}

void dreamlink_send_control_line_data_cmd(uint16_t controlLines) {
    uint8_t buf[] = { 
        COMMAND_START_BYTE_0, 
        COMMAND_START_BYTE_1, 
        DREAMLINK_CMD_SEND_CONTROL_LINE, 
        0x0, 
        0x3, // 2 bytes + 1 padding for 16 bit alignment
        0x0, // 0 padding
        (uint8_t)(controlLines << 8),
        (uint8_t)controlLines
        };
    interconnect_tx(buf, sizeof(buf));
}