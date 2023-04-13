/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

// Cache the control line values
#define CACHED_PORT_FUNCTION_REGISTER_CS0  (0x0)
#define CACHED_PORT_FUNCTION_REGISTER_CS1  (0x1)
#define CACHED_PORT_FUNCTION_REGISTER_DA2  (0x2)
#define CACHED_PORT_FUNCTION_REGISTER_DA1  (0x3)
#define CACHED_PORT_FUNCTION_REGISTER_DA0  (0x4)
#define CACHED_PORT_FUNCTION_REGISTER_DIOR (0x5)
#define CACHED_PORT_FUNCTION_REGISTER_DIOW (0x6)
#define CACHED_CONTROL_LINE_REGISTER_IORDY (0x7);
#define CACHED_CONTROL_LINE_REGISTER_INTRQ (0x8);
#define CACHED_CONTROL_LINE_REGISTER_DMARQ (0x9);
#define CACHED_CONTROL_LINE_REGISTER_DMACK (0xA);
// This register is for local use and not part of the sega packet interface 
extern volatile uint16_t cached_control_line_register;

// TODO stub out some methods to read and write data based on control lines