/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

extern uint sega_cs0_low_sm;
extern uint sega_cs1_low_sm;
extern uint sega_bus_read_request_sm;
extern uint sega_bus_write_request_sm;

void init_cs0_low_program();
void init_cs1_low_program();
void init_bus_read_request_program();
void init_bus_write_request_program();