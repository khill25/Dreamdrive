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
void setup_sega_pio_programs();
void start_sega_pio_programs();