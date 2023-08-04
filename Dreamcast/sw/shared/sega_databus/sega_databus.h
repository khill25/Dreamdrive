/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

extern uint sega_databus_handler_sm;
void setup_sega_pio_programs();
void start_sega_pio_programs();
void swap_cs_detect_for_rw_detect();