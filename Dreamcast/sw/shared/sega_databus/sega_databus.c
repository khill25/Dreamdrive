/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#include "pico/stdlib.h"
#include "sega_databus.h"

volatile uint16_t cached_control_line_register = 0;

