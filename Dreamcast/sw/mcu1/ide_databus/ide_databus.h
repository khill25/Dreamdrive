/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

// Low level data methods
void databus_put(uint16_t data);
uint16_t databus_get();
void databus_set_dir(bool isOut);