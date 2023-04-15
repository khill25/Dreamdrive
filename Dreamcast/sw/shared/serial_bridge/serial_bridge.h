/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

#pragma once

// Dat pins are consecutive so only the first one needs to be specified
// ctrl1 and ctrl2 are the signal line gpio pins
//
// start_as_tx = true: This mcu will be setup to send info and only swap to rx when requested
// start_as_tx = false: This mcu will be setup as rx to start
void interconnect_init(int ctrl1, int ctrl2, int dat0, bool start_as_tx);

void interconnect_set_direction(bool isTx);

// Send data - Blocking
void interconnect_tx(uint8_t* buf, int len);
// Send a single 16bit value - Blocking
void interconnect_tx16(uint16_t value); 

bool interconnect_rx_buffer_has_data();
bool interconnect_rx_has_data();
uint8_t interconnect_rx_get(); // Read 8 bits of the rx buffer