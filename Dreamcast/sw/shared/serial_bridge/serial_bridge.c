/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 */

/*
 *
 * Interconnect notes:
 * MCU1 contains all the data lines to the dreamcast
 * MCU2 contains all the control lines
 * 
 * I *THINK* that we need MCU1 to be setup to listen to MCU2 as that is where
 * the actions to perform read/write on the data lines comes from.
 * 
 * I'm not sure what kind of data MCU2 may need to send, but if it turns out that cd_sdat
 * needs to stream audio info or something, we may want to change that pin to MCU1 
 * and maybe bodge wire and remove the "key" switch from the weact board, as that is the 
 * only other place we can get another gpio without going naked rp2040s.
 * 
 * So with that: 
 * MCU1 will be setup with the RX program
 * MCU2 will be setup with the TX Program
 * There should be a way to flip the comm channels as currently there is no
 * need for duplex... (unless we run into the issue in the above paragraph)
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "serial_bridge.h"
#include "serial_bridge.pio.h"
 
#define NUM_INTERCONNECT_DAT_PINS 4 // Total pins
#define NUM_INTERCONNECT_DAT_TX_PINS 4 // Pins used when tx'ing
#define NUM_INTERCONNECT_DAT_RX_PINS 4 // pins used when rx'ing

static inline void interconnect_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx_start, uint pin_ctrl, uint divider);
static inline void interconnect_rx_program_init(PIO pio, uint sm, uint offset, uint pin_rx_start, uint pin_ctrl, uint divider);
bool interconnect_rx_has_data();

typedef struct pio_interconnect_inst {
    PIO pio;
    uint sm;
    int ctrl;
    uint offset;
} pio_interconnect_inst_t;

pio_interconnect_inst_t tx_interconnect_config = {
    .pio = pio0,
    .sm = 0,
    .ctrl = -1,
    .offset = 0
};

pio_interconnect_inst_t rx_interconnect_config = {
    .pio = pio0,
    .sm = 1,
    .ctrl = -1,
    .offset = 0
};

#define RX_RING_BUFFER_SIZE 1024
typedef struct RXRingBuffer_t {                                                                  
    uint16_t buf[RX_RING_BUFFER_SIZE / sizeof(uint16_t)];
    uint32_t head;                                                   
    uint32_t tail;                                                   
} RXRingBuffer_t;
RXRingBuffer_t rxRingBuffer = {};

uint16_t rx_interconnect_fetch_half_word() {
    // 16-bit read from the uppermost byte of the FIFO, as data is left-justified
    io_rw_16 *rxfifo_shift = (io_rw_16*)&rx_interconnect_config.pio->rxf[rx_interconnect_config.sm] + 1;
    while (pio_sm_is_rx_fifo_empty(rx_interconnect_config.pio, rx_interconnect_config.sm))
        tight_loop_contents();
    return (uint16_t)*rxfifo_shift;
}


void rx_interconnect_interrupt() {
    while(interconnect_rx_has_data()) {
        // Wait for data
        //while (pio_sm_is_rx_fifo_empty(rx_interconnect_config.pio, rx_interconnect_config.sm)) { tight_loop_contents(); }

        // uint32_t word_shift = (rx_interconnect_config.pio->rxf[rx_interconnect_config.sm] >> 16);
        
        // // Fetch the data and put it in the ring buffer
        // uint16_t halfWord = (uint16_t)word_shift;

        rxRingBuffer.buf[rxRingBuffer.head++] = rx_interconnect_fetch_half_word();
        if (rxRingBuffer.head == RX_RING_BUFFER_SIZE) {
            rxRingBuffer.head = 0;
        }
    }
}

void interconnect_init(int ctrl1, int ctrl2, int dat0, bool start_as_tx) {
    if(start_as_tx) {
        uint offset = pio_add_program(tx_interconnect_config.pio, &inter_mcu_tx_program);
        tx_interconnect_config.ctrl = ctrl2;
        tx_interconnect_config.offset = offset;
        pio_gpio_init(tx_interconnect_config.pio, ctrl1);
        pio_gpio_init(tx_interconnect_config.pio, ctrl2);
        interconnect_tx_program_init(tx_interconnect_config.pio, tx_interconnect_config.sm, offset, dat0, tx_interconnect_config.ctrl, 1);
    } else {
        uint offset = pio_add_program(rx_interconnect_config.pio, &inter_mcu_rx_program);
        rx_interconnect_config.ctrl = ctrl1;
        rx_interconnect_config.offset = offset;
        pio_gpio_init(rx_interconnect_config.pio, ctrl1);
        pio_gpio_init(rx_interconnect_config.pio, ctrl2);
        interconnect_rx_program_init(rx_interconnect_config.pio, rx_interconnect_config.sm, offset, dat0, rx_interconnect_config.ctrl, 1);
        
        uint irqNum = rx_interconnect_config.pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0;
        irq_set_exclusive_handler(irqNum, rx_interconnect_interrupt);
        
	    pio_set_irq0_source_enabled(rx_interconnect_config.pio, pis_sm1_rx_fifo_not_empty, true);
        irq_set_enabled(irqNum, true);

        rxRingBuffer.head = 0;
        rxRingBuffer.tail = 0;
    }
}

void interconnect_set_direction(bool isTx) {
    // TODO implement me
}

void interconnect_tx(uint8_t* buf, int len) {
    uint16_t* buf16 = (uint16_t*)buf;
    for(int i = 0; i < len/2; i++) {
        pio_sm_put_blocking(tx_interconnect_config.pio, tx_interconnect_config.sm, buf16[i]);
    }
}

bool interconnect_rx_has_data() {
    return !pio_sm_is_rx_fifo_empty(rx_interconnect_config.pio, rx_interconnect_config.sm);
}

bool interconnect_rx_buffer_has_data() {
    return rxRingBuffer.tail != rxRingBuffer.head;
}

// Read 16 bits of the rx buffer
uint16_t interconnect_rx_get() {
    uint16_t ret = rxRingBuffer.buf[rxRingBuffer.tail++];
    if (rxRingBuffer.tail >= RX_RING_BUFFER_SIZE/2) {
        rxRingBuffer.tail = 0;
    }
    return ret;
} 

static inline void interconnect_tx_program_init(PIO pio, uint sm, uint offset, uint pin_tx_start, uint pin_ctrl, uint divider) {

    for (int i = pin_tx_start; i < pin_tx_start+NUM_INTERCONNECT_DAT_TX_PINS; i++) {
        pio_gpio_init(pio, i);
    }

    // 24, 26, 27, 28, 29 = output
    // 22 = input

    // Pin 22 is IN all the other pins are OUT
    pio_sm_set_pins_with_mask(pio, sm, 0x00000000, 0x3D400000);
    pio_sm_set_pindirs_with_mask(pio, sm, 0x3D000000, 0x3D400000);

    pio_sm_config c = inter_mcu_tx_program_get_default_config(offset);

    // Setup autopull, grab 16 bits at a time
    sm_config_set_out_shift(&c, true, true, 16);
    sm_config_set_in_pins(&c, 22); 
    sm_config_set_out_pins(&c, 26, 4);

    // ctrl pin (24) is used to signal we have put data on the lines
    sm_config_set_sideset_pins(&c, pin_ctrl);

    // sm_config_set_clkdiv(&c, divider);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void interconnect_rx_program_init(PIO pio, uint sm, uint offset, uint pin_rx_start, uint pin_ctrl, uint divider) {
    
    printf("rx_pin: %u, ctrl: %u\n", pin_rx_start, pin_ctrl);

    for (int i = pin_rx_start; i < pin_rx_start+NUM_INTERCONNECT_DAT_RX_PINS; i++) {
        pio_gpio_init(pio, i);
    }

    gpio_init(24);
    gpio_set_dir(24, false); // input

    // 22 = output
    // 24, 26, 27, 28, 29 = input

    // Pin 22 is OUT all others are IN
    pio_sm_set_pins_with_mask(pio, sm, 0x00000000, 0x3D400000);
    pio_sm_set_pindirs_with_mask(pio, sm, 0x00400000, 0x3D400000);

    pio_sm_set_consecutive_pindirs(pio, sm, pin_rx_start, 4, false);

    pio_sm_config c = inter_mcu_rx_program_get_default_config(offset);

    sm_config_set_in_shift(&c, true, true, 16);
    
    // sm_config_set_in_pins(&c, 24);
    sm_config_set_in_pins(&c, pin_rx_start);
    
    // ctrl pin is used to signal we have sampled the data
    // Set side set pin
    sm_config_set_sideset_pins(&c, pin_ctrl);

    // sm_config_set_clkdiv(&c, divider);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}