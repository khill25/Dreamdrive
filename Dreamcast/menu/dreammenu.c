/**
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Kaili Hill
 * 
 * dreammenu.c
 * 
 * This is the main entry file that setups up the Dreamcast video output, inputs, and starts the main menu loop.
 */

#include <kos.h>
#include <dc/biosfont.h>

int main(int argc, char **argv) {
    pvr_init_defaults();
    vid_set_mode(DM_640x480, PM_RGB565);
    vid_border_color(0, 0, 0);

    int o = 20 * 640 + 20;

    /* Test with ISO8859-1 encoding */
    bfont_set_encoding(BFONT_CODE_ISO8859_1);
    bfont_draw_str(vram_s + o, 640, 1, "Test of basic ASCII");

    while(1) {
        usleep(1 * 1000 * 1000);
    }

    return 0;
}