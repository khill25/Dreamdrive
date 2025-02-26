/*
 * gdrom_utils.c
 * 
 * Utility functions to convert between CD read info and the iso on the sd card
 * IE Using the params from a SPI command, get the data from the iso image
 */

 /*
gdi file contents

-- Looks like
num tracks
track x, x, x, x, filename, x
....

3
1 0 4 2352 track01.bin 0
2 600 0 2352 track02.raw 0
3 45000 4 2352 track03.bin 0

*/
