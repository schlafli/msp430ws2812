/*
 * graphics.c
 *
 *  Created on: 9 Apr 2017
 *      Author: schlafli
 */
#include "graphics.h"

const uint8_t sinLUT[] = { 15, 18, 21, 24, 26, 28, 29, 30, 31, 30, 29, 28, 26,
		24, 21, 18, 15, 12, 9, 6, 4, 2, 1, 0, 0, 0, 1, 2, 4, 6, 9, 12 };

uint8_t t_sin(uint8_t in) {
	return sinLUT[in & 0x1f];
}

uint8_t t_cos(uint8_t in) {
	return sinLUT[(in + 8) & 0x1f];
}
