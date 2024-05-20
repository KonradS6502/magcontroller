/*
 * util.c
 *
 *  Created on: May 3, 2024
 *      Author: konra
 */

#include "util.h"

void uint16_to_uint8(uint16_t* data_in, uint8_t* data_out){
	  *data_out = (*data_in >> 8) & 255;
	  *(data_out+1) = *data_in & 255;
}
