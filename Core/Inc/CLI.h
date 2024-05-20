/*
 * CLI.h
 *
 *  Created on: May 15, 2024
 *      Author: konra
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include "main.h"
#include <stdio.h>
#include <string.h>
//#include "circlebuffer.h"



typedef struct
{
	char command[32];
	uint8_t data;
	uint8_t (*function_handler)(uint8_t);

} Command_struct;

void state_machine(Command_struct* cmd_array, uint8_t* data, uint8_t length);
uint8_t parse_string(uint8_t* data);

#endif /* INC_CLI_H_ */
