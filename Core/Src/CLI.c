/*
 * CLI.c
 *
 *  Created on: May 15, 2024
 *      Author: konra
 */

#include "CLI.h"


//static char command[16];

//uint8_t parse_string(uint8_t* data){
//	uint8_t temp_data;
//	uint8_t i=0;
//	for(uint8_t n=0; n<32; n++){
//		command[n]=*(data+n);
//	}
//	return i;
//}

void state_machine(Command_struct* cmd_array, uint8_t* data, uint8_t length){
	//uint8_t length = parse_string(data);
	for(uint8_t i =0; i < 32;i++){
		if((strncmp((cmd_array+i)->command,(char*)data, length)==0) && (strlen((cmd_array+i)->command)== strlen((char*)data))){
			((cmd_array+i)->function_handler)((cmd_array+i)->data);
		}
	}
}
