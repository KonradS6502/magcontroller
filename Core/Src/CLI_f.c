/*
 * CLI_f.c
 *
 *  Created on: May 15, 2024
 *      Author: konra
 */

#include "CLI_f.h"

extern UART_HandleTypeDef huart3;
extern struct magnetron_struct MAG[3];

extern uint8_t terminal_mode;


uint8_t test_CLI(uint8_t data){
	char out[32];
	uint8_t length = 0;
	length += sprintf(&out[0],"{ \"status\": \"working\" }\r\n");

	HAL_UART_Transmit(&huart3,(uint8_t*)&out[0], length,100);
	return(data);
}


uint8_t get_status(uint8_t data){
	char out[128];
	uint8_t length =0;
	length += sprintf(&out[0],"{\"mag_index\": %i, \"fault_state\": %i, \"ready_state\": %i, \"output_state\": %i } \r\n", data, ((~MAG[data].Fault_state)&0b1), MAG[data].Ready_state, MAG[data].Output_state);
	HAL_UART_Transmit(&huart3,(uint8_t*)&out[0], length,100);
	return(data);
}


uint8_t get_all_status(uint8_t data){
	char out[256];
	uint16_t length =0;
	length += sprintf(&out[0]+length,"{\r\n");
	length += sprintf(&out[0]+length,"\"%i\":{\"mag_index\": %i, \"fault_state\": %i, \"ready_state\": %i, \"output_state\": %i},", 0,0, ((~MAG[0].Fault_state)&0b1), MAG[0].Ready_state, MAG[0].Output_state);
	length += sprintf(&out[0]+length,"\"%i\":{\"mag_index\": %i, \"fault_state\": %i, \"ready_state\": %i, \"output_state\": %i},", 1,1, ((~MAG[1].Fault_state)&0b1), MAG[1].Ready_state, MAG[1].Output_state);
	length += sprintf(&out[0]+length,"\"%i\":{\"mag_index\": %i, \"fault_state\": %i, \"ready_state\": %i, \"output_state\": %i}", 2,2, ((~MAG[2].Fault_state)&0b1), MAG[2].Ready_state, MAG[2].Output_state);
	length += sprintf(&out[0]+length,"}\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)&out[0], length,100);
	return(data);
}

uint8_t mag_turn_on(uint8_t data){
	 Set_output_mag(&MAG[data], 1);
	 char out[64];
	 uint8_t length=0;
	 length += sprintf(&out[0],"{\"mag_index\": %i,\"status\":1}\r\n",data);
	 HAL_UART_Transmit(&huart3,(uint8_t*)&out[0], length,100);
	 return(data);
}


uint8_t mag_turn_off(uint8_t data){
	 Set_output_mag(&MAG[data], 0);
	 char out[64];
	 uint8_t length=0;
	 length += sprintf(&out[0],"{\"mag_index\": %i,\"status\":0}\r\n",data);
	 HAL_UART_Transmit(&huart3,(uint8_t*)&out[0], length,100);
	 return(data);
}

uint8_t mag_reset(uint8_t data){
	Reset_sofstarter(&MAG[data]);
	Set_output_mag(&MAG[data], 0);
	char out[64];
	uint8_t length=0;
	length += sprintf(&out[0],"{\"mag_index\": %i,\"status\":0}\r\n",data);
	HAL_UART_Transmit(&huart3,(uint8_t*)&out[0], length,100);
	return(data);
}


uint8_t mag_all_reset(uint8_t data){
	char out[256];
	uint8_t length=0;
	length += sprintf(&out[0]+length,"{");
	for(uint8_t i=0; i<data; i++){
		Reset_sofstarter(&MAG[i]);
		Set_output_mag(&MAG[i], 0);
		length += sprintf(&out[0],"\"%i\":{\"mag_index\": %i,\"status\":0}",i,i);
	}
	length += sprintf(&out[0]+length,"}\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)&out[0], length,100);
	return(data);
}

uint8_t terminal_switch(uint8_t data){
	if(data==1){
		terminal_mode=1;
	}
	else{
		terminal_mode=0;
	}
	return(data);
}
