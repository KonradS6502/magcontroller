/*
 * sofstarter.c
 *
 *  Created on: Apr 26, 2024
 *      Author: konra
 */

#include "softstarter.h"

GPIO_PinState Mag1_init(struct magnetron_struct* mag){
	mag->GPIO_fault=FAULT1_Pin;
	mag->GPIO_fault_port=FAULT1_GPIO_Port;
	mag->GPIO_ready=READY1_Pin;
	mag->GPIO_ready_port=READY1_GPIO_Port;
	mag->GPIO_reset=RESET1_Pin;
	mag->GPIO_reset_port=RESET1_GPIO_Port;
	mag->GPIO_output=OUT1_Pin;
	mag->GPIO_output_port=OUT1_GPIO_Port;
	return(ReadFault(mag));
}

GPIO_PinState Mag2_init(struct magnetron_struct* mag){
	mag->GPIO_fault=FAULT2_Pin;
	mag->GPIO_fault_port=FAULT2_GPIO_Port;
	mag->GPIO_ready=READY2_Pin;
	mag->GPIO_ready_port=READY2_GPIO_Port;
	mag->GPIO_reset=RESET2_Pin;
	mag->GPIO_reset_port=RESET2_GPIO_Port;
	mag->GPIO_output=OUT2_Pin;
	mag->GPIO_output_port=OUT2_GPIO_Port;
	return(ReadFault(mag));
}

GPIO_PinState Mag3_init(struct magnetron_struct* mag){
	mag->GPIO_fault=FAULT3_Pin;
	mag->GPIO_fault_port=FAULT3_GPIO_Port;
	mag->GPIO_ready=READY3_Pin;
	mag->GPIO_ready_port=READY3_GPIO_Port;
	mag->GPIO_reset=RESET3_Pin;
	mag->GPIO_reset_port=RESET3_GPIO_Port;
	mag->GPIO_output=OUT3_Pin;
	mag->GPIO_output_port=OUT3_GPIO_Port;
	return(ReadFault(mag));
}

GPIO_PinState ReadFault(struct magnetron_struct* mag){
	return(HAL_GPIO_ReadPin(mag->GPIO_fault_port, mag->GPIO_fault));
}

GPIO_PinState ReadReady(struct magnetron_struct* mag){
	return(HAL_GPIO_ReadPin(mag->GPIO_ready_port, mag->GPIO_ready));
}

GPIO_PinState ReadOutput(struct magnetron_struct* mag){
	return(HAL_GPIO_ReadPin(mag->GPIO_output_port, mag->GPIO_output));
}

GPIO_PinState ReadReset(struct magnetron_struct* mag){
	return(HAL_GPIO_ReadPin(mag->GPIO_reset_port, mag->GPIO_reset));
}

GPIO_PinState Set_output_mag(struct magnetron_struct* mag, uint8_t state){
	HAL_GPIO_WritePin(mag->GPIO_output_port, mag->GPIO_output, state);
	return(ReadFault(mag));
}

GPIO_PinState Reset_sofstarter(struct magnetron_struct* mag){
	HAL_GPIO_WritePin(mag->GPIO_reset_port, mag->GPIO_reset, GPIO_PIN_SET);
	for(uint32_t i=0; i<1200000;i++){
		asm("NOP");
	}
	HAL_GPIO_WritePin(mag->GPIO_reset_port, mag->GPIO_reset, GPIO_PIN_RESET);
	for(uint32_t i=0; i<1200000;i++){
		asm("NOP");
	}
	HAL_GPIO_WritePin(mag->GPIO_reset_port, mag->GPIO_reset, GPIO_PIN_SET);
	return(ReadFault(mag));
}

void Mag_struct_update(struct magnetron_struct* mag){
	mag->Fault_state = ReadFault(mag);
	mag->Ready_state = ReadReady(mag);
	mag->Output_state = ReadOutput(mag);
	mag->Reset_state = ReadReset(mag);
}
