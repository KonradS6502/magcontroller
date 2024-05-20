/*
 * softstarter.h
 *
 *  Created on: Apr 26, 2024
 *      Author: konra
 */

#ifndef INC_SOFTSTARTER_H_
#define INC_SOFTSTARTER_H_

#include "main.h"

typedef enum
{
  OUTPUT_RESET = 0U,
  OUTPUT_SET
}MAG_OutputState;

struct magnetron_struct{
	uint8_t Fault_state;
	uint8_t Ready_state;
	uint8_t Output_state;
	uint8_t Reset_state;
	GPIO_TypeDef* GPIO_ready_port;
	uint16_t GPIO_ready;
	GPIO_TypeDef* GPIO_fault_port;
	uint16_t GPIO_fault;
	GPIO_TypeDef* GPIO_reset_port;
	uint16_t GPIO_reset;
	GPIO_TypeDef* GPIO_output_port;
	uint16_t GPIO_output;
};

GPIO_PinState Mag1_init(struct magnetron_struct* mag);
GPIO_PinState Mag2_init(struct magnetron_struct* mag);
GPIO_PinState Mag3_init(struct magnetron_struct* mag);

GPIO_PinState ReadFault(struct magnetron_struct* mag);
GPIO_PinState ReadReady(struct magnetron_struct* mag);
GPIO_PinState ReadOutput(struct magnetron_struct* mag);
GPIO_PinState ReadReset(struct magnetron_struct* mag);
GPIO_PinState Set_output_mag(struct magnetron_struct* mag, uint8_t state);
GPIO_PinState Reset_sofstarter(struct magnetron_struct* mag);
void Mag_struct_update(struct magnetron_struct* mag);
#endif /* INC_SOFTSTARTER_H_ */
