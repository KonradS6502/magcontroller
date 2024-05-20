/*
 * CLI_f.h
 *
 *  Created on: May 15, 2024
 *      Author: konra
 */

#ifndef INC_CLI_F_H_
#define INC_CLI_F_H_

#include "main.h"
#include "softstarter.h"


uint8_t test_CLI(uint8_t data);
uint8_t get_status(uint8_t data);
uint8_t get_all_status(uint8_t data);
uint8_t mag_turn_on(uint8_t data);
uint8_t mag_turn_off(uint8_t data);
uint8_t mag_all_reset(uint8_t data);
uint8_t mag_reset(uint8_t data);
uint8_t terminal_switch(uint8_t data);
#endif /* INC_CLI_F_H_ */
