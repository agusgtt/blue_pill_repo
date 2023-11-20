/*
 * keypad_lib.h
 *
 *  Created on: Nov 19, 2023
 *      Author: agusg
 */

#ifndef INC_KEYPAD_LIB_H_
#define INC_KEYPAD_LIB_H_


//#define HAL_GPIO_MODULE_ENABLED

#include "stm32f1xx_hal.h"


void keypad_init(void);
char keypad_scan(void);
uint8_t tipo_dat0(char input);



#endif /* INC_KEYPAD_LIB_H_ */
