/*
 * keypad_lib.h
 *
 *  Created on: Nov 19, 2023
 *      Author: agusg
 */

#ifndef INC_KEYPAD_LIB_H_
#define INC_KEYPAD_LIB_H_


#define HAL_GPIO_MODULE_ENABLED

#include "stm32f1xx_hal.h"

#define KEYPAD_UNITS  1
#define ROWS          4
#define COLS          4
#define KEYS          16

#define KEY_PRESSED   1
#define KEY_RELEASED  0

#define KEY_1       0
#define KEY_2       1
#define KEY_3       2
#define KEY_F1      3
#define KEY_4       4
#define KEY_5       5
#define KEY_6       6
#define KEY_F2      7
#define KEY_7       8
#define KEY_8       9
#define KEY_9       10
#define KEY_F3      11
#define KEY_A       12
#define KEY_0       13
#define KEY_H       14
#define KEY_F4      15

typedef struct
{
	GPIO_TypeDef * ROW_GPIO[ROWS];
	uint16_t       ROW_PIN[ROWS];
	GPIO_TypeDef * COL_GPIO[COLS];
	uint16_t       COL_PIN[COLS];
}KEYPAD_CfgType;



void KEYPAD_Init(uint16_t au16_Instance, uint8_t* au8_KeyStates);
void KEYPAD_Scan(uint16_t au16_Instance);




#endif /* INC_KEYPAD_LIB_H_ */
