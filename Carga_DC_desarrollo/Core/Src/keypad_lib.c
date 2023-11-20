#include "keypad_lib.h"





void KEYPAD_Scan(){
/* Make ROW 1 LOW and all other ROWs HIGH */
HAL_GPIO_WritePin (R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
HAL_GPIO_WritePin (R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);  // Pull the R2 High
HAL_GPIO_WritePin (R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);  // Pull the R3 High
HAL_GPIO_WritePin (R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);  // Pull the R4 High

if (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)))   // if the Col 1 is low
{
	while (!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));   // wait till the button is pressed
	return '1';
}

if (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)))   // if the Col 2 is low
{
	while (!(HAL_GPIO_ReadPin (C2_GPIO_Port, C2_Pin)));   // wait till the button is pressed
	return '2';
}

if (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)))   // if the Col 3 is low
{
	while (!(HAL_GPIO_ReadPin (C3_GPIO_Port, C3_Pin)));   // wait till the button is pressed
	return '3';
}

if (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)))   // if the Col 4 is low
{
	while (!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin)));   // wait till the button is pressed
	return 'A';
}

}
