
/********************************************************************
* Libreria para el manejo de pantalla LCD alfanumerica por I2C
* 4 BITS DE DATOS Y 2 PINES DE CONTROL
* Compatible con PIC, AVR y ARM
* Todos los pines son remapeables
* Fecha de creaciin 23/04/20
********************************************************************/

#include "LCD_I2C.h"

extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly
#define SLAVE_ADDRESS_LCD  0x4e	//0x4E // change this according to ur setup 0x7e

/* Envia comando por LCD ---------------------------------------*/

void LCD_I2C_cmd(char cmd)
{
  	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	HAL_Delay(1);
}

/* Envia dato por LCD ------------------------------------------*/

void LCD_I2C_char(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	HAL_Delay(1);
}

/* Inicializa LCD -----------------------------------------------*/

void LCD_I2C_init(void)
{
	LCD_I2C_cmd(0x02);
	LCD_I2C_cmd(0x28);
	LCD_I2C_cmd(0x0c);
	LCD_I2C_cmd(0x80);
	LCD_I2C_cmd(LCD_CLEAR);
}

/* Envia cadena de caracteres al LCD -----------------------------*/

void LCD_I2C_write_text(char *str)
{
	while (*str) LCD_I2C_char(*str++);
}
