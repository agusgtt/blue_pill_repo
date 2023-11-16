
/**************************************************************
* Libreria para el manejo de pantalla LCD alfanumerica por I2C
* 4 BITS DE DATOS Y 2 PINES DE CONTROL
* Compatible con PIC, AVR y ARM
* Todos los pines son remapeables
* Fecha de creaciin 23/04/20
*************************************************************/

#ifndef __LCD_I2C_H
#define __LCD_I2C_H

#include "stm32f1xx_hal.h"

/********** Definiciones de Comandos de LCD *********/

#define     LCD_CLEAR	    0x01    //Limpia pantalla
#define     LCD_HOME	    0x02    //Retorno al inicio
#define     LCD_CURSOR_ON	0x0F    //Cursor on
#define     LCD_CURSOR_OFF	0x0C    //Cursor off
#define     LCD_LINEA1	    0x00	//Promera Fila
#define     LCD_LINEA2		0XC0	//Segunda Fila
#define     LCD_LINEA3		0x94	//Tercera Fila
#define     LCD_LINEA4		0xD4	//Cuarta Fila
#define     LCD_LEFT		0x10	//Cursor a la izquierda
#define     LCD_RIGHT		0x14	//Cursor a la derecha
#define     LCD_ROT_LEFT	0x18	//Rotar a la izquierda
#define     LCD_ROT_RIGHT	0x1C	//Rotar a la derecha
#define     LCD_OFF 		0x08	//apaga el display

/* Definiciones de Comandos de LCD ------------------------*/

void LCD_I2C_init(void);				// Inicializa LCD
void LCD_I2C_cmd(char cmd);  			// Envia comando LCD
void LCD_I2C_char(char data);  			// Envia dato LCD
void LCD_I2C_write_text(char *dato);  	// Envia cadena de caracteres


#endif /* __LCD_I2C_H */
