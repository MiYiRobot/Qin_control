#ifndef _MY_UART_
#define _MY_UART_

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"

extern float pos_x;
extern float pos_y;
extern float zangle;

void uart4_init(u32 bound);

void UART_SendString(USART_TypeDef* USARTx, char *DataString);
#endif
