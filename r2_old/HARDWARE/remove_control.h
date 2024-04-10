#ifndef __REMOVE_CONTROL_H
#define __REMOVE_CONTROL_H

#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "String.h"

#define Hour        3
#define Minute      2
#define Second      1
#define MicroSecond 0

void TIM2_GET_TIM_Init(void);
void PPM_Init(void);
#endif 


