#ifndef  __CHASSIS_H
#define  __CHASSIS_H

#include <stm32f4xx.h>
#define PI 3.1415926f
#define x  0
#define y  1
#define w  2

void CAN_motor_init(void);//can��pid�ĳ�ʼ��
void Moving(void *pvParameters);
void Set_World_Speed_Trans(int World_Vx,int World_Vy,int World_Vw);
#endif
