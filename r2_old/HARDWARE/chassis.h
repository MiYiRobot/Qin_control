#ifndef  __CHASSIS_H
#define  __CHASSIS_H

#define PI 3.1415926f
#define x  0
#define y  1
#define w  2

void CAN_motor_init(void);//can和pid的初始化
void Moving(void *pvParameters);
void Set_World_Speed_Trans(void);
#endif
