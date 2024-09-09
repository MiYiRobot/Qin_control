#ifndef __SERVO_H
#define __SERVO_H

void Servo_SetAngle(float Angle);
void Servo_Control(void);
void PWM_SERVO(void);
void servo_init(void);
void USART3_Send_String(unsigned char *p_array, short sendSize, USART_TypeDef *usart);
#endif
