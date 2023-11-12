#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "action.h"

#define START 0X11     //一帧数据传送开始标志位

int usartReceiveData(void);
void usartSendData(short LeftRPM,short RightRPM,float angle,unsigned char ctrlFlag);
void Send_Action_Data(ACTION_GL_POS ACTION_DATA);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);
void Usart_Send_String(unsigned char *p,short sendSize);
#endif

