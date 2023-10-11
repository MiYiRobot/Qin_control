#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#define START 0X11     //一帧数据传送开始标志位
typedef struct RELATIVE_POSITION  //相对位置结构体，用来储存两个物体（球和车）的相对位置
{
   float distance;     //距离
   float angle;        //角度
}RELATIVE_POSITION;

extern RELATIVE_POSITION BALL_TO_CAR;   //球到车的相对位置
extern unsigned char receiveFlag;

int usartReceiveData(float *data1,float *data2,unsigned char *flag);
void usartSendData(short LeftRPM,short RightRPM,float angle,unsigned char ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);
void Usart_Send_String(unsigned char *p,short sendSize);
void receiveinit(void);
#endif

