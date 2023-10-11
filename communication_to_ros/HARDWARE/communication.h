#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#define START 0X11     //һ֡���ݴ��Ϳ�ʼ��־λ
typedef struct RELATIVE_POSITION  //���λ�ýṹ�壬���������������壨��ͳ��������λ��
{
   float distance;     //����
   float angle;        //�Ƕ�
}RELATIVE_POSITION;

extern RELATIVE_POSITION BALL_TO_CAR;   //�򵽳������λ��
extern unsigned char receiveFlag;

int usartReceiveData(float *data1,float *data2,unsigned char *flag);
void usartSendData(short LeftRPM,short RightRPM,float angle,unsigned char ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);
void Usart_Send_String(unsigned char *p,short sendSize);
void receiveinit(void);
#endif

