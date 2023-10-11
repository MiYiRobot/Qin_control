#include "main.h"

/************************************
   �˴�������ʵ��STM32��ros������λ��ͨѶ
************************************/

RELATIVE_POSITION BALL_TO_CAR;   //�򵽳������λ��
unsigned char receiveFlag;       //�������ݵı�־λ

//ͨ��Э�鳣��
const uint8_t header[2] = {0x55,0xaa};
const uint8_t ender[2] = {0x0d,0x0a};

//-----------------------�����巢�ͺͽ�������----------------------------------
//�������� ��ת�ӽǶȣ�ת�٣�������
union SendDataRPM
{ 
    short d; 
    unsigned char data[2];
}RPM1,RPM2;

union SendDataAngle
{
   float d;
   unsigned char data[4];
}Angle;
//�������� ��Ŀ����룬Ŀ��Ƕȣ�
union ReceiveData1
{
   float d;
   unsigned char data[4];
}Distance;

union ReceiveData2
{
   float d;
   unsigned char data[4];
}Target_Angle;
//�������ݻ�����
unsigned char  receiveBuff[15] = {0};
unsigned char USART_Receiver   = 0;          //��������
/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/
int usartReceiveData(float *data1,float *data2,unsigned char *flag)
{
        static unsigned char checkSum          = 0;            //����У��
        static unsigned char USARTBufferIndex  = 0;    //���ڻ�������ʼ��
        static short j=0,k=0;      //�������ݱ�־λ
        static unsigned char USARTReceiveFront = 0;     //���ڽ�������ͷ
        static unsigned char Start_Flag        = START;  //һ֡���ݴ��Ϳ�ʼ��־λ
        static short datalength                = 0;     //���ݳ���
    
        USART_Receiver = USART_ReceiveData(USART1);  //��������
      //��������ͷ
       if(Start_Flag == START)
       {
            if(USART_Receiver == 0xaa)       //buf[1]
            {
               if(USARTReceiveFront == 0x55) //buf[0]
               {                               //����ͷ��λ
                 Start_Flag = !START;     //�յ�����ͷ����ʼ��������
                 receiveBuff[0] = USARTReceiveFront;   //buf[0]
                 receiveBuff[1] = USART_Receiver;      //buf[1]
				 USARTBufferIndex = 0;             //��������ʼ��
				 checkSum = 0x00;				  //У��ͳ�ʼ��                   
               }
            }
            else
            {
                 USARTReceiveFront = USART_Receiver;
            }             
         }
         else
         {
            switch(USARTBufferIndex)
            {
                case 0: //�������ݳ���
                    receiveBuff[2] = USART_Receiver;   //buf[2]    length = 9 = 4 + 4 + 1
                    datalength =receiveBuff[2];  
                    USARTBufferIndex++;
                    break;
                case 1:    //�����������ݣ�����ֵ����
                      receiveBuff[j+3] = USART_Receiver;     //buf[3].....buf[11]
                      j++;
                      if(j>= datalength)    
                      {
                         j = 0;
                         USARTBufferIndex++;    //���յ���������
                      }
                      break;
                case 2:  //����У��λ
                      receiveBuff[3+datalength-1]=USART_Receiver;       //buf[12]
                      checkSum = getCrc8(receiveBuff,3+datalength);
                      //�����ϢУ��λ
                      if(checkSum != receiveBuff[3+datalength-1])
                      {
                        return 0;
                      }
                      USARTBufferIndex++;
                      break;
                case 3:   //������Ϣβ
				   if(k==0)
				   {
				    	//����0d     buf[11]  �����ж�
				    	k++;
				   }
				   else if (k==1)
				   {
					//����0a     buf[12] �����ж�   
                    
                    //���о���Ƕȸ�ֵ   
                     for(k = 0;k<4;k++)
                     { 
                        Distance.data[k] = receiveBuff[3+k];      //buf[3] buf[4] buf[5] buf[6] --->Distance.data[0]...[3]
                        Target_Angle.data[k] = receiveBuff[7+k];  //buf[7]..buf[10]  ---> Target_Angle.data[0]...[3]
                     }
  
                    //��ֵ����
                     *data1 = Distance.d;   //����
                     *data2 = Target_Angle.d;  //�Ƕ�
                     //��־λֱ�Ӹ�ֵ
                     *flag=receiveBuff[11];                       
                        
                     					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiveFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					datalength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------	
                   }
                   break;
                default:break;
              }
            }
    return 0;
}



/*---------------------����Э��----------------------------------------
 ------------55 aa size 00 00 00 00 crc8 0d 0a-------------------------   
//����ͷ55aa + �����ֽ���size + ����(���ù�����) + У׼crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ������Ŀ���������չ������size������
----------------------------------------------------------------------*/
/**
  * @brief  ��ת�ӽǶȡ�������ת�ٺͿ����źŽ��д����ͨ�����ڷ��͸�ros
  * @param  ת�ӽǶȡ�������ת�ٺͿ����ź�
  * @retval ��
  */
void usartSendData(short LeftRPM,short RightRPM,float angle,unsigned char ctrlFlag)
{
    //Э�����ݻ�������   
    unsigned char buf[15];
    int i,length=0;    
    //����Ƕȡ�ת�١�����
    Angle.d = angle;
    RPM1.d = LeftRPM;
    RPM2.d = RightRPM;
    //������Ϣͷ
    for(i = 0;i<2;i++)
    {
      buf[i] = header[i];        //buf[0] buf[1]
    }
    //���ó���
    length = 9;   //2 + 2 + 4 + 1 = 9
    buf[2] = length;            //buf[2]
    //��������ת��
    for(i = 0;i < 2; i++)
    {
      buf[3+i] = RPM1.data[i];      //buf[3]  buf[4]
      buf[5+i] = RPM2.data[i];      //buf[5]  buf[6]
    }
    //�������ӽǶ�
    for(i = 0;i<4; i++)
    {
      buf[7+i] = Angle.data[i];    //buf[7] buf[8] buf[9] buf[10]
    }
    //Ԥ������ָ��
    buf[3 + length - 1] = ctrlFlag;    //buf[11]
    //crcУ��
    buf[3 + length] = getCrc8(buf,3+ length);   //buf[12]
    //��Ϣβ 
    for(i = 0;i < 2;i++)
    {
       buf[length+3+1+i] = ender[i];    //buf[13]   buf[14]
    }
    //��������
    Usart_Send_String(buf,sizeof(buf));
}

/**
  * @brief  ����ָ����С���ַ�����
  * @param  �����ַ�������С
  * @retval ��
  */
void Usart_Send_String(unsigned char *p,short sendSize)
{
    static int length = 0;
    while(length<sendSize)
    {
       while(!(USART1->SR&(0x01<<7)) ){};  //���ͻ�����Ϊ��
        USART1->DR = *p;
       p++;
       length++;
    }
        length= 0 ;
}

/**************************************************************************
�������ܣ������λѭ������У�飬��usartSendData��usartReceiveOneData��������
��ڲ����������ַ�������С
����  ֵ����
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}

/**
  * @brief  �������ݳ�ʼ��
  * @param  ��
  * @retval ��
  */
void receiveinit(void)
{
    BALL_TO_CAR.distance=0;
    BALL_TO_CAR.angle=0;
    receiveFlag=0;
}

/**********************************END***************************************/

//�����жϷ����������ڽ���32����
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
        usartReceiveData(&BALL_TO_CAR.distance,&BALL_TO_CAR.angle,&receiveFlag); 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);			 //����жϱ�־λ
	 }
}


