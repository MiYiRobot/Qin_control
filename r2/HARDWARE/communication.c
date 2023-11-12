#include "main.h"

/************************************
   �˴�������ʵ��STM32��ros������λ��ͨѶ
************************************/

float Cmd_x,Cmd_y,Cmd_w;      //ros����������ٶ���Ϣ
uint8_t cmd_flag;             //ros���Ʊ�־����Ԥ����

//ͨ��Э�鳣��
const uint8_t header[2] = {0x55,0xaa};
const uint8_t ender[2] = {0x0d,0x0a};

//�������ݻ�����
unsigned char  receiveBuff[19] = {0};
unsigned char USART_Receiver   = 0;          //��������
/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
   ��������ros���ٶȿ���ָ�Ԥ����һ������λ
--------------------------------------------------------------------------*/
int usartReceiveData(void)
{
        static unsigned char checkSum          = 0;            //����У��
        static unsigned char USARTBufferIndex  = 0;    //���ڻ�������ʼ��
        static short j=0,k=0;      //�������ݱ�־λ
        static unsigned char USARTReceiveFront = 0;     //���ڽ�������ͷ
        static unsigned char Start_Flag        = START;  //һ֡���ݴ��Ϳ�ʼ��־λ
        static short datalength                = 0;     //���ݳ���
    
        static union 
        {
           uint8_t data[12]; 
           float d[3];      //����cmd_vel���ݣ�ֻ�������õ�
        }cmd_vel;
    
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
                    receiveBuff[2] = USART_Receiver;   //buf[2]    length = 13 = 4 + 4 + 4 + 1
                    datalength =receiveBuff[2];        //13
                    USARTBufferIndex++;
                    break;
                case 1:    //�����������ݣ�����ֵ����
                      receiveBuff[j+3] = USART_Receiver;     //buf[3].....buf[15]
                      j++;
                      if(j>= datalength)    
                      {
                         j = 0;
                         USARTBufferIndex++;    //���յ���������
                      }
                      break;
                case 2:  //����У��λ
                      receiveBuff[3+datalength]=USART_Receiver;       //buf[16]
                      checkSum = getCrc8(receiveBuff,3+datalength);   
                      //�����ϢУ��λ
                      if(checkSum != receiveBuff[3+datalength])
                      {
                        return 0;
                      }
                      USARTBufferIndex++;
                      break;
                case 3:   //������Ϣβ
				   if(k==0)
				   {
				    	//����0d     buf[17]  �����ж�
				    	k++;
				   }
				   else if (k==1)
				   {
					//����0a     buf[18] �����ж�   
                    
                    //���о���Ƕȸ�ֵ   
                     for(k = 0;k<4;k++)
                     { 
                        cmd_vel.data[k]   = receiveBuff[3+k];      //buf[3]...buf[6]   ---> cmd_vel.data[0]...[3]
                        cmd_vel.data[k+4] = receiveBuff[7+k];      //buf[7]...buf[10]  ---> cmd_vel.data[4]...[7]
                        cmd_vel.data[k+8] = receiveBuff[11+k];     //buf[11]..buf[14]  ---> cmd_vel.data[8]...[11]
                     }
  
                    //��ֵ����
                     Cmd_x = cmd_vel.d[0];  //x�����ٶ�
                     Cmd_y = cmd_vel.d[1];  //y�����ٶ�
                     Cmd_w = cmd_vel.d[2];  //ƫ�����ٶ�
                     //��־λֱ�Ӹ�ֵ
                     cmd_flag=receiveBuff[15];                                               
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
----------------------------------------------------------------------*/
void Send_Action_Data(ACTION_GL_POS ACTION_DATA)
{
    unsigned char buf[30];
    int i,length=0; 
    static union
    {    //�����壺�����������һ����ַ      
         uint8_t data[24];
         float ActVal[6];
    }posture;
    
    posture.ActVal[0] = ACTION_DATA.ANGLE_Z;        //ƫ����
    posture.ActVal[1] = ACTION_DATA.ANGLE_Y;        //������
    posture.ActVal[2] = ACTION_DATA.ANGLE_X;        //������
    posture.ActVal[3] = ACTION_DATA.POS_X;          //X����
    posture.ActVal[4] = ACTION_DATA.POS_Y;          //Y����
    posture.ActVal[5] = ACTION_DATA.W_Z;            //ƫ�����ٶ�

    //    //������Ϣͷ
    for(i = 0;i<2;i++)
    {
      buf[i] = header[i];        //buf[0] buf[1]
    }
    //���ó���
    length = 24;   //6 * 4 =24
    buf[2] = length;            //buf[2]

    //����action����
    for(i = 0;i<4; i++)
    {
      buf[3+i]  = posture.data[i];     //buf[3]  buf[4]  buf[5]  buf[6]     data[0]....data[3]
      buf[7+i]  = posture.data[i+4];   //buf[7]  buf[8]  buf[9]  buf[10]    data[4]....data[7]
      buf[11+i] = posture.data[i+8];   //buf[11] buf[12] buf[13] buf[14]     data[8]....data[11]   
      buf[15+i] = posture.data[i+12];  //buf[15] buf[16] buf[17] buf[18]   data[12]...data[15]
      buf[19+i] = posture.data[i+16]; //buf[19] buf[20] buf[21] buf[22]    data[16]...data[19]
      buf[23+i] = posture.data[i+20]; //buf[23] buf[24] buf[25] buf[26]    data[20]...data[23]    
    }
    //crcУ��
    buf[3 + length] = getCrc8(buf,3+ length);   //buf[27]
    //��Ϣβ 
    for(i = 0;i < 2;i++)
    {
       buf[length+3+1+i] = ender[i];    //buf[28]   buf[29]
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

/**********************************END***************************************/

//�����жϷ����������ڽ���32����
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
        usartReceiveData(); 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);			 //����жϱ�־λ
	 }
}


