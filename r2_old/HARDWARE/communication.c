#include "main.h"

/************************************
   此代码用来实现STM32与ros的上下位机通讯
************************************/

float Cmd_x,Cmd_y,Cmd_w;      //ros传输过来的速度信息
uint8_t cmd_flag;             //ros控制标志（先预留）

//通信协议常量
const uint8_t header[2] = {0x55,0xaa};
const uint8_t ender[2] = {0x0d,0x0a};

//接收数据缓存区
unsigned char  receiveBuff[19] = {0};
unsigned char USART_Receiver   = 0;          //接收数据
/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
   接收来自ros的速度控制指令，预留了一个控制位
--------------------------------------------------------------------------*/
int usartReceiveData(void)
{
        static unsigned char checkSum          = 0;            //用于校验
        static unsigned char USARTBufferIndex  = 0;    //用于缓冲区初始化
        static short j=0,k=0;      //接收数据标志位
        static unsigned char USARTReceiveFront = 0;     //用于接收数据头
        static unsigned char Start_Flag        = START;  //一帧数据传送开始标志位
        static short datalength                = 0;     //数据长度
    
        static union 
        {
           uint8_t data[12]; 
           float d[3];      //接收cmd_vel数据，只接收有用的
        }cmd_vel;
    
        USART_Receiver = USART_ReceiveData(USART1);  //接收数据
      //接收数据头
       if(Start_Flag == START)
       {
            if(USART_Receiver == 0xaa)       //buf[1]
            {
               if(USARTReceiveFront == 0x55) //buf[0]
               {                               //数据头两位
                 Start_Flag = !START;     //收到数据头，开始接收数据
                 receiveBuff[0] = USARTReceiveFront;   //buf[0]
                 receiveBuff[1] = USART_Receiver;      //buf[1]
				 USARTBufferIndex = 0;             //缓冲区初始化
				 checkSum = 0x00;				  //校验和初始化                   
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
                case 0: //接收数据长度
                    receiveBuff[2] = USART_Receiver;   //buf[2]    length = 13 = 4 + 4 + 4 + 1
                    datalength =receiveBuff[2];        //13
                    USARTBufferIndex++;
                    break;
                case 1:    //接收所有数据，并赋值处理
                      receiveBuff[j+3] = USART_Receiver;     //buf[3].....buf[15]
                      j++;
                      if(j>= datalength)    
                      {
                         j = 0;
                         USARTBufferIndex++;    //接收到所有数据
                      }
                      break;
                case 2:  //接收校验位
                      receiveBuff[3+datalength]=USART_Receiver;       //buf[16]
                      checkSum = getCrc8(receiveBuff,3+datalength);   
                      //检查信息校验位
                      if(checkSum != receiveBuff[3+datalength])
                      {
                        return 0;
                      }
                      USARTBufferIndex++;
                      break;
                case 3:   //接收信息尾
				   if(k==0)
				   {
				    	//数据0d     buf[17]  无需判断
				    	k++;
				   }
				   else if (k==1)
				   {
					//数据0a     buf[18] 无需判断   
                    
                    //进行距离角度赋值   
                     for(k = 0;k<4;k++)
                     { 
                        cmd_vel.data[k]   = receiveBuff[3+k];      //buf[3]...buf[6]   ---> cmd_vel.data[0]...[3]
                        cmd_vel.data[k+4] = receiveBuff[7+k];      //buf[7]...buf[10]  ---> cmd_vel.data[4]...[7]
                        cmd_vel.data[k+8] = receiveBuff[11+k];     //buf[11]..buf[14]  ---> cmd_vel.data[8]...[11]
                     }
  
                    //赋值操作
                     Cmd_x = cmd_vel.d[0];  //x方向速度
                     Cmd_y = cmd_vel.d[1];  //y方向速度
                     Cmd_w = cmd_vel.d[2];  //偏航角速度
                     //标志位直接赋值
                     cmd_flag=receiveBuff[15];                                               
                     					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
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


/*---------------------发送协议----------------------------------------
 ------------55 aa size 00 00 00 00 crc8 0d 0a-------------------------   
//数据头55aa + 数据字节数size + 数据(利用共用体) + 校准crc8 + 数据尾0d0a
----------------------------------------------------------------------*/
void Send_Action_Data(ACTION_GL_POS ACTION_DATA)
{
    unsigned char buf[30];
    int i,length=0; 
    static union
    {    //联合体：多个变量共用一个地址      
         uint8_t data[24];
         float ActVal[6];
    }posture;
    
    posture.ActVal[0] = ACTION_DATA.ANGLE_Z;        //偏航角
    posture.ActVal[1] = ACTION_DATA.ANGLE_Y;        //俯仰角
    posture.ActVal[2] = ACTION_DATA.ANGLE_X;        //翻滚角
    posture.ActVal[3] = ACTION_DATA.POS_X;          //X坐标
    posture.ActVal[4] = ACTION_DATA.POS_Y;          //Y坐标
    posture.ActVal[5] = ACTION_DATA.W_Z;            //偏航角速度

    //    //设置消息头
    for(i = 0;i<2;i++)
    {
      buf[i] = header[i];        //buf[0] buf[1]
    }
    //设置长度
    length = 24;   //6 * 4 =24
    buf[2] = length;            //buf[2]

    //发送action数据
    for(i = 0;i<4; i++)
    {
      buf[3+i]  = posture.data[i];     //buf[3]  buf[4]  buf[5]  buf[6]     data[0]....data[3]
      buf[7+i]  = posture.data[i+4];   //buf[7]  buf[8]  buf[9]  buf[10]    data[4]....data[7]
      buf[11+i] = posture.data[i+8];   //buf[11] buf[12] buf[13] buf[14]     data[8]....data[11]   
      buf[15+i] = posture.data[i+12];  //buf[15] buf[16] buf[17] buf[18]   data[12]...data[15]
      buf[19+i] = posture.data[i+16]; //buf[19] buf[20] buf[21] buf[22]    data[16]...data[19]
      buf[23+i] = posture.data[i+20]; //buf[23] buf[24] buf[25] buf[26]    data[20]...data[23]    
    }
    //crc校验
    buf[3 + length] = getCrc8(buf,3+ length);   //buf[27]
    //消息尾 
    for(i = 0;i < 2;i++)
    {
       buf[length+3+1+i] = ender[i];    //buf[28]   buf[29]
    }
    //发送数据
    Usart_Send_String(buf,sizeof(buf));
}

/**
  * @brief  发送指定大小的字符数组
  * @param  数组地址、数组大小
  * @retval 无
  */
void Usart_Send_String(unsigned char *p,short sendSize)
{
    static int length = 0;
    while(length<sendSize)
    {
       while(!(USART1->SR&(0x01<<7)) ){};  //发送缓存区为空
        USART1->DR = *p;
       p++;
       length++;
    }
        length= 0 ;
}

/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
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

//串口中断服务函数，用于接收32数据
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
        usartReceiveData(); 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);			 //清除中断标志位
	 }
}


