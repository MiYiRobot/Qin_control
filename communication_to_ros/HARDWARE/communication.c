#include "main.h"

/************************************
   此代码用来实现STM32与ros的上下位机通讯
************************************/

RELATIVE_POSITION BALL_TO_CAR;   //球到车的相对位置
unsigned char receiveFlag;       //接收数据的标志位

//通信协议常量
const uint8_t header[2] = {0x55,0xaa};
const uint8_t ender[2] = {0x0d,0x0a};

//-----------------------共用体发送和接收数据----------------------------------
//发送数据 （转子角度，转速，电流）
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
//接收数据 （目标距离，目标角度）
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
//接收数据缓存区
unsigned char  receiveBuff[15] = {0};
unsigned char USART_Receiver   = 0;          //接收数据
/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/
int usartReceiveData(float *data1,float *data2,unsigned char *flag)
{
        static unsigned char checkSum          = 0;            //用于校验
        static unsigned char USARTBufferIndex  = 0;    //用于缓冲区初始化
        static short j=0,k=0;      //接收数据标志位
        static unsigned char USARTReceiveFront = 0;     //用于接收数据头
        static unsigned char Start_Flag        = START;  //一帧数据传送开始标志位
        static short datalength                = 0;     //数据长度
    
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
                    receiveBuff[2] = USART_Receiver;   //buf[2]    length = 9 = 4 + 4 + 1
                    datalength =receiveBuff[2];  
                    USARTBufferIndex++;
                    break;
                case 1:    //接收所有数据，并赋值处理
                      receiveBuff[j+3] = USART_Receiver;     //buf[3].....buf[11]
                      j++;
                      if(j>= datalength)    
                      {
                         j = 0;
                         USARTBufferIndex++;    //接收到所有数据
                      }
                      break;
                case 2:  //接收校验位
                      receiveBuff[3+datalength-1]=USART_Receiver;       //buf[12]
                      checkSum = getCrc8(receiveBuff,3+datalength);
                      //检查信息校验位
                      if(checkSum != receiveBuff[3+datalength-1])
                      {
                        return 0;
                      }
                      USARTBufferIndex++;
                      break;
                case 3:   //接收信息尾
				   if(k==0)
				   {
				    	//数据0d     buf[13]  无需判断
				    	k++;
				   }
				   else if (k==1)
				   {
					//数据0a     buf[14] 无需判断   
                    
                    //进行距离角度赋值   
                     for(k = 0;k<4;k++)
                     { 
                        Distance.data[k] = receiveBuff[3+k];      //buf[3] buf[4] buf[5] buf[6] --->Distance.data[0]...[3]
                        Target_Angle.data[k] = receiveBuff[7+k];  //buf[7]..buf[10]  ---> Target_Angle.data[0]...[3]
                     }
  
                    //赋值操作
                     *data1 = Distance.d;   //距离
                     *data2 = Target_Angle.d;  //角度
                     //标志位直接赋值
                     *flag=receiveBuff[11];                       
                        
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
//注意：这里数据中预留了一个字节的控制位，其余的可以自行扩展，更改size和数据
----------------------------------------------------------------------*/
/**
  * @brief  将转子角度、左右轮转速和控制信号进行打包，通过串口发送给ros
  * @param  转子角度、左右轮转速和控制信号
  * @retval 无
  */
void usartSendData(short LeftRPM,short RightRPM,float angle,unsigned char ctrlFlag)
{
    //协议数据缓存数组   
    unsigned char buf[15];
    int i,length=0;    
    //计算角度、转速、电流
    Angle.d = angle;
    RPM1.d = LeftRPM;
    RPM2.d = RightRPM;
    //设置消息头
    for(i = 0;i<2;i++)
    {
      buf[i] = header[i];        //buf[0] buf[1]
    }
    //设置长度
    length = 9;   //2 + 2 + 4 + 1 = 9
    buf[2] = length;            //buf[2]
    //发送轮子转速
    for(i = 0;i < 2; i++)
    {
      buf[3+i] = RPM1.data[i];      //buf[3]  buf[4]
      buf[5+i] = RPM2.data[i];      //buf[5]  buf[6]
    }
    //发送轮子角度
    for(i = 0;i<4; i++)
    {
      buf[7+i] = Angle.data[i];    //buf[7] buf[8] buf[9] buf[10]
    }
    //预留控制指令
    buf[3 + length - 1] = ctrlFlag;    //buf[11]
    //crc校验
    buf[3 + length] = getCrc8(buf,3+ length);   //buf[12]
    //消息尾 
    for(i = 0;i < 2;i++)
    {
       buf[length+3+1+i] = ender[i];    //buf[13]   buf[14]
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

/**
  * @brief  接收数据初始化
  * @param  无
  * @retval 无
  */
void receiveinit(void)
{
    BALL_TO_CAR.distance=0;
    BALL_TO_CAR.angle=0;
    receiveFlag=0;
}

/**********************************END***************************************/

//串口中断服务函数，用于接收32数据
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
        usartReceiveData(&BALL_TO_CAR.distance,&BALL_TO_CAR.angle,&receiveFlag); 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);			 //清除中断标志位
	 }
}


