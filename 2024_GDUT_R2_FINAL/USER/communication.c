#include "main.h"

/*********************************************************************************************************************************************************
2023/10/2 Author：Yang JianYi
STM32和ROS的串口通讯，使用的端口为USART1，该通讯协议编写了float类型，int16类型和char类型的数据，预留一个控制位来进行是否使用ROS自动控制的开关
debug用的串口暂时选定为uart5
**********************************************************************************************************************************************************/

 /*--------------------------------发送协议-----------------------------------
	STM32与ROS的通讯协议
	中断回调函数的启用需要有一次中断服务函数的触发，改触发放在了main.c中
--------------------------------------------------------------------------*/
 /*--------------------------------发送协议-----------------------------------
----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
	数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
	注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
	数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
	注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/
 
STIR_BUTTON Button = {0};

//通信协议常量
const unsigned char serial_header[2]  = {0x55, 0xaa};
const unsigned char serial_ender[2]   = {0x0d, 0x0a};

//接收电机转速命令共用体
union Recieve_Data_RPM
{
	short data;
	unsigned char tem_array[2];
}RosToStm32_RPM1,RosToStm32_RPM2;

//接收电机角度命令共用体
union Recieve_Data_CHASSIS
{
	float data;
	unsigned char tem_array[4];
}RosToStm32_CHASSIS_X,RosToStm32_CHASSIS_Y,RosToStm32_CHASSIS_W;

union Receive_Data_YAW
{
    uint8_t tem_array[4];
    float data;
    
}LADAR_YAW;
unsigned char testRece4 = 0x00;
uint8_t test = 0;
int test_uart= 0;
unsigned char USART_Receiver5 = 0; 
unsigned char USART_Receiver6 = 0;
/**
 * @brief 串口中断回调函数，数据解包以及赋值在里面进行操作
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6)
    {
//		R2_CONTROLLER.ros_stm32_noconnected_cnt = 0;
//		R2_CONTROLLER.ros_stm32_noconnected_flag = 0;
//        test_uart++;
//		//机构下个状态
	//	STM32_READ_FROM_ROS(&R2_CONTROLLER.NEXT_CONTROLLER_STATE,&R2_CONTROLLER.filter_ball_state);
         HAL_UART_Receive_IT(&huart6,&USART_Receiver6,1);
        if(USART_Receiver6 == 0)R2_CONTROLLER.ball_position = BALL_NO;
        if(USART_Receiver6 == 1)R2_CONTROLLER.ball_position = BALL_inner;
        if(USART_Receiver6 == 2)R2_CONTROLLER.ball_position = BALL_middle;
        if(USART_Receiver6 == 3)R2_CONTROLLER.ball_position = BALL_BOTH;
        
        
			
    }
	if(huart->Instance == UART5)
	{
        HAL_UART_Receive_IT(&huart5,&USART_Receiver5,1);	
        if(USART_Receiver5 == 0||USART_Receiver5 == 1||USART_Receiver5 == 4||USART_Receiver5 == 5){
			R2_CONTROLLER.claw_state = CLAW_CLOSE;
		}else{
			R2_CONTROLLER.claw_state = CLAW_OPEN;
		}

		if(USART_Receiver5 == 0||USART_Receiver5 == 2||USART_Receiver5 == 4||USART_Receiver5 == 6){
            R2_CONTROLLER.Gate_state = UP;
        }else{
            R2_CONTROLLER.Gate_state = NOT_UP;
        }
        
        if(USART_Receiver5 == 0||USART_Receiver5 == 1||USART_Receiver5 == 2||USART_Receiver5 == 3){
            R2_CONTROLLER.put_ball_arrival = 1;
        }else{
            R2_CONTROLLER.put_ball_arrival = 0;
        }
	}
    

	if(huart->Instance == UART4)
	{
        R2_CONTROLLER.ros_stm32_noconnected_cnt = 0;
		R2_CONTROLLER.ros_stm32_noconnected_flag = 0;
        test_uart++;
		//机构下个状态
		STM32_READ_FROM_ROS(&R2_CONTROLLER.NEXT_CONTROLLER_STATE,&R2_CONTROLLER.filter_ball_state,&R2_CONTROLLER.robot_yaw,&R2_CONTROLLER.clean_ball_flag);		
		//action_data_analyse();
	}
}


//数据接收暂存区
unsigned char receivesend_buff[13]={0};
//
//unsigned char USART_Receiver1              = 0;          //接收数据
unsigned char USART_Receiver4              = 0;          //接收数据

/**
 * @brief 接收ROS发来的数据并解包
 * @param 接收的数据地址
*/
int STM32_READ_FROM_ROS(unsigned char *next_controller_state,unsigned char *take_ball_flag,float *yaw,unsigned char *clean_ball_flag)
{
	test =2;
	static unsigned char checkSum             = 0;			//校验值
	static unsigned char USARTsend_bufferIndex     = 0;		//用于记录解包的操作步骤
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;			
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;			//数据包中数据的总长度

	//HAL_UART_Receive_DMA(&huart1,&USART_Receiver1,1);		//中断服务函数
	// HAL_UART_Receive_IT(&huart1,&USART_Receiver1,1);		//调试使用，勿管。开关在freertos.c
    HAL_UART_Receive_IT(&huart4,&USART_Receiver4,1);		//调试使用，勿管。开关在freertos.c


	if(Start_Flag == START)
	{
		if(USART_Receiver4 == 0xaa)                         //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         			//数据头两位 //buf[0]
			{
				Start_Flag = !START;             			//收到数据头，开始接收数据
				//printf("header ok\n");
				receivesend_buff[0]=serial_header[0];       //buf[0]
				receivesend_buff[1]=serial_header[1];		//buf[1]
				USARTsend_bufferIndex = 0;             		//缓冲区初始化
				checkSum = 0x00;				  			//校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver4;  
		}
	}
	else
    { 
		test =3;
		switch(USARTsend_bufferIndex)
		{
			case 0://接收数据的长度
			{
				receivesend_buff[2] = USART_Receiver4;
				dataLength     =  receivesend_buff[2];            //buf[2]      dateLength = 7 
				USARTsend_bufferIndex++;
				break;
			}
			case 1://接收所有数据，并赋值处理 
			{
					receivesend_buff[j + 3] = USART_Receiver4;	//buf[3] buf[4] buf[5] buf[6] buf[7] buf[8] buf[9]
					j++;
					if(j >= dataLength)    						//进入条件 j = 6                   
					{
						j = 0;									//置0
						USARTsend_bufferIndex++;
					}
					break;
			}
			case 2://接收校验值信息(设定为0x0b)
			{
				receivesend_buff[3 + dataLength] = USART_Receiver4;					//接收数据包中的校验值
				checkSum = serial_get_crc8_value(receivesend_buff, 3 + dataLength); //buf[10]
				
				// 检查信息校验值
				if (checkSum != receivesend_buff[3 + dataLength]) 					//buf[10]
				{
              
					debug_safe_printf("Received tem_array check sum error!\r\n");
					return 0;
				}
				USARTsend_bufferIndex++;
				break;
			}
				
			case 3://接收信息尾
			{
				if(k==0)
				{
					//数据0d     buf[10]  无需判断
					k++;
				}
				else if(k==1)
				{
					//数据0a     buf[11] 无需判断
					//进行赋值操作				
					*next_controller_state = receivesend_buff[3];
					*take_ball_flag = receivesend_buff[4];
                    for(k = 0; k < 4;k++)
                    {
                        //0 1 2 3              5 6 7 8
                        LADAR_YAW.tem_array[k] = receivesend_buff[k + 5];
                    }       
                    *yaw = LADAR_YAW.data;
                    *clean_ball_flag = receivesend_buff[9];
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTsend_bufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			}
			default:break;
		}
	}
	return 0;
}


/**
 * @brief 串口发送数据函数
* @param 当前控制器状态，球的状态，到达放球标志位，堵转标志位，侧边微动开关标志位，
*        启动开关，红蓝半场开关，正常或重启标志位，调试自动标志位，计数器
 * @return NULL
*/
void Usart_Send_Data(uint8_t now_controller_state,uint8_t ball_state,uint8_t arrive_flag,uint8_t lock_flag,uint8_t tough_flag,
 uint8_t start_flag,uint8_t red_blue_flag,uint8_t restart_flag,uint8_t if_auto_flag,uint8_t test_cnt)
{
    // 协议数据缓存数组
	unsigned char send_buf[6+10] = {0};
	int i, Length = 0;

    //设置消息头
    for (i = 0; i < 2; i++)
    {
        send_buf[i] = serial_header[i];
    }

    Length = 10; 
	send_buf[2] = Length;    

	//数据位赋值
	send_buf[3] = now_controller_state;	//buf[3]
	send_buf[4] = ball_state;	//buf[4]
	send_buf[5] = arrive_flag;	//buf[5]
	send_buf[6] = lock_flag;		//buf[6]
	send_buf[7] = tough_flag;		//buf[7]
    send_buf[8] = start_flag;       //buf[8]
    send_buf[9] = red_blue_flag;   //buf[9]
    send_buf[10] = restart_flag; //buf[10]
    send_buf[11] = if_auto_flag;   //buf[11]
    send_buf[12] = test_cnt; //buf[12]
    
      
    // 设置校验值、消息尾
    send_buf[3 + Length] = serial_get_crc8_value(send_buf, 3 + Length);	//send_buf[13]		3+10
    send_buf[3 + Length + 1] = serial_ender[0];							//send_buf[14]		4+10
    send_buf[3 + Length + 2] = serial_ender[1];							//send_buf[15]		5+10

    USART_Send_String(send_buf, sizeof(send_buf), UART4);
   
}

/**
 * @brief 发送字符数组（字符串）的函数
 * @param 数组地址(直接传入数组名字即可)
 * @param 数组长度(可以用sizeof)
 * @param 串口端口，例如USART1，USART2。。。
*/
void USART_Send_String(unsigned char *p_array, short sendSize, USART_TypeDef *usart)
{ 
	static int length =0;
	while(length<sendSize)
	{  
		while( !(usart->SR&(0x01<<7)) );//发送缓冲区为空
		usart->DR=*p_array;                   
		p_array++;
		length++;
	}
	length =0;
}

/**
 * @brief 校验位函数Crc8
 * @param 传入数组
 * @param 当前数组长度
 * @return 检验值
*/
unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len)
{
	unsigned char crc = 0;
	unsigned char i;
	while(len--)
	{
		crc ^= *tem_array++;
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

