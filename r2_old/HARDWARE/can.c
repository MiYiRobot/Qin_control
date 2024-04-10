#include "main.h"

// CAN1
// PA11 -> CANRX
// PA12 -> CANTX

void CAN1_init(void)
{
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;

	//开时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	//配置IO口
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

	gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &gpio);
	
	//配置工作模式
	CAN_DeInit(CAN1);
	CAN_StructInit(&can);
	
	can.CAN_TTCM = DISABLE;   //非时间触发通讯模式
	can.CAN_ABOM = DISABLE;   //使用该模式可以在节点出错离线后适时的自动恢复，不需要软件干涉
	can.CAN_AWUM = DISABLE;   //使用该功能可以在监测到总线活动后自动唤醒
	can.CAN_NART = DISABLE;   //DISABLE代表的是使用自动重传的功能，ENABLE是代表不使用自动重传的功能
	can.CAN_RFLM = DISABLE;   //是否锁定FIFO,如果锁定，FIFO溢出会丢弃新数据；如果不锁定，FIFO溢出时，新数据会覆盖旧数据
	can.CAN_TXFP = ENABLE;    //使能时会以存入发送邮箱的顺序进行发送，失能时，以报文ID的优先级发送
	can.CAN_Mode = CAN_Mode_Normal;    //正常模式
	//以下三个值用来配置波特率
    can.CAN_SJW  = CAN_SJW_1tq;     //定义位段加长或缩短的上限，在1到4个时间片上调整
	can.CAN_BS1 = CAN_BS1_9tq;      //定义采样点的位置，持续长度在1到16个时间片
	can.CAN_BS2 = CAN_BS2_4tq;      //定义发送点的位置，持续长度在1到8个时间片
	can.CAN_Prescaler = 3;   //波特率 ：42/(1+9+4)/3=1Mbps
	CAN_Init(CAN1, &can);

    //筛选器，设置为掩码模式，全部ID都接收
	can_filter.CAN_FilterNumber = 0;      //过滤器选择组别为0
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;  //掩码模式
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit; //过滤器长度
	can_filter.CAN_FilterIdHigh = 0x0000;     //设置验证码高低各4字节
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000; //设置屏蔽码高低各4字节
	can_filter.CAN_FilterMaskIdLow = 0x0000;
//	can_filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;  //过滤器0关联到FIFO0
    can_filter.CAN_FilterFIFOAssignment = 00;  //过滤器0关联到FIFO0
	can_filter.CAN_FilterActivation=ENABLE;       //
	CAN_FilterInit(&can_filter);
	
	//中断控制
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	//使能中断
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  //接收中断
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);   //发送中断
  
  //M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = 1000;
}
unsigned char can1_tx_success_flag = 0 , can2_tx_success_flag = 0;  //判断消息是否发送成功
// CAN1发送中断
void CAN1_TX_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{	

	    CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		can1_tx_success_flag = 1 ;
    }
}


//can1的FIFO0接收中断
void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg CAN1_RX0_message;  //临时存放数据结构体
  if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)    //收到数据
  {
		CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RX0_message);  // 读取数据
		
		m3508_update_m3508_info(&CAN1_RX0_message);  // M3508电机数据处理
	    //角度积分	
        M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[0]);//角度积分
        M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[1]);//角度积分
        M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[2]);//角度积分
        M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[3]);//角度积分
        M3508AngleIntegral4(&M3508_CHASSIS_MOTOR_REAL_INFO[4]);//角度积分
        M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[5]);//角度积分
        M3508AngleIntegral(&M3508_CHASSIS_MOTOR_REAL_INFO[6]);//角度积分            
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);  //清除标志位
  }
}

/**
  * @brief  角度积分，使传回来的3508角度不清零，角度数据累加
  * @param  3508结构体指针
  * @retval 无
  */
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR)
{
        float delta_pos = 0;
        
        //记录第一次进入时的数据
        if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
        {
             M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;   
             M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;  
             return;
        }
        //计算变化的角度   
        if(M3508_MOTOR->RPM >= 0)     //正转
        {
              if(ABS(8191 + M3508_MOTOR ->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)   // 利用两次CAN接收时间电机最大转动角度进行滤波
              {
               	delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;//已经转完一圈了，前面要加8191
				delta_pos = delta_pos / 19;	                            //减速比                
              }
        }
        else     //反转
        {
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//减速比                
        }
        
        //滤波
        if(delta_pos > 0)
        {
            M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
        }
        else
        {
            if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
            {
                if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
                {
                    delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
                    delta_pos = delta_pos /19;	//减速比
                }
            }	
            else
            {
                delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
                delta_pos = delta_pos / 19;	//减速比
            }
            
            // 滤波
            if(delta_pos < 0)
            {
                M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
            }
           }  
        //存储角度值
         M3508_MOTOR->LAST_ANGLE = M3508_MOTOR ->ANGLE;
}
void M3508AngleIntegral4(M3508_REAL_INFO* M3508_MOTOR){
  
  	float delta_pos = 0;
	
	// 记录第一次进入时的数据
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// 计算变化的角度
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 19;	//减速比
			}
		}
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//减速比
		}
		
		// 滤波
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分	
	}
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos /19;	//减速比
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//减速比
		}
		
		// 滤波
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
	}

	// 存储角度值 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}
  

