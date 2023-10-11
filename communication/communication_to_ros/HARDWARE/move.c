#include "main.h"

//定义3508转态结构体
M3508_REAL_INFO  M3508_CHASSIS_MOTOR_REAL_INFO[4]; 		//底盘电机M3508的真正信息
uint16_t Speed_x=0,Speed_y=0,Speed_w=0;       //电机往xyw移动的速度
     
void Speed_Calculate(void)
{
    
	Set_World_Speed_Trans(Speed_x,Speed_y,Speed_w);     
	M3508_Send_Motor_Currents();
        
}    


//利用电机通过can反馈的数据更新m3508状态信息
//接受平率：1KHZ
void m3508_update_m3508_info(CanRxMsg *msg)
{
	switch(msg -> StdId)  // 检测标准ID
	{
        case M3508_CHASSIS_MOTOR_ID_1:
      	{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
        case M3508_CHASSIS_MOTOR_ID_2:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
        case M3508_CHASSIS_MOTOR_ID_3:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		case M3508_CHASSIS_MOTOR_ID_4:   //检测到电调ID
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		default: break;
	}
}

//can1发送底盘m3508的电流
//转子角度范围：0~8191				映射到0~8191
//转子转速RPM
//电机温度单位
void M3508_Send_Motor_Currents(void)
{
	CanTxMsg tx_message;

	//配置控制端
	tx_message.IDE = CAN_Id_Standard;   //标准帧
	tx_message.RTR = CAN_RTR_Data;      //标志校验帧，为正常帧
	tx_message.DLC = 0x08;              //0x02对应一个电机  0x08对应4个电机
	
	//配置仲裁段和数据段
	tx_message.StdId = M3508_CHASSIS_MOTOR_ALL_ID;  // 用于ID为 1 2 3 4的电机
    //消息的具体内容
	tx_message.Data[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	tx_message.Data[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT;
  
  tx_message.Data[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	tx_message.Data[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT;
  
  tx_message.Data[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	tx_message.Data[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT;
  
  tx_message.Data[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	tx_message.Data[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT;
  
	CAN_Transmit(CAN1, &tx_message);  // 发送指令
}




