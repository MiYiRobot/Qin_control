#include "main.h"

//定义3508转态结构体
M3508_REAL_INFO  M3508_CHASSIS_MOTOR_REAL_INFO[8]; 		//底盘电机M3508的真正信息
float Speed_x=0.0,Speed_y=0.0,Speed_w=0.0;       //电机往xyw移动的速度
 //解算并发送速度电流信息给底盘    
void Speed_Chassis_Calculate(void)
{  
    Speed_x = -((PPM_Databuf[1]-1500.0f)/500)*Vx_max;//世界坐标下速度映射
    Speed_y = ((PPM_Databuf[0]-1500.0f)/500)*Vy_max;
    Speed_w =((PPM_Databuf[3]-1500.0f)/500)*Vx_max;
	Set_World_Speed_Trans();     
	M3508_Send_Motor_Currents1();
  printf("%d,%d,%d,%d\n",M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM,M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM,M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM,M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM);
    
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
        case M3508_CHASSIS_MOTOR_ID_5:
      	{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[4].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[4].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
        case M3508_CHASSIS_MOTOR_ID_6:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[5].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
        case M3508_CHASSIS_MOTOR_ID_7:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[6].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[6].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[6].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		case M3508_CHASSIS_MOTOR_ID_8:   //检测到电调ID
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[7].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[7].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[7].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		default: break;
	}
}

//can1发送底盘m3508的电流            ID为1234电机
//转子角度范围：0~8191				映射到0~8191
//转子转速RPM
//电机温度单位
void M3508_Send_Motor_Currents1(void)
{
	CanTxMsg tx_message;

	//配置控制端
	tx_message.IDE = CAN_Id_Standard;   //标准帧
	tx_message.RTR = CAN_RTR_Data;      //标志校验帧，为正常帧
	tx_message.DLC = 0x08;              //0x02对应一个电机  0x08对应4个电机
	
	//配置仲裁段和数据段
	tx_message.StdId = M3508_CHASSIS_MOTOR_1234_ID;  // 用于ID为 1 2 3 4的电机
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


//can1发送底盘m3508的电流            ID为1234电机
//转子角度范围：0~8191				映射到0~8191
//转子转速RPM
//电机温度单位
void M3508_Send_Motor_Currents2(void)
{
	CanTxMsg tx_message;

	//配置控制端
	tx_message.IDE = CAN_Id_Standard;   //标准帧
	tx_message.RTR = CAN_RTR_Data;      //标志校验帧，为正常帧
	tx_message.DLC = 0x08;              //0x02对应一个电机  0x08对应4个电机
	
	//配置仲裁段和数据段
	tx_message.StdId = M3508_CHASSIS_MOTOR_5678_ID;  // 用于ID为 1 2 3 4的电机
    //消息的具体内容
	tx_message.Data[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT >> 8);
	tx_message.Data[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT;
  
  tx_message.Data[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT >> 8);
	tx_message.Data[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT;
  
  tx_message.Data[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT >> 8);
	tx_message.Data[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT;
  
  tx_message.Data[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[7].TARGET_CURRENT >> 8);
	tx_message.Data[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[7].TARGET_CURRENT;
  
	CAN_Transmit(CAN1, &tx_message);  // 发送指令
}


