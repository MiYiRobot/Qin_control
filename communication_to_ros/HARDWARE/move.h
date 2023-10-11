#ifndef __MOVE__H
#define __MOVE__H

#include <stm32f4xx.h>
#include "stm32f4xx_can.h"
#define COS60 0.500000f
#define COS30 0.866025f
#define COS45 0.707106f

#define RM_transition_MS (PI * WHEEL_R) / 570.0f      //转速与速度的转换  
#define MS_transition_RM 570.0f / (PI * WHEEL_R) //速度与转速的转换
#define WHEEL_R 0.152f / 2

// M3508电机编号
#define M3508_CHASSIS_MOTOR_ALL_ID 0x200
#define M3508_CHASSIS_MOTOR_ID_1 0x201
#define M3508_CHASSIS_MOTOR_ID_2 0x202
#define M3508_CHASSIS_MOTOR_ID_3 0x203
#define M3508_CHASSIS_MOTOR_ID_4 0x204

// M3508返回电机的真实信息
typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		    //采样角度			
	int16_t  RPM;				//实际转子转速				
	int16_t  CURRENT;           //电流
	int16_t  TARGET_CURRENT;   //目标电流
    int16_t  TARGET_RPM;       //目标转速
	float  REAL_ANGLE;         //真实角度
}M3508_REAL_INFO;

extern M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[4]; 

//机器人底盘数据结构体
typedef struct ROBOT_CHASSIS
{

	float World_V[3]; // X , Y , W
	float Robot_V[2];
	float Position[2];
	float Motor_RPM[4];  //最多四轮
	float expect_angle ;
	float Angle;
} ROBOT_CHASSIS;

extern uint16_t Speed_x,Speed_y,Speed_w;   //各方向的速度

void Speed_Calculate(void);
void M3508_Send_Motor_Currents(void);
void m3508_update_m3508_info(CanRxMsg *msg);

#endif



