#ifndef __MOVE__H
#define __MOVE__H

#include "stm32f4xx_can.h"
#define COS60 0.500000f
#define COS30 0.866025f
#define COS45 0.707106f

#define RM_transition_MS (PI * WHEEL_R) / 570.0f      //转速与速度的转换  
#define MS_transition_RM 570.0f / (PI * WHEEL_R) //速度与转速的转换
#define WHEEL_R 0.152f / 2

// M3508电机编号
#define M3508_CHASSIS_MOTOR_1234_ID 0x200
#define M3508_CHASSIS_MOTOR_ID_1 0x201
#define M3508_CHASSIS_MOTOR_ID_2 0x202
#define M3508_CHASSIS_MOTOR_ID_3 0x203
#define M3508_CHASSIS_MOTOR_ID_4 0x204
#define M3508_CHASSIS_MOTOR_5678_ID 0x1FF
#define M3508_CHASSIS_MOTOR_ID_5 0x205
#define M3508_CHASSIS_MOTOR_ID_6 0x206
#define M3508_CHASSIS_MOTOR_ID_7 0x207
#define M3508_CHASSIS_MOTOR_ID_8 0x208

//速度规划结构体
typedef struct VELOCITY_PLANNING{
float Distance;
	float Pstart;        //开始位置
	float Pend;          //结束位置
	float Vstart;        //开始的速度           // 单位：RPM 绝对值
	float Vmax;          //最大的速度
	float Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
	int flag;            //完成标志位，电机停下来的时候置1
}VELOCITY_PLANNING;

//速度最大值
#define Vx_max 7000
#define Vy_max 7000

// M3508返回电机的真实信息
typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		    //采样角度			
	int16_t  RPM;				//实际转子转速				
	int16_t  CURRENT;           //电流
	int16_t  TARGET_CURRENT;   //目标电流
    int16_t  TARGET_RPM;       //目标转速
    int16_t TARGET_POS;        //目标距离
    int     Velflag;           //速度标志
    //转矩结构体
    VELOCITY_PLANNING velocity_planning;
    
	float  REAL_ANGLE;         //真实角度    
    uint8_t FIRST_ANGLE_INTEGRAL_FLAG;   //首次积分标志
    uint16_t LAST_ANGLE;       //上次角度
    int16_t filter_RPM;        //转速滤波
}M3508_REAL_INFO;

extern M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[8]; 



void Speed_Chassis_Calculate(void);
void M3508_Send_Motor_Currents1(void);
void M3508_Send_Motor_Currents2(void);
void m3508_update_m3508_info(CanRxMsg *msg);

#endif



