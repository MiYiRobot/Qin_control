#include "main.h"
//#define PI 3.1415926
#define COS 0.707106
#define SIN 0.707106

void CAN_motor_init(void)//配置can和初始化pid
{
    //can1初始化   
    CAN1_init();
    //四个轮子赋pid   
    // PID初始化              PID结构体     P I  D 输出限幅  死区 积分分离  积分限幅                       
	PID_Parameter_Speed_Init(&M3508_PID[0], 4,0, 0, 16000,10, 1000, 10000);//3 0 1.5   左前轮
	PID_Parameter_Speed_Init(&M3508_PID[1], 3,0, 0, 16000,10, 1000, 10000);//3 0 1.5   左后轮
	PID_Parameter_Speed_Init(&M3508_PID[2], 4,0, 0, 16000,10, 1000, 10000);//3 0 1.5   右后轮
	PID_Parameter_Speed_Init(&M3508_PID[3], 3,0, 0, 16000,10, 1000, 10000);//3 0 1.5   右前轮
//抬升机构速度环
	PID_Parameter_Speed_Init(&M3508_PID[4], 10,0.1, 0, 4000,0, 1000, 10000);
	//摩擦轮速度环
	PID_Parameter_Speed_Init(&M3508_PID[5], 10,0, 0, 6000,0, 1000, 10000);//3 0 1.5   摩擦轮
	PID_Parameter_Speed_Init(&M3508_PID[6], 10,0, 0, 6000,0, 1000, 10000);//3 0 1.5   摩擦轮
	PID_Parameter_Speed_Init(&M3508_PID[7], 1.5,0, 0, 16000,0, 1000, 10000);     //抬升
    
} 


//给电机一个xyw的速度，让他以xyw速度移动   xy 平移速度  w 转动速度
void Set_World_Speed_Trans(void)
{
	float motor1_target_speed,motor2_target_speed,motor3_target_speed,motor4_target_speed;

	motor1_target_speed= -Speed_x*COS + Speed_y*COS + Speed_w;//左前轮
	motor2_target_speed= -Speed_x*COS - Speed_y*COS + Speed_w;//左后轮
	motor3_target_speed= Speed_x*COS - Speed_y*COS + Speed_w;//右后轮
	motor4_target_speed= Speed_x*COS + Speed_y*COS + Speed_w;//右前轮
	//PID闭环控制，计算输出电流数据
	M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[0],motor1_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM);
	M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[1],motor2_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM);
	M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[2],motor3_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM);
	M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[3],motor4_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM);

}



