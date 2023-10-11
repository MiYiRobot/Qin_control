#include "main.h"
  int World_Speed[3];//x,y,w
//#define PI 3.1415926
#define COS 0.707106
#define SIN 0.707106

void CAN_motor_init(void)//配置can和初始化pid
{
    //can1初始化   
    CAN1_init();
    //四个轮子赋pid
    //PID_Parameter_Speed_Init(&M3508_PID[0],10.0,1.0,0.0 , 15000 ,0 , 0 , 500);
	
     PID_Parameter_Speed_Init(&M3508_PID[0],KP,KI,KD,OUT_MAX,DEAD_SIZE,I_SEPARATE,I_LIMIT);   //1号电机 
    //     PID_Parameter_Speed_Init(&M3508_PID[1],KP,KI,KD,OUT_MAX,DEAD_SIZE,I_SEPARATE,I_LIMIT);  //2号电机
    //     PID_Parameter_Speed_Init(&M3508_PID[2],KP,KI,KD,OUT_MAX,DEAD_SIZE,I_SEPARATE,I_LIMIT);  //3号电机
    //     PID_Parameter_Speed_Init(&M3508_PID[3],KP,KI,KD,OUT_MAX,DEAD_SIZE,I_SEPARATE,I_LIMIT);  //4号电机
} 


//给电机一个xyw的速度，让他以xyw速度移动
void Set_World_Speed_Trans(int World_Vx,int World_Vy,int World_Vw){
	int motor1_target_speed,motor2_target_speed,motor3_target_speed,motor4_target_speed;
    
//	M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT= -World_Vx*COS + World_Vy*COS + World_Vw;//左前轮  这个是开环控制
//	M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT= -World_Vx*COS - World_Vy*COS + World_Vw;//左后轮
//	M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT= World_Vx*COS - World_Vy*COS + World_Vw;//右前轮
//	M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT= World_Vx*COS + World_Vy*COS + World_Vw;//右后轮
	motor1_target_speed= -World_Vx*COS + World_Vy*COS + World_Vw;//左前轮
	motor2_target_speed= -World_Vx*COS - World_Vy*COS + World_Vw;//左后轮
	motor3_target_speed= World_Vx*COS - World_Vy*COS + World_Vw;//右前轮
	motor4_target_speed= World_Vx*COS + World_Vy*COS + World_Vw;//右后轮
	//PID闭环控制
	M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[0],motor1_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM);
	M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[1],motor2_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM);
	M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[0],motor3_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM);
	M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[0],motor4_target_speed , M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM);

}



