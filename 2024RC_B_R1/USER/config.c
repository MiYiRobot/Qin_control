//
// Created by Ray on 2023/11/26.
//

#include "config.h"

/**
 * @brief 底盘和两个6020的电机类型初始化
 * 
 */
void can1_config(void)
{
    can1motorRealInfo[0].Motor_Type = M_3508;//底盘右后轮
    can1motorRealInfo[1].Motor_Type = M_3508;//底盘右前轮
    can1motorRealInfo[2].Motor_Type = M_3508;//底盘左前轮
    can1motorRealInfo[3].Motor_Type = M_3508;//底盘左后轮
    can1motorRealInfo[4].Motor_Type = M_6020;//左6020转动电机      ID编号 205
    can1motorRealInfo[5].Motor_Type = M_3508;//左秧苗抬升     ID编号 206
    can1motorRealInfo[6].Motor_Type = M_3508;//左凹轮     ID编号 207
    //6020位置环单圈模式
    can1motorRealInfo[4].once_flag = 1;   
    //底盘只需要速度环
    pid_param_init(&can1MOTOR_PID_RPM[0], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[1], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[2], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[3], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);

    //左6020的位置环和速度环
    pid_param_init(&can1MOTOR_PID_RPM[4], PID_Position, 8192, 1024, 0, 0.5, 16384, 0.5f, 0.0f, 0.2f);
    pid_param_init(&can1MOTOR_PID_POS[4], PID_Position, 8192, 800, 0, 0.1f, 16384, 10, 0.0, 0.3); 
    //秧苗抬升机构
    pid_param_init(&can1MOTOR_PID_RPM[5], PID_Incremental, 10000, 1024, 8192, -0.5,8192, 12.0f, 0.4f, 0.0f);
    pid_param_init(&can1MOTOR_PID_POS[5], PID_Position, 1024, 800, 0, 0.1f, 16384, 8, 0, 0.2);
    //左凹轮的位置环和速度环
    pid_param_init(&can1MOTOR_PID_RPM[6], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can1MOTOR_PID_POS[6], PID_Position, 1024, 800, 0, 0.1f, 16384, 12, 0, 0.3);
}

/**
 * @brief can2初始化
 * 
 */
void can2_config(void)
{
    can2motorRealInfo[0].Motor_Type = M_3508;//发射机构履带
    can2motorRealInfo[1].Motor_Type = M_3508;//发射机构履带
    can2motorRealInfo[2].Motor_Type = M_3508;//夹爪转动电机
    can2motorRealInfo[3].Motor_Type = M_3508;//发射云台
    can2motorRealInfo[4].Motor_Type = M_6020;//6020转动电机   ID编号 205
    can2motorRealInfo[5].Motor_Type = M_3508;//右秧苗抬升
    can2motorRealInfo[6].Motor_Type = M_3508;//右凹轮

    can2motorRealInfo[4].once_flag = 1;     //6020位置环单圈模式
    //履带速度环PID
    pid_param_init(&can2MOTOR_PID_RPM[0], PID_Position, 10000, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can2MOTOR_PID_RPM[1], PID_Position, 10000, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    //夹爪转动电机位置环和速度环
    pid_param_init(&can2MOTOR_PID_RPM[2], PID_Incremental, 10000, 1024, 8192, -0.5,8192, 12.0f, 0.4f, 0.0f);
    pid_param_init(&can2MOTOR_PID_POS[2], PID_Position, 1024, 800, 0, 0.1f, 16384, 3, 0, 0);
    //云台位置环和速度环
    pid_param_init(&can2MOTOR_PID_RPM[3], PID_Incremental, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.2f, 0.2f);
    pid_param_init(&can2MOTOR_PID_POS[3], PID_Position, 2048, 800, 0, 0.1f, 16384, 15, 0, 0.3);
    //右6020的位置环和速度环
    pid_param_init(&can2MOTOR_PID_RPM[4], PID_Position, 8192, 1024, 0, 0.5, 16384, 1.0f, 0.0f, 0.1f);
    pid_param_init(&can2MOTOR_PID_POS[4], PID_Position, 8192, 800, 0, 0.1f, 16384, 10, 0.0, 0.3);
    //秧苗抬升机构
    pid_param_init(&can2MOTOR_PID_RPM[5], PID_Incremental, 10000, 1024, 8192, -0.5,8192, 12.0f, 0.4f, 0.0f);
    pid_param_init(&can2MOTOR_PID_POS[5], PID_Position, 1024, 800, 0, 0.1f, 16384, 8, 0, 0.2);
    //右凹轮的位置环和速度环
    pid_param_init(&can2MOTOR_PID_RPM[6], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can2MOTOR_PID_POS[6], PID_Position, 1024, 800, 0, 0.1f, 16384, 12, 0, 0.3);

}


