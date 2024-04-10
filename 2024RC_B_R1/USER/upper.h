//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_UPPER_H
#define INC_2024RC_B_R1_UPPER_H

#include "rm_motor.h"
//夹苗电机及角度参数
typedef struct SEED_FLAG
{
    uint8_t state;                  //夹苗机构状态  
    uint8_t seed_aim_flag;          //夹苗是对准标志位 0为云台移动到储苗位置，1时云台移动到拿苗放苗位置 
    uint8_t desposit_first_flag;    //储苗机构第一次进入标志位   0为第一次进入，1为后面进入
    uint8_t put_first_flag;         //放苗机构第一次进入标志位   0为第一次进入，1为后面进入
    uint8_t rotate_change_flag;      //凹轮电机转动角度改变标志位
    int16_t rotate_flag;            //凹轮电机转动次数标志位
}SEED_FLAG;

extern SEED_FLAG Seed_Flag;  //夹苗机构状态结构体
//秧苗控制状态结构体        
typedef struct SEED_CONTROL_STATE
{
    MOTOR_REAL_INFO *rotate_motor;  //6020转动电机结构体
    MOTOR_REAL_INFO *cam_motor;     //凹轮电机结构体
    MOTOR_REAL_INFO *lift_motor; //抬苗电机结构体
    uint8_t state;                  //夹苗机构各部分状态  0为判断状态，1为运动到存苗位置，2为夹苗上升并运动到放苗位置，3为放苗下降并放苗 
    uint8_t seed_aim_flag;          //夹苗是对准标志位 0为云台移动到拿苗放苗位置，1时夹爪夹紧，云台移动到储苗位置    
    float peek_angle;               //夹苗6020的角度位置
    float deposit_angle;            //存苗6020的角度位置
    float put_angle;                //凹轮电机开始动时6020的角度位置（未用） 
    float lift_angle;               //抬升角度           
}SEED_CONTROL_STATE;

extern SEED_CONTROL_STATE Left_Seed_Control;  //左秧苗控制状态结构体
extern SEED_CONTROL_STATE Right_Seed_Control; //右秧苗控制状态结构体
//云台相关结构体
typedef struct GIMBAL
{
    int16_t target_point[2];    //目标点
    int16_t revole_rpm;         //云台转速
    float TAN;                  //
    double Theta;               
    float Descripe;             //减数比
    float gimbal_angle;         //云台角度
    uint8_t centor_flag;        //是否回正标志位 
    uint8_t aim_flag;           //瞄准成功标志位
}GIMBAL;

extern GIMBAL Gimbal; 
extern int16_t Theta;  //车体相对于目标点的切角
extern int16_t Alpha;
void Init_Seed(void);        //夹秧苗初始化函数
void Deposit_Seed(void);     //拿苗存苗
void Put_Seed(void);         //放苗
void seed_control(void);
void Shoot_Ball(void);  //射球函数
void Load_Ball(void);   //装球函数    
void Init_Ball(void);   //发射机构初始化函数
void Gimbal_controller(void);      //云台计算函数                    
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time);
#endif //INC_2024RC_B_R1_UPPER_H
