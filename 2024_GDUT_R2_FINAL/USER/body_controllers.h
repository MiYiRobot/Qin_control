#ifndef __BODY_CONTROLLERS_H
#define __BODY_CONTROLLERS_H

#include "main.h"

typedef enum
{   
    TAKE_BALL = 2,  //取球        2
    SHOOT_BALL,     //出球        3
    CONTROLLER_OFF, //关闭        4
    CONTROLLER_ERROR        //错误状态  5
}CONTROLLER_STATE;

typedef enum 
{
    // UP = 0,     //最上面检测到
    // BENEATH,    //最下面检测到
    // MIDDLE,      //中间检测到
    // B_M_BOTH    //中间和下面都有球

    UP = 0,//最上面检测到
    NOT_UP//最上面没有检测到
}PHOTOGATE;


typedef enum
{
    BALL_OUTSIDE = 0,    //未拿到球（放球成功以及第一次进入3区时的状态） 0
    BALL_HAVE_TAKEN,   //已拿到球(机器人拿到球，但夹爪还没碰到微动开关)    1 
    BALL_IN_CLAW,    //拿到球且夹爪到位（夹爪碰到微动开关）  2
}BALL_STATE;


typedef enum
{
    INVERSE_BALL = 0, //反转
    LEFT_FILTER_BALL  = 1,   //左筛球
    RIGHT_FILTER_BALL = 2,   //右筛球
}TAKE_BALL_STATE;

//夹爪状态
typedef enum
{
    CLAW_OPEN = 0,                //夹爪还没拍到微动开关
    CLAW_CLOSE                  //夹爪拍到微动开关  
}CLAW_STATE;

//灰度放球状态
typedef enum
{
    NO_DETECTION = 0,       //没检测到
    ALIGNED = 1,    //对齐
    PUT_LEFT,           //左偏
    PUT_RIGHT           //右偏
}HUIDU_PUT;

//有效球状态
typedef enum
{
    NO_USEFULBALL = 0,    //无有效球
    USEFULBALL_ENTER = 1, //有效球进来
    USEFULBALL_MIDDLE = 2     //有效球在中间
}USEFUL_BALL_STATUS;

typedef enum{
    BALL_NO = 0,
    BALL_inner,
    BALL_middle,
    BALL_BOTH
      
}BALL_POSITION;

typedef struct R2_CONTROLLER_STR
{
    uint8_t ros_stm32_noconnected_cnt;
    uint8_t ros_stm32_noconnected_flag;

    PHOTOGATE Gate_state;    //光电开关状态
    BALL_STATE Ball_sta; //是否有球在车上  
    HUIDU_PUT HuiDu_Put;     //灰度放球标志位 
   // BALL_COLOR ball_color;   //检测到球的颜色  0是无球，1是蓝球，2是紫球，3是红球
    uint8_t HuiDu_Arrive;     //灰度传感器到达位置标志位        0为未到达，1为到达 
    uint8_t Huidu_Flag;  //灰度传感器标志位 0为全都没检测到 5为正好中间，4~2为向左偏，数值越小偏越多；6~8为向右偏，数值越大偏越多  
    CLAW_STATE claw_state;
    USEFUL_BALL_STATUS useful_ball_status;         //有效球是否在入口，0为没有在入口，1为在入口
    uint8_t ball_on_up;            //球在上面  0为没在上面，1为在上面（最上面的光电门检测到）
    uint8_t lock_flag;      //摩擦带堵转标志位
    uint8_t put_ball_arrival;//放球到地方的标志位，1为到地方
    BALL_POSITION ball_position;  //球的位置
    float robot_yaw;   //机器人偏航角
    uint8_t clean_ball_flag;  //清球开启标志位  
    
    TAKE_BALL_STATE filter_ball_state; //吸球状态
    ROBOT_CHASSIS CHASSIS_CONTROLLER;
    CONTROLLER_STATE NEXT_CONTROLLER_STATE; //上层机构的下个状态，ros决定
    CONTROLLER_STATE NOW_CONTROLLER_STATE;  //上层机构的当前状态
}R2_CONTROLLER_STR;



extern R2_CONTROLLER_STR R2_CONTROLLER;
void Chassis_Controller(void);
void ROS_Control(void);
//void liner_actuator(void);
CONTROLLER_STATE upper_controller(CONTROLLER_STATE NEXT_STATE);
void motor_controller(int16_t right_3508_rpm,int16_t left_3508_rpm,int16_t back_motor_rpm,
                      int16_t Below_Roller_rpm,int16_t Middle_Roller_rpm,int16_t Up_Roller_rpm,int16_t filler_3508_rpm,int16_t front_2006_rpm);
void take_ball(void);
void Claw_PutDown(void);
void Get_Ball(void);
void ball_judge(void);
#endif
