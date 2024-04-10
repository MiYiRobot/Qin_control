#ifndef __MAIN_H
#define __MAIN_H
//系统配置所需的头文件
#include "stm32f4xx.h"   
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"
#include <math.h>
#include "stdlib.h"
//自己写得头文件
#include "chassis.h"
#include "pid.h"
#include "can.h"
#include "move.h"
#include "led.h"
#include "communication.h"
#include "action.h"
#include "remove_control.h"
#include "upper.h"

//定义所有宏定义
#define  ABS(x)      ((x)>0? (x):(-(x)))   //绝对值
#define KP 3    //p
#define KI 0.0       //i
#define KD 0.0      //d
#define OUT_MAX    1000    //电流限幅
#define DEAD_SIZE  0.5    //死区大小
#define I_SEPARATE 1000      //积分分离
#define I_LIMIT 1000       //积分限幅

//定义枚举选择模式
typedef enum Control_Mode
{
    remt_ctrl_mode,    //航模遥控器
    go_to_p_mode,      //到达某个位置
    auto_mode          //自动模式，ros控
}Control_Mode;

extern Control_Mode Mode;

extern uint16_t PPM_Databuf[10];   //遥控器信号

extern float Speed_x,Speed_y,Speed_w;   //各方向的速度
extern float Cmd_x,Cmd_y,Cmd_w;         //ros传输过来的速度信息
extern uint8_t cmd_flag;                //ros控制标志
//定义所有的FreeRTOS任务
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t LED0Task_Handler;
extern TaskHandle_t LED1Task_Handler;
extern TaskHandle_t SendDataTASK_Handler;
extern TaskHandle_t moveTask_Handler;

void start_task(void *pvParameters);
void led0_task(void *pvParameters);
void led1_task(void *pvParameters);
void SendData_task(void *pvParameters);
void move_task(void *pvParameters);
#endif
