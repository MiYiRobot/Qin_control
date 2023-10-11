#ifndef __MAIN_H
#define __MAIN_H
//系统配置所需的头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"
//自己写得头文件
#include "chassis.h"
#include "pid.h"
#include "can.h"
#include "move.h"
#include "led.h"
#include "communication.h"

//定义所有宏定义
#define  ABS(x)      ((x)>0? (x):(-(x)))   //绝对值
#define KP 9    //p
#define KI 0.01       //i
#define KD 0.0      //d
#define OUT_MAX    15000    //电流限幅
#define DEAD_SIZE  1    //死区大小
#define I_SEPARATE 5000      //积分分离
#define I_LIMIT 500       //积分限幅

extern int16_t speed;
extern float angle;
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
