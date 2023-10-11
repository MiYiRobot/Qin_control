#ifndef __MAIN_H
#define __MAIN_H
//ϵͳ���������ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"
//�Լ�д��ͷ�ļ�
#include "chassis.h"
#include "pid.h"
#include "can.h"
#include "move.h"
#include "led.h"
#include "communication.h"

//�������к궨��
#define  ABS(x)      ((x)>0? (x):(-(x)))   //����ֵ
#define KP 9    //p
#define KI 0.01       //i
#define KD 0.0      //d
#define OUT_MAX    15000    //�����޷�
#define DEAD_SIZE  1    //������С
#define I_SEPARATE 5000      //���ַ���
#define I_LIMIT 500       //�����޷�

extern int16_t speed;
extern float angle;
//�������е�FreeRTOS����
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
