#ifndef __MAIN_H
#define __MAIN_H
//ϵͳ���������ͷ�ļ�
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
//�Լ�д��ͷ�ļ�
#include "chassis.h"
#include "pid.h"
#include "can.h"
#include "move.h"
#include "led.h"
#include "communication.h"
#include "action.h"
#include "remove_control.h"
#include "upper.h"

//�������к궨��
#define  ABS(x)      ((x)>0? (x):(-(x)))   //����ֵ
#define KP 3    //p
#define KI 0.0       //i
#define KD 0.0      //d
#define OUT_MAX    1000    //�����޷�
#define DEAD_SIZE  0.5    //������С
#define I_SEPARATE 1000      //���ַ���
#define I_LIMIT 1000       //�����޷�

//����ö��ѡ��ģʽ
typedef enum Control_Mode
{
    remt_ctrl_mode,    //��ģң����
    go_to_p_mode,      //����ĳ��λ��
    auto_mode          //�Զ�ģʽ��ros��
}Control_Mode;

extern Control_Mode Mode;

extern uint16_t PPM_Databuf[10];   //ң�����ź�

extern float Speed_x,Speed_y,Speed_w;   //��������ٶ�
extern float Cmd_x,Cmd_y,Cmd_w;         //ros����������ٶ���Ϣ
extern uint8_t cmd_flag;                //ros���Ʊ�־
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
