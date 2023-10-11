#include "main.h"

//任务句柄
TaskHandle_t StartTask_Handler;   
TaskHandle_t LED0Task_Handler;
TaskHandle_t LED1Task_Handler;
TaskHandle_t SendDataTASK_Handler;
TaskHandle_t moveTask_Handler;
int16_t speed;
float angle;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);		//初始化延时函数
	uart_init(115200);     	//初始化串口
	LED_Init();		        //初始化LED端口
    Speed_x=1000;
    CAN_motor_init();       //初始化can
    receiveinit();        
	
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )128,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )1,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}
 
//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建LED0任务
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )54, 
                (void*          )NULL,				
                (UBaseType_t    )2,	
                (TaskHandle_t*  )&LED0Task_Handler);   
    //创建LED1任务
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )54, 
                (void*          )NULL,
                (UBaseType_t    )3,
                (TaskHandle_t*  )&LED1Task_Handler);        
    //发送数据任务
    xTaskCreate((TaskFunction_t )SendData_task,     
                (const char*    )"usart_task",   
                (uint16_t       )128, 
                (void*          )NULL,
                (UBaseType_t    )4,
                (TaskHandle_t*  )&SendDataTASK_Handler);  
    //电机运动任务
    xTaskCreate((TaskFunction_t )move_task,
                (const char*    )"move_task",
                (uint16_t       )256,
                (void*          )NULL,
                (UBaseType_t    )5,
                (TaskHandle_t*  )&moveTask_Handler);
                
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}




