#include "main.h"

//LED0任务函数 
void led0_task(void *pvParameters)
{
    while(1)
    {
        LED0=~LED0;
        vTaskDelay(500);
    }
}   

//LED1任务函数
void led1_task(void *pvParameters)
{
    while(1)
    {
        LED1=0;
        vTaskDelay(200);
        LED1=1;
        vTaskDelay(800);
    }
}

//浮点测试任务
//void usart_task(void *pvParameters)
//{
//	
//	while(1)
//	{
//        speed =  -M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM;
//		printf("%d\n",speed);
//        vTaskDelay(5);
//	}
//}

