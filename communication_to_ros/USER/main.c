#include "main.h"

//������
TaskHandle_t StartTask_Handler;   
TaskHandle_t LED0Task_Handler;
TaskHandle_t LED1Task_Handler;
TaskHandle_t SendDataTASK_Handler;
TaskHandle_t moveTask_Handler;
int16_t speed;
float angle;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	delay_init(168);		//��ʼ����ʱ����
	uart_init(115200);     	//��ʼ������
	LED_Init();		        //��ʼ��LED�˿�
    Speed_x=1000;
    CAN_motor_init();       //��ʼ��can
    receiveinit();        
	
	//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )128,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )1,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}
 
//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //����LED0����
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )54, 
                (void*          )NULL,				
                (UBaseType_t    )2,	
                (TaskHandle_t*  )&LED0Task_Handler);   
    //����LED1����
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )54, 
                (void*          )NULL,
                (UBaseType_t    )3,
                (TaskHandle_t*  )&LED1Task_Handler);        
    //������������
    xTaskCreate((TaskFunction_t )SendData_task,     
                (const char*    )"usart_task",   
                (uint16_t       )128, 
                (void*          )NULL,
                (UBaseType_t    )4,
                (TaskHandle_t*  )&SendDataTASK_Handler);  
    //����˶�����
    xTaskCreate((TaskFunction_t )move_task,
                (const char*    )"move_task",
                (uint16_t       )256,
                (void*          )NULL,
                (UBaseType_t    )5,
                (TaskHandle_t*  )&moveTask_Handler);
                
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}




