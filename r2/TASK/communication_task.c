#include "main.h"

void SendData_task(void *pvParameters)
{
	while(1)
	{
      //  speed = M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM;   //ת��
      //  angle = M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE; //�Ƕ�
      //  usartSendData(speed,2000,angle,1);              //�������ݣ������ǰת�٣�2�������Ƕȣ�1������λ��
        Send_Action_Data(ACTION_GL_POS_DATA);    //����action���ݵ�ros
        vTaskDelay(10);
	}
}


