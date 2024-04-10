#include "main.h"

void SendData_task(void *pvParameters)
{
	while(1)
	{
      //  speed = M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM;   //转数
      //  angle = M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE; //角度
      //  usartSendData(speed,2000,angle,1);              //发送数据（电机当前转速（2个），角度，1个控制位）
        Send_Action_Data(ACTION_GL_POS_DATA);    //发送action数据到ros
        vTaskDelay(10);
	}
}


