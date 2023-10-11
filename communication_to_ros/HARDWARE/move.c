#include "main.h"

//����3508ת̬�ṹ��
M3508_REAL_INFO  M3508_CHASSIS_MOTOR_REAL_INFO[4]; 		//���̵��M3508��������Ϣ
uint16_t Speed_x=0,Speed_y=0,Speed_w=0;       //�����xyw�ƶ����ٶ�
     
void Speed_Calculate(void)
{
    
	Set_World_Speed_Trans(Speed_x,Speed_y,Speed_w);     
	M3508_Send_Motor_Currents();
        
}    


//���õ��ͨ��can���������ݸ���m3508״̬��Ϣ
//����ƽ�ʣ�1KHZ
void m3508_update_m3508_info(CanRxMsg *msg)
{
	switch(msg -> StdId)  // ����׼ID
	{
        case M3508_CHASSIS_MOTOR_ID_1:
      	{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
        case M3508_CHASSIS_MOTOR_ID_2:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
        case M3508_CHASSIS_MOTOR_ID_3:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		case M3508_CHASSIS_MOTOR_ID_4:   //��⵽���ID
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		default: break;
	}
}

//can1���͵���m3508�ĵ���
//ת�ӽǶȷ�Χ��0~8191				ӳ�䵽0~8191
//ת��ת��RPM
//����¶ȵ�λ
void M3508_Send_Motor_Currents(void)
{
	CanTxMsg tx_message;

	//���ÿ��ƶ�
	tx_message.IDE = CAN_Id_Standard;   //��׼֡
	tx_message.RTR = CAN_RTR_Data;      //��־У��֡��Ϊ����֡
	tx_message.DLC = 0x08;              //0x02��Ӧһ�����  0x08��Ӧ4�����
	
	//�����ٲöκ����ݶ�
	tx_message.StdId = M3508_CHASSIS_MOTOR_ALL_ID;  // ����IDΪ 1 2 3 4�ĵ��
    //��Ϣ�ľ�������
	tx_message.Data[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	tx_message.Data[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT;
  
  tx_message.Data[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	tx_message.Data[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT;
  
  tx_message.Data[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	tx_message.Data[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT;
  
  tx_message.Data[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	tx_message.Data[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT;
  
	CAN_Transmit(CAN1, &tx_message);  // ����ָ��
}




