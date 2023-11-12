#include "main.h"

//����3508ת̬�ṹ��
M3508_REAL_INFO  M3508_CHASSIS_MOTOR_REAL_INFO[8]; 		//���̵��M3508��������Ϣ
float Speed_x=0.0,Speed_y=0.0,Speed_w=0.0;       //�����xyw�ƶ����ٶ�
 //���㲢�����ٶȵ�����Ϣ������    
void Speed_Chassis_Calculate(void)
{  
    Speed_x = -((PPM_Databuf[1]-1500.0f)/500)*Vx_max;//�����������ٶ�ӳ��
    Speed_y = ((PPM_Databuf[0]-1500.0f)/500)*Vy_max;
    Speed_w =((PPM_Databuf[3]-1500.0f)/500)*Vx_max;
	Set_World_Speed_Trans();     
	M3508_Send_Motor_Currents1();
  printf("%d,%d,%d,%d\n",M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM,M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM,M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM,M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM);
    
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
        case M3508_CHASSIS_MOTOR_ID_5:
      	{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[4].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[4].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
        case M3508_CHASSIS_MOTOR_ID_6:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[5].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
        case M3508_CHASSIS_MOTOR_ID_7:
        { 
			M3508_CHASSIS_MOTOR_REAL_INFO[6].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[6].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[6].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		case M3508_CHASSIS_MOTOR_ID_8:   //��⵽���ID
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[7].ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[7].RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[7].CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		default: break;
	}
}

//can1���͵���m3508�ĵ���            IDΪ1234���
//ת�ӽǶȷ�Χ��0~8191				ӳ�䵽0~8191
//ת��ת��RPM
//����¶ȵ�λ
void M3508_Send_Motor_Currents1(void)
{
	CanTxMsg tx_message;

	//���ÿ��ƶ�
	tx_message.IDE = CAN_Id_Standard;   //��׼֡
	tx_message.RTR = CAN_RTR_Data;      //��־У��֡��Ϊ����֡
	tx_message.DLC = 0x08;              //0x02��Ӧһ�����  0x08��Ӧ4�����
	
	//�����ٲöκ����ݶ�
	tx_message.StdId = M3508_CHASSIS_MOTOR_1234_ID;  // ����IDΪ 1 2 3 4�ĵ��
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


//can1���͵���m3508�ĵ���            IDΪ1234���
//ת�ӽǶȷ�Χ��0~8191				ӳ�䵽0~8191
//ת��ת��RPM
//����¶ȵ�λ
void M3508_Send_Motor_Currents2(void)
{
	CanTxMsg tx_message;

	//���ÿ��ƶ�
	tx_message.IDE = CAN_Id_Standard;   //��׼֡
	tx_message.RTR = CAN_RTR_Data;      //��־У��֡��Ϊ����֡
	tx_message.DLC = 0x08;              //0x02��Ӧһ�����  0x08��Ӧ4�����
	
	//�����ٲöκ����ݶ�
	tx_message.StdId = M3508_CHASSIS_MOTOR_5678_ID;  // ����IDΪ 1 2 3 4�ĵ��
    //��Ϣ�ľ�������
	tx_message.Data[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT >> 8);
	tx_message.Data[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT;
  
  tx_message.Data[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT >> 8);
	tx_message.Data[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT;
  
  tx_message.Data[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT >> 8);
	tx_message.Data[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT;
  
  tx_message.Data[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[7].TARGET_CURRENT >> 8);
	tx_message.Data[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[7].TARGET_CURRENT;
  
	CAN_Transmit(CAN1, &tx_message);  // ����ָ��
}


