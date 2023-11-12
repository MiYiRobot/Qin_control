#include "main.h"
#define locked_rotor_time 30   //����ʱ�䣬������ֵ��������ĵ����ת����ֵ��������ʱ�����ж�Ϊ����ס

uint8_t is_have_b_flag = 0;    //�Ƿ���ס��    0Ϊ����   1Ϊ����
uint8_t is_set_b_flag = 0;   //���ñ�־λ    0Ϊδ���䣬1Ϊ����
uint8_t is_shoot_b_flag = 0;   //�����־λ   0Ϊδ����   1Ϊ����
int p;

//if(PPM_Databuf[5]<1500)
//p= PID_Speed_Calculate(&M3508_PID[4],1300,M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE);
//M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[4],p,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);

void upper_control(void)
{
	if(is_have_b_flag == 0 )    //û��״̬�����ã�
	{
		if(test_rise_time_up(abs(M3508_CHASSIS_MOTOR_REAL_INFO[4].CURRENT),2000,locked_rotor_time))   //�������
		{
			M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],0,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);
            M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE = 0;
		}
		else
		{
			M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],-1500,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);
		}
		//�½�
//		p= PID_Speed_Calculate(&M3508_PID[7],0,M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE);
//		M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],p,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);//λ�û�
		
//		//�½�����͵㣬������ڣ�λ�õ���
//		if(test_rise_time_up(ABS(M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT),3000,locked_rotor_time))
//		{
//			M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE = 0;
//		}
		//Ħ����ת
		M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[5],2200 , M3508_CHASSIS_MOTOR_REAL_INFO[6].RPM);
		M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[6],-2200, M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM);  			
	}
	else if(is_have_b_flag==1)      //����
	{
        //̧��
		p= PID_Speed_Calculate(&M3508_PID[7],1500,M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE);
		M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],p,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);//λ�û�
		
		if(is_set_b_flag==0)    //����ťû����ť����
		{
            //ͣ�򣬶���
//		    M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT = 2000;
//			M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT = -2000;     
        M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[5],800 , M3508_CHASSIS_MOTOR_REAL_INFO[6].RPM);
		M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[6],-800, M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM); 
		}
		else if (is_set_b_flag == 1)
		{ 
		
			M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[5],0 , M3508_CHASSIS_MOTOR_REAL_INFO[6].RPM);
			M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[6],0, M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM);  
            vTaskDelay(100);
            
            if(  M3508_CHASSIS_MOTOR_REAL_INFO[6].CURRENT < 3000) is_have_b_flag = 0;
		}
		 if (is_shoot_b_flag == 1)
		{ 
		
			M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[5],-4000 , M3508_CHASSIS_MOTOR_REAL_INFO[6].RPM);
			M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[6],4000, M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM);  
            
            if(  M3508_CHASSIS_MOTOR_REAL_INFO[6].CURRENT < 3000) is_have_b_flag = 0;
		}
	}
	//�����ת����һ��ʱ�䣬��˵������
	if(test_rise_time(ABS(M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT),3000,locked_rotor_time)) is_have_b_flag = 1;
 
	//if(abs(M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT )> 9000)is_have_b_flag = 1;
	if(PPM_Databuf[5] > 1500)  {is_shoot_b_flag = 1;}//����
    	if(PPM_Databuf[5] < 1500)  {is_shoot_b_flag = 0;}//����
	if(PPM_Databuf[4] < 1500) is_set_b_flag = 0; //��ͣ
	if(PPM_Databuf[4] > 1500) is_set_b_flag = 1; //����
//    if(PPM_Databuf[6] < 1500) is_shoot_b_flag = 0;   //��ͣ
//    if(PPM_Databuf[6] > 1500) is_shoot_b_flag = 0;   //����
  	M3508_Send_Motor_Currents2();
    
}




 uint16_t time_up = 0;
//�������ʱ�䣬��������涨ʱ�䣬�򷵻�1
//time ������0.001ms
//time��boundary_time�ĵ�λδ֪....
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time)
{ 
  if(current<boundary_current) time_up=0;
  if(current>boundary_current) time_up++;
  if(time_up>=boundary_time)return 1;
  return 0; 
}

 uint16_t time = 0;
//�������ʱ�䣬��������涨ʱ�䣬�򷵻�1 ����1��֤�������ת
//time ������0.001ms
//time��boundary_time�ĵ�λδ֪....
uint8_t test_rise_time(int current,int boundary_current,int boundary_time)
{ 
  if( current<boundary_current) time=0;
  if(current>boundary_current) time++;
  if(time>=boundary_time)return 1;
  return 0; 
} 

