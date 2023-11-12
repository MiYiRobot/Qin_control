#include "main.h"
#define locked_rotor_time 30   //锁球时间，若电流值大于所测的电机堵转电流值，超过此时间则判定为球被锁住

uint8_t is_have_b_flag = 0;    //是否吸住球    0为无球   1为有球
uint8_t is_set_b_flag = 0;   //放置标志位    0为未发射，1为发射
uint8_t is_shoot_b_flag = 0;   //发射标志位   0为未发射   1为发射
int p;

//if(PPM_Databuf[5]<1500)
//p= PID_Speed_Calculate(&M3508_PID[4],1300,M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE);
//M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[4],p,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);

void upper_control(void)
{
	if(is_have_b_flag == 0 )    //没球状态（重置）
	{
		if(test_rise_time_up(abs(M3508_CHASSIS_MOTOR_REAL_INFO[4].CURRENT),2000,locked_rotor_time))   //在最底下
		{
			M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],0,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);
            M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE = 0;
		}
		else
		{
			M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],-1500,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);
		}
		//下降
//		p= PID_Speed_Calculate(&M3508_PID[7],0,M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE);
//		M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],p,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);//位置环
		
//		//下降到最低点，电机卡在，位置调零
//		if(test_rise_time_up(ABS(M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT),3000,locked_rotor_time))
//		{
//			M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE = 0;
//		}
		//摩擦轮转
		M3508_CHASSIS_MOTOR_REAL_INFO[6].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[5],2200 , M3508_CHASSIS_MOTOR_REAL_INFO[6].RPM);
		M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT = PID_Speed_Calculate(&M3508_PID[6],-2200, M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM);  			
	}
	else if(is_have_b_flag==1)      //有球
	{
        //抬升
		p= PID_Speed_Calculate(&M3508_PID[7],1500,M3508_CHASSIS_MOTOR_REAL_INFO[4].REAL_ANGLE);
		M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT=PID_Speed_Calculate(&M3508_PID[4],p,M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM);//位置环
		
		if(is_set_b_flag==0)    //放球按钮没按按钮按下
		{
            //停球，堵球
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
	//如果堵转超过一定时间，则说明有球
	if(test_rise_time(ABS(M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT),3000,locked_rotor_time)) is_have_b_flag = 1;
 
	//if(abs(M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT )> 9000)is_have_b_flag = 1;
	if(PPM_Databuf[5] > 1500)  {is_shoot_b_flag = 1;}//射球
    	if(PPM_Databuf[5] < 1500)  {is_shoot_b_flag = 0;}//射球
	if(PPM_Databuf[4] < 1500) is_set_b_flag = 0; //暂停
	if(PPM_Databuf[4] > 1500) is_set_b_flag = 1; //放球
//    if(PPM_Databuf[6] < 1500) is_shoot_b_flag = 0;   //暂停
//    if(PPM_Databuf[6] > 1500) is_shoot_b_flag = 0;   //射球
  	M3508_Send_Motor_Currents2();
    
}




 uint16_t time_up = 0;
//检测上升时间，如果超过规定时间，则返回1
//time 》》》0.001ms
//time和boundary_time的单位未知....
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time)
{ 
  if(current<boundary_current) time_up=0;
  if(current>boundary_current) time_up++;
  if(time_up>=boundary_time)return 1;
  return 0; 
}

 uint16_t time = 0;
//检测上升时间，如果超过规定时间，则返回1 返回1则证明电机堵转
//time 》》》0.001ms
//time和boundary_time的单位未知....
uint8_t test_rise_time(int current,int boundary_current,int boundary_time)
{ 
  if( current<boundary_current) time=0;
  if(current>boundary_current) time++;
  if(time>=boundary_time)return 1;
  return 0; 
} 

