#include "main.h"
/*
PID  (积分分离，死区，限幅)
*/
 
//pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]

PID_Date M3508_PID[4], YAW_PID, TRACK_PID, M2006_PID[2] ,Turn_PID[2];


//速度环PID，参数：PID, 期待速度，当前转速
float PID_Speed_Calculate(PID_Date *PID, float expect, float Encoder_Count)//增量型pid
{
	float integral, proportion, differential;
	PID->error = expect - Encoder_Count; //计算偏差

	if (PID->I_Separate == 0)
		integral = PID->K_I * PID->error;   //计算积分
	else
	{
		//判断是否积分分离
		if (ABS(PID->error) < PID->I_Separate)
			integral = PID->K_I * PID->error;
		else
			integral = 0;
	}//当刚开始时或者收到干扰时，误差偏大，去除这种情况对系统的影响

  PID_Parameter_Limit( PID,&integral,PID->I_Limit);//积分限幅
  
	//计算各值
	proportion = PID->K_P * (PID->error - PID->last_error);//比例P
	differential = PID->K_D * (PID->error - 2 * PID->last_error + PID->earlier_error);//差分D

	PID->Output += integral + proportion + differential; //计算输出

	PID->earlier_error = PID->last_error;//刷新pid数据
	PID->last_error = PID->error;

	if (PID->Output > PID->Out_MAX)//对输出限幅
		PID->Output = PID->Out_MAX;
	if (PID->Output < PID->Out_MIN)
		PID->Output = PID->Out_MIN;


	return PID->Output;
}

//位置环PID  参数： PID，期待值，当前转速
float PID_Position_Calculate(PID_Date *PID, float expect, float Encoder_Count)
{
	static float integral = 0;
	float differential = 0;
	PID->error = expect - Encoder_Count;		 //计算误差
	differential = PID->error - PID->last_error; //微分

	if (PID->I_Separate == 0)
		integral += PID->error;
	else
	{
		//判断是否积分分离
		if (ABS(PID->error) < PID->I_Separate)
			integral += PID->error;
		else
			integral = 0;
	}

	//积分限幅
	if (integral > PID->I_Limit)
		integral = PID->I_Limit;
	if (integral < -PID->I_Limit)
		integral = -PID->I_Limit;

	//死区
	if (ABS(PID->error) > PID->Dead_Size)
	{
		PID->Output = PID->K_P * PID->error + PID->K_I * integral + PID->K_D * differential;
	}
	else
		PID->Output = 0;

	PID->last_error = PID->error;

	if (PID->Output > PID->Out_MAX)
		PID->Output = PID->Out_MAX;
	if (PID->Output < PID->Out_MIN)
		PID->Output = PID->Out_MIN;

	return PID->Output;
}

 void PID_Parameter_Limit(PID_Date *PID,float *parameter,int limit){//积分限幅函数
   
	//积分限幅
	if (*parameter > limit)
		*parameter = limit;
	if (*parameter < -limit)
		*parameter = -limit;
 }
 

 //初始化PID，输入值为PID的结构体，参数分别为：P,I,D,电流限幅，死区大小，积分分离，积分限幅
void PID_Parameter_Speed_Init(PID_Date *PID, float Pi, float Ki, float Di, float Out_MAX, float Dead_Size, float I_Separate, float I_Limit)
{
	PID->K_P = Pi;
	PID->K_I = Ki;
	PID->K_D = Di;
	PID->Dead_Size = Dead_Size;
	PID->Out_MAX = Out_MAX;
	PID->Out_MIN = -Out_MAX;
	PID->I_Limit = I_Limit;
	PID->I_Separate = I_Separate;

	PID->error = PID->last_error = PID->earlier_error = 0;
	PID->Output = 0;
}
