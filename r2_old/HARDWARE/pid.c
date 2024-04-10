#include "main.h"

 int integral =0;
 int error =0;
 int D_error =0;
  int LAST_error =0;
  
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
  
/*
PID  (积分分离，死区，限幅)
*/
 
//pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]

PID_Date M3508_PID[8];
//位置式PID  
//输入： PID   期望值    当前值
float PID_Speed_Calculate(PID_Date *PID, float expect, float Encoder_Count)
{    
    PID->D_error = (expect-Encoder_Count) - PID->error;
    PID->error = expect - Encoder_Count;
    PID->integral += PID->error;
    //积分限幅   
    if (PID->integral > PID->I_Limit)    PID->integral = PID->I_Limit;
	if (PID->integral < -(PID->I_Limit)) PID->integral = -(PID->I_Limit);
    
   if(ABS(PID->error) > PID->Dead_Size) //误差大于死区  
   {      
    PID->Output = PID->K_P * PID->error + PID->K_I * PID->integral + PID->K_D * PID->D_error;
   } 
   else
   {
       PID->Output = 0;
   }
   //输出限幅
   if(PID->Output > PID->Out_MAX)        PID->Output = PID->Out_MAX;
   if(PID->Output < -(PID->Out_MAX))     PID->Output = -(PID->Out_MAX);
  return PID->Output;
}



/**
  * 函数功能: 梯形速度规划
  * 输入参数: M3508_REAL_INFO结构体
  * 返 回 值: 无
  * 说    明: 无
 */
void VelocityPlanningMODE(M3508_REAL_INFO *M3508_MOTOR)	
{
	//static int cnt;//记时用
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	// 如果所配数据有误，则不执行速度规划		
	if((M3508_MOTOR->velocity_planning.Rac > 1) || (M3508_MOTOR->velocity_planning.Rac < 0) ||		//检查加速路程的比例
		 (M3508_MOTOR->velocity_planning.Rde > 1) || (M3508_MOTOR->velocity_planning.Rde < 0) ||	//检查减速路程的比例
		 (M3508_MOTOR->velocity_planning.Vmax < M3508_MOTOR->velocity_planning.Vstart) )			//最大的速度<开始的速度 
	{
		M3508_MOTOR->TARGET_RPM = 0;  
		return;
	}
	// 匀速模式
	if(M3508_MOTOR->velocity_planning.Pstart == M3508_MOTOR->velocity_planning.Pend)	//开始位置=结束位置
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vmax;	//开始的速度*最大的速度
		return;
	}
	
	// 计算一些变量
	Ssu = ABS(M3508_MOTOR->velocity_planning.Pend - M3508_MOTOR->velocity_planning.Pstart); 	//总路程   
	Sac = Ssu * M3508_MOTOR->velocity_planning.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * M3508_MOTOR->velocity_planning.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;		//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (M3508_MOTOR->velocity_planning.Vmax * M3508_MOTOR->velocity_planning.Vmax - M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend -   M3508_MOTOR->velocity_planning.Vmax *   M3508_MOTOR->velocity_planning.Vmax) / (2.0f * Sde);	  
	
	// 过滤异常情况
	if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pend)) ||
		      ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pend)))
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pstart);      //开始位置
		
		// 规划RPM
    if (S==0)S++;
		if     (S < Sac)     {  M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart);}               // 加速阶段
		else if(S < (Sac+Sco)) {M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vmax;}                                                        // 匀速阶段
		else                   {M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend - 2.0f * Ade * ABS(Ssu - S)); } // 减速阶段
	}
	 
	// 分配合适的正负号
	if(M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
	//判断是否完成
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) < 3)
		M3508_MOTOR->velocity_planning.flag = 1;//设置标志位			
						
   if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) > 3)
			M3508_MOTOR->velocity_planning.flag = 0;
}


 


