#ifndef  __PID_H
#define  __PID_H



typedef struct PID_Date
{
	float error ;  
	float last_error ;
	float earlier_error ;
	float K_P ;
	float K_I ;
	float K_D ;
	float I_Separate ;   //积分分离
	float I_Limit ;     //积分限幅
	float Out_MAX ;    //电流限幅
	float Out_MIN ; 
	float Dead_Size;   //死区大小
  float Output ;
}PID_Date;


extern PID_Date M3508_PID[4];

float PID_Speed_Calculate(PID_Date * PID  ,float expect,float Encoder_Count);
float PID_Position_Calculate(PID_Date * PID,float expect,float Encoder_Count);
void PID_Parameter_Speed_Init(PID_Date *PID, float Pi, float Ki, float Di, float Out_MAX, float Dead_Size, float I_Separate, float I_Limit);
 void PID_Parameter_Limit(PID_Date *PID,float *parameter,int limit);
#endif



