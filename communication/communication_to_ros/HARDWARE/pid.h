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
	float I_Separate ;   //���ַ���
	float I_Limit ;     //�����޷�
	float Out_MAX ;    //�����޷�
	float Out_MIN ; 
	float Dead_Size;   //������С
  float Output ;
}PID_Date;


extern PID_Date M3508_PID[4];

float PID_Speed_Calculate(PID_Date * PID  ,float expect,float Encoder_Count);
float PID_Position_Calculate(PID_Date * PID,float expect,float Encoder_Count);
void PID_Parameter_Speed_Init(PID_Date *PID, float Pi, float Ki, float Di, float Out_MAX, float Dead_Size, float I_Separate, float I_Limit);
 void PID_Parameter_Limit(PID_Date *PID,float *parameter,int limit);
#endif



