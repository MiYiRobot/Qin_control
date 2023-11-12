#ifndef  __PID_H
#define  __PID_H



typedef struct PID_Date
{
	float error ;       //ƫ�� 
    float D_error;      //΢��ƫ��
	float last_error ;  //�ϴ�ƫ��
	float earlier_error ; //����ƫ��
	float K_P ;           
	float K_I ;
	float K_D ;
	float I_Separate ;   //���ַ���
	float I_Limit ;     //�����޷�
	float Out_MAX ;    //�����޷�
	float Out_MIN ; 
	float Dead_Size;   //������С
    float integral;    //����
  float Output ;
}PID_Date;


extern PID_Date M3508_PID[8];

float PID_Speed_Calculate(PID_Date * PID  ,float expect,float Encoder_Count);
float PID_Position_Calculate(PID_Date * PID,float expect,float Encoder_Count);
void PID_Parameter_Speed_Init(PID_Date *PID, float Pi, float Ki, float Di, float Out_MAX, float Dead_Size, float I_Separate, float I_Limit);
#endif



