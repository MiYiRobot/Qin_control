#ifndef __MOVE__H
#define __MOVE__H

#include "stm32f4xx_can.h"
#define COS60 0.500000f
#define COS30 0.866025f
#define COS45 0.707106f

#define RM_transition_MS (PI * WHEEL_R) / 570.0f      //ת�����ٶȵ�ת��  
#define MS_transition_RM 570.0f / (PI * WHEEL_R) //�ٶ���ת�ٵ�ת��
#define WHEEL_R 0.152f / 2

// M3508������
#define M3508_CHASSIS_MOTOR_1234_ID 0x200
#define M3508_CHASSIS_MOTOR_ID_1 0x201
#define M3508_CHASSIS_MOTOR_ID_2 0x202
#define M3508_CHASSIS_MOTOR_ID_3 0x203
#define M3508_CHASSIS_MOTOR_ID_4 0x204
#define M3508_CHASSIS_MOTOR_5678_ID 0x1FF
#define M3508_CHASSIS_MOTOR_ID_5 0x205
#define M3508_CHASSIS_MOTOR_ID_6 0x206
#define M3508_CHASSIS_MOTOR_ID_7 0x207
#define M3508_CHASSIS_MOTOR_ID_8 0x208

//�ٶȹ滮�ṹ��
typedef struct VELOCITY_PLANNING{
float Distance;
	float Pstart;        //��ʼλ��
	float Pend;          //����λ��
	float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
	float Vmax;          //�����ٶ�
	float Vend;          //ĩβ���ٶ�
	float Rac;           //����·�̵ı���
	float Rde;           //����·�̵ı���
	int flag;            //��ɱ�־λ�����ͣ������ʱ����1
}VELOCITY_PLANNING;

//�ٶ����ֵ
#define Vx_max 7000
#define Vy_max 7000

// M3508���ص������ʵ��Ϣ
typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   		    //�����Ƕ�			
	int16_t  RPM;				//ʵ��ת��ת��				
	int16_t  CURRENT;           //����
	int16_t  TARGET_CURRENT;   //Ŀ�����
    int16_t  TARGET_RPM;       //Ŀ��ת��
    int16_t TARGET_POS;        //Ŀ�����
    int     Velflag;           //�ٶȱ�־
    //ת�ؽṹ��
    VELOCITY_PLANNING velocity_planning;
    
	float  REAL_ANGLE;         //��ʵ�Ƕ�    
    uint8_t FIRST_ANGLE_INTEGRAL_FLAG;   //�״λ��ֱ�־
    uint16_t LAST_ANGLE;       //�ϴνǶ�
    int16_t filter_RPM;        //ת���˲�
}M3508_REAL_INFO;

extern M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[8]; 



void Speed_Chassis_Calculate(void);
void M3508_Send_Motor_Currents1(void);
void M3508_Send_Motor_Currents2(void);
void m3508_update_m3508_info(CanRxMsg *msg);

#endif



