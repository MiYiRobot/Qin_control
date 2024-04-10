//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_CHASSIS_H
#define INC_2024RC_B_R1_CHASSIS_H

#include "main.h"

#define COS60 0.500000f
#define COS30 0.866025f
#define COS45 0.707106f
#define PI              3.1415926f                    	//PI��ֵ
#define WHEEL_R              0.152f/2                 	//���Ӱ뾶  m
#define RM_transition_MS      (PI*WHEEL_R)/570.0f       //ת�����ٶȵ�ת�� rpm 2 m  //(2 * PI * WHEEL_R)/1.41f
#define MS_transition_RM      570.0f/(PI*WHEEL_R)       //�ٶ���ת�ٵ�ת�� m 2 rpm//    1.41f/(2 * PI * WHEEL_R)
#define CHASSIS_R 0.40f                               	//���̰뾶 m

#define Mecanum_Rx 0.5
#define Mecanum_Ry 0.5

typedef struct ROBOT_CHASSIS
{
	float Vx;//�����ٶ�
    float Vy;
    float Vw;

    float world_x;//����action����������������
    float world_y;
    float world_w;

	float plan_x;//����·���滮���ص��ٶ�����ֵ
    float plan_y;
    float plan_w;

	float Vy_MAX;//����ٶ�����
	float Vx_MAX;
	float Vw_MAX;

	float Motor_Target_RPM[4];           //4�����ӵ�Ŀ��ת��

	int8_t Path_planning;      			//·���滮��־λ
	int8_t World_Move_Flag;				//��������ϵ���ñ�־λ����ʱδ�ã�
} ROBOT_CHASSIS;

extern ROBOT_CHASSIS ROBOT_CHASSI;

void chassis_init(void);
void Robot_Wheels_RPM_calculate(void);
void chassis_stop(void);
void Free_Control(void);

#endif //INC_2024RC_B_R1_CHASSIS_H
