//
// Created by Ray on 2023/11/26.
//

#include "config.h"

/**
 * @brief ���̺�����6020�ĵ�����ͳ�ʼ��
 * 
 */
void can1_config(void)
{
    can1motorRealInfo[0].Motor_Type = M_3508;//�����Һ���
    can1motorRealInfo[1].Motor_Type = M_3508;//������ǰ��
    can1motorRealInfo[2].Motor_Type = M_3508;//������ǰ��
    can1motorRealInfo[3].Motor_Type = M_3508;//���������
    can1motorRealInfo[4].Motor_Type = M_6020;//��6020ת�����      ID��� 205
    can1motorRealInfo[5].Motor_Type = M_3508;//������̧��     ID��� 206
    can1motorRealInfo[6].Motor_Type = M_3508;//����     ID��� 207
    //6020λ�û���Ȧģʽ
    can1motorRealInfo[4].once_flag = 1;   
    //����ֻ��Ҫ�ٶȻ�
    pid_param_init(&can1MOTOR_PID_RPM[0], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[1], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[2], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);
    pid_param_init(&can1MOTOR_PID_RPM[3], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.1f, 0.2f);

    //��6020��λ�û����ٶȻ�
    pid_param_init(&can1MOTOR_PID_RPM[4], PID_Position, 8192, 1024, 0, 0.5, 16384, 0.5f, 0.0f, 0.2f);
    pid_param_init(&can1MOTOR_PID_POS[4], PID_Position, 8192, 800, 0, 0.1f, 16384, 10, 0.0, 0.3); 
    //����̧������
    pid_param_init(&can1MOTOR_PID_RPM[5], PID_Incremental, 10000, 1024, 8192, -0.5,8192, 12.0f, 0.4f, 0.0f);
    pid_param_init(&can1MOTOR_PID_POS[5], PID_Position, 1024, 800, 0, 0.1f, 16384, 8, 0, 0.2);
    //���ֵ�λ�û����ٶȻ�
    pid_param_init(&can1MOTOR_PID_RPM[6], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can1MOTOR_PID_POS[6], PID_Position, 1024, 800, 0, 0.1f, 16384, 12, 0, 0.3);
}

/**
 * @brief can2��ʼ��
 * 
 */
void can2_config(void)
{
    can2motorRealInfo[0].Motor_Type = M_3508;//��������Ĵ�
    can2motorRealInfo[1].Motor_Type = M_3508;//��������Ĵ�
    can2motorRealInfo[2].Motor_Type = M_3508;//��צת�����
    can2motorRealInfo[3].Motor_Type = M_3508;//������̨
    can2motorRealInfo[4].Motor_Type = M_6020;//6020ת�����   ID��� 205
    can2motorRealInfo[5].Motor_Type = M_3508;//������̧��
    can2motorRealInfo[6].Motor_Type = M_3508;//�Ұ���

    can2motorRealInfo[4].once_flag = 1;     //6020λ�û���Ȧģʽ
    //�Ĵ��ٶȻ�PID
    pid_param_init(&can2MOTOR_PID_RPM[0], PID_Position, 10000, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can2MOTOR_PID_RPM[1], PID_Position, 10000, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    //��צת�����λ�û����ٶȻ�
    pid_param_init(&can2MOTOR_PID_RPM[2], PID_Incremental, 10000, 1024, 8192, -0.5,8192, 12.0f, 0.4f, 0.0f);
    pid_param_init(&can2MOTOR_PID_POS[2], PID_Position, 1024, 800, 0, 0.1f, 16384, 3, 0, 0);
    //��̨λ�û����ٶȻ�
    pid_param_init(&can2MOTOR_PID_RPM[3], PID_Incremental, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.2f, 0.2f);
    pid_param_init(&can2MOTOR_PID_POS[3], PID_Position, 2048, 800, 0, 0.1f, 16384, 15, 0, 0.3);
    //��6020��λ�û����ٶȻ�
    pid_param_init(&can2MOTOR_PID_RPM[4], PID_Position, 8192, 1024, 0, 0.5, 16384, 1.0f, 0.0f, 0.1f);
    pid_param_init(&can2MOTOR_PID_POS[4], PID_Position, 8192, 800, 0, 0.1f, 16384, 10, 0.0, 0.3);
    //����̧������
    pid_param_init(&can2MOTOR_PID_RPM[5], PID_Incremental, 10000, 1024, 8192, -0.5,8192, 12.0f, 0.4f, 0.0f);
    pid_param_init(&can2MOTOR_PID_POS[5], PID_Position, 1024, 800, 0, 0.1f, 16384, 8, 0, 0.2);
    //�Ұ��ֵ�λ�û����ٶȻ�
    pid_param_init(&can2MOTOR_PID_RPM[6], PID_Position, 8192, 1024, 0, 0.5, 16384, 12.0f, 0.0f, 0.2f);
    pid_param_init(&can2MOTOR_PID_POS[6], PID_Position, 1024, 800, 0, 0.1f, 16384, 12, 0, 0.3);

}


