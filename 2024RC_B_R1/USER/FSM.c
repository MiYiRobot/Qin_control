//
// Created by Ray on 2023/11/24.
//

#include "FSM.h"

State robot_state = init;
State chassis_state = high_speed;
State seed_state = seed_disable;
State shoot_state = shoot_init;

SHOOT_SPEED shoot_speed = far_shoot;

void robot_fsm(void)
{
    switch (robot_state)
    {
    case init:          //��ʼ��
        chassis_state = high_speed;
        seed_state = seed_disable;
        shoot_state = shoot_init;
        chassis_init();
        robot_state = calibration;
        break;
    case calibration:     //У׼
        ROBOT_CHASSI.Vx_MAX = 0.0f;
        ROBOT_CHASSI.Vy_MAX = 0.0f;
        ROBOT_CHASSI.Vw_MAX = 0.0f;
        if (SWA != 0 && SWB != 0 && SWC != 0 && SWD != 0)   //ȷ�Ϻ�ģ��ʼ���ɹ�
        {
            robot_state = auto_ctrl;
//            SWA_judge();
//            SWB_judge();
//            SWC_judge();
//            SWD_judge();
        }
        break;
    case auto_ctrl:         //�Զ�����
        if (SWA == 0 && SWB == 0 && SWC == 0 && SWD == 0){
            robot_state = calibration;
        }
        else
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
            SWD_judge();
        }
        break;
    case seed_ctrl:         //������� 

        if (SWA == 0 && SWB == 0 && SWC == 0 && SWD == 0){
            robot_state = calibration;
        }
        else
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
            SWD_judge();
        }
        break;
    case shoot_ctrl:
        if (SWA == 0 && SWB == 0 && SWC == 0 && SWD == 0){
            robot_state = calibration;
        }
        else
        {
            SWA_judge();
            SWB_judge();
            SWC_judge();
            SWD_judge();
        }
        break;
    }
}

void SWA_judge(void)
{
    if (SWA < 1500)    //�ϲ�Ϊ���̸�����ʻ
    {
        chassis_state = high_speed;
    }
    else if (SWA > 1500)    //�²�Ϊ���̵�����ʻ
    {
        chassis_state = low_speed;
    }
}

void SWB_judge(void)
{
    if (SWB < 1200)     //SWB�ϲ�Ϊ�Զ�����
    {
        robot_state = auto_ctrl;
    }
    else if (1300 < SWB && SWB < 1700)  //SWB�м䵵λΪ�������
    {
        robot_state = seed_ctrl;
    }
    else if (SWB > 1800)            //SWB�²�Ϊ����ṹ����
    {
        robot_state = shoot_ctrl;
    }
}

void SWC_judge(void)
{
    if (SWC < 1200)     //SWC�ϲ�
    {
        if (robot_state == auto_ctrl)   //�Զ����ƣ������
        {
            //
        }
        else if (robot_state == seed_ctrl)  
        {
            seed_state = seed_init;     //�����ʼ���������ͣ��
        }
        else if (robot_state == shoot_ctrl)
        {
            shoot_state = shoot_init;   //����ṹ��ʼ��
        }
    }
    else if (1300 < SWC && SWC < 1700)  //SWC�м䵵
    {
        if (robot_state == auto_ctrl)
        {
            //
        }
        else if (robot_state == seed_ctrl)
        {
            seed_state = peek_deposit;      //ȡ��
        }
        else if (robot_state == shoot_ctrl)
        {
            shoot_state = load;         //װ��
        }
    }
    else if (SWC > 1800)        //SWC�²�
    {
        if (robot_state == auto_ctrl)
        {
            //
        }
        else if (robot_state == seed_ctrl)
        {
            seed_state = take_put;          //����
        }
        else if (robot_state == shoot_ctrl)
        {
            shoot_state = shooting;       //����  
        }
    }
}

void SWD_judge(void)
{
    if (SWD < 1500)
    {
        
        if (robot_state == auto_ctrl)   //�Զ����ƣ������
        {
            //
        }
        else if (robot_state == seed_ctrl)  //�ϲ�Ϊ6020�ƶ�������λ�ã�����צ��׼��ʱ�ϲ�
        {
            Seed_Flag.seed_aim_flag = 0;      //�����׼���־λ��0
        }   
        else if (robot_state == shoot_ctrl)  //
        {
            shoot_speed = far_shoot;   //�ϲ�ΪԶ��
        }

    }
    else if (SWD > 1500)
    {
                if (robot_state == auto_ctrl)   //�Զ����ƣ������
        {
            //
        }
        else if (robot_state == seed_ctrl)  //�²�Ϊ6020�ƶ����������λ��
        {
            Seed_Flag.seed_aim_flag = 1;      //�����׼��־λ��1
        }
        else if (robot_state == shoot_ctrl)  //
        {
            shoot_speed = near_shoot;   //�²�Ϊ����
        }
    }
}
