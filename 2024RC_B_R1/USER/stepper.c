//
// Created by Ray on 2023/11/25.
//

#include "stepper.h"

Screws LIFT;        //��ʱ��10
Screws RIGHT;       //��ʱ��11

void screw_init(void)
{
    LIFT.run_state = 0;
    LIFT.dir = 0;
    LIFT.target_step = 0;
    LIFT.step_count = 0;
    LIFT.dir_cal = 1;
    LIFT.angle = 0;
    LIFT.pos = 0;
    Step1_ENABLE;//������ʱֻ��һ���������

    RIGHT.run_state = 0;
    RIGHT.dir = 0;
    RIGHT.target_step = 0;
    RIGHT.step_count = 0;
    RIGHT.dir_cal = 1;
    RIGHT.angle = 0;
    RIGHT.pos = 0;
    Step2_ENABLE;//������ʱֻ��һ���������
}

/**
 * @brief ����������ƺ���
 * 
 * @param screws           ����ṹ�� LIFT ���� RIGHT
 * @param targe_distance    ֱ�������������� ����Ϊ����������Ϊ�½� ��λmm
 */
void screws_move(Screws* screws, float targe_distance)
{
    Max_Value_Limit(targe_distance, 130);//��ֵ�����ܳ���130
    screws->step_count= 0;

    if(screws == &LIFT)
    {
        if(targe_distance >= 0)
        {
            screws->dir = CW;
            screws->dir_cal = 1;
            Step1_CW;
        }
        else
        {
            targe_distance = -targe_distance;
            screws->dir = CCW;
            screws->dir_cal = -1;
            Step1_CCW;
        }
        screws->target_step = targe_distance / LPS;
        HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
    }
    else if(screws == &RIGHT)
    {
        if(targe_distance >= 0)
        {
            screws->dir = CW;
            screws->dir_cal = 1;
            Step2_CW;
        }
        else
        {
            targe_distance = -targe_distance;
            screws->dir = CCW;
            screws->dir_cal = -1;
            Step2_CCW;
        }
        screws->target_step = targe_distance / LPS;
        HAL_TIM_PWM_Start_IT(&htim11, TIM_CHANNEL_1);
    }
}

/**
 * @brief ˿��λ�ÿ��ƺ���
 * 
 * @param step_id ����˿��
 * @param targe_pos ����λ�� ���������������½� ��λmm ��0~100�����ǵ�������������Դ���һ�١�
 */
void screws_pos(Screws* screws, float targe_pos)
{
    if(targe_pos > 130) targe_pos = 130;
    if(targe_pos < -5) targe_pos = -5;
    screws_move(screws, targe_pos - screws->pos);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim10.Instance)	// ȷ���Ƿ�Ϊ������������жϵĻص�
	{
        LIFT.run_state = 1;
		LIFT.step_count ++;
        LIFT.angle += LIFT.dir_cal * LIFT.step_count * 360 / SPR;
        LIFT.pos += LIFT.dir_cal * LIFT.step_count * LPS;
		if(LIFT.step_count >= LIFT.target_step)
		{
            LIFT.run_state = 0;
			LIFT.step_count = 0;
			HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);		// ֹͣ���PWM
		}
	}
    if(htim->Instance == htim11.Instance)	// ȷ���Ƿ�Ϊ������������жϵĻص�
	{
        RIGHT.run_state = 1;
		RIGHT.step_count ++;
        RIGHT.angle += RIGHT.dir_cal * RIGHT.step_count * 360 / SPR;
        RIGHT.pos += RIGHT.dir_cal * RIGHT.step_count * LPS;
		if(RIGHT.step_count >= RIGHT.target_step)
		{
            RIGHT.run_state = 0;
			RIGHT.step_count = 0;
			HAL_TIM_PWM_Stop_IT(&htim11, TIM_CHANNEL_1);		// ֹͣ���PWM
		}
	}
}
