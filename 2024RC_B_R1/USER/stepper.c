//
// Created by Ray on 2023/11/25.
//

#include "stepper.h"

Screws LIFT;        //定时器10
Screws RIGHT;       //定时器11

void screw_init(void)
{
    LIFT.run_state = 0;
    LIFT.dir = 0;
    LIFT.target_step = 0;
    LIFT.step_count = 0;
    LIFT.dir_cal = 1;
    LIFT.angle = 0;
    LIFT.pos = 0;
    Step1_ENABLE;//这里暂时只有一个步进电机

    RIGHT.run_state = 0;
    RIGHT.dir = 0;
    RIGHT.target_step = 0;
    RIGHT.step_count = 0;
    RIGHT.dir_cal = 1;
    RIGHT.angle = 0;
    RIGHT.pos = 0;
    Step2_ENABLE;//这里暂时只有一个步进电机
}

/**
 * @brief 步进电机控制函数
 * 
 * @param screws           电机结构体 LIFT 或者 RIGHT
 * @param targe_distance    直接设置期望距离 正数为上升，负数为下降 单位mm
 */
void screws_move(Screws* screws, float targe_distance)
{
    Max_Value_Limit(targe_distance, 130);//限值，不能超过130
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
 * @brief 丝杆位置控制函数
 * 
 * @param step_id 左右丝杆
 * @param targe_pos 期望位置 正数上升，负数下降 单位mm （0~100）考虑到丢步情况，可以大于一百。
 */
void screws_pos(Screws* screws, float targe_pos)
{
    if(targe_pos > 130) targe_pos = 130;
    if(targe_pos < -5) targe_pos = -5;
    screws_move(screws, targe_pos - screws->pos);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim10.Instance)	// 确认是否为步进电机脉冲中断的回调
	{
        LIFT.run_state = 1;
		LIFT.step_count ++;
        LIFT.angle += LIFT.dir_cal * LIFT.step_count * 360 / SPR;
        LIFT.pos += LIFT.dir_cal * LIFT.step_count * LPS;
		if(LIFT.step_count >= LIFT.target_step)
		{
            LIFT.run_state = 0;
			LIFT.step_count = 0;
			HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);		// 停止输出PWM
		}
	}
    if(htim->Instance == htim11.Instance)	// 确认是否为步进电机脉冲中断的回调
	{
        RIGHT.run_state = 1;
		RIGHT.step_count ++;
        RIGHT.angle += RIGHT.dir_cal * RIGHT.step_count * 360 / SPR;
        RIGHT.pos += RIGHT.dir_cal * RIGHT.step_count * LPS;
		if(RIGHT.step_count >= RIGHT.target_step)
		{
            RIGHT.run_state = 0;
			RIGHT.step_count = 0;
			HAL_TIM_PWM_Stop_IT(&htim11, TIM_CHANNEL_1);		// 停止输出PWM
		}
	}
}
