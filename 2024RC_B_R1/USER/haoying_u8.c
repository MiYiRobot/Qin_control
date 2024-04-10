//
// Created by Ray on 2023/11/24.
//

#include "haoying_u8.h"

// HAOYING_DRIVE U8_MOTOR[2];

/**
 * @brief ��ʼ����û��Ϲ���ѷ�װ�ģ�֮����vesc�����ò�������ˣ�
 * 
 * @param u8motor 
 * @param htim 
 * @param Channel 
 */
void HaoYing_U8_Init(void)
{
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);     //����PWMͨ��
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);     //����PWMͨ��
    
    // u8_ctrl(0, 0);                            //Ȼ���ɲ��
    // u8_ctrl(1, 0);                            //Ȼ���ɲ��
    HAL_Delay(2000);
}

/**
 * @brief u8���ƺ���
 * 
 * @param pulse_value ����ֵ��0~100)
 */
void u8_ctrl(int16_t pulse_value)
{
    TIM13->CCR1 = 1000 + pulse_value * 10;
    TIM14->CCR1 = 1000 + pulse_value * 10;
}
