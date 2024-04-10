//
// Created by Ray on 2023/11/24.
//

#include "haoying_u8.h"

// HAOYING_DRIVE U8_MOTOR[2];

/**
 * @brief 初始化（没事瞎几把封装的，之后用vesc，就用不上这个了）
 * 
 * @param u8motor 
 * @param htim 
 * @param Channel 
 */
void HaoYing_U8_Init(void)
{
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);     //开启PWM通道
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);     //开启PWM通道
    
    // u8_ctrl(0, 0);                            //然后踩刹车
    // u8_ctrl(1, 0);                            //然后踩刹车
    HAL_Delay(2000);
}

/**
 * @brief u8控制函数
 * 
 * @param pulse_value 油门值（0~100)
 */
void u8_ctrl(int16_t pulse_value)
{
    TIM13->CCR1 = 1000 + pulse_value * 10;
    TIM14->CCR1 = 1000 + pulse_value * 10;
}
