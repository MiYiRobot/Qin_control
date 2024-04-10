//
// Created by Ray on 2023/11/25.
//

#ifndef INC_2024RC_B_R1_STEPPER_H
#define INC_2024RC_B_R1_STEPPER_H

#include "main.h"

#define CW 0    //顺时针
#define CCW 1   //逆时针

//一些数据的计算
#define SPR         3200                //步进电机单圈步数
#define MICRO_STEP  16                  //步进电机驱动器细分数

#define R2L         12                  //步进电机转一圈，丝杆移动12mm
#define LPD         R2L / 360           //步进电机每转一度，丝杆移动距离
#define LPS         R2L / SPR           //每脉冲 丝杆运行路程
#define SSpd        60                  //丝杆速度 mm每秒
#define MSpd        SSpd / R2L          //电机速度 圈每秒
#define FPWM        (MSpd * SPR)        //满足速度需要的pwm频率 16000  计数5250 2625

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

typedef struct Screws
{
    uint8_t  run_state;    // 电机旋转状态 0 表示闲置 1 表示正在运行
    uint8_t  dir;          // 电机旋转方向 CW CCW
    int32_t  target_step;   // 期望脉冲数      根据电机细分数计算得到
    int32_t  step_count;    // 计步 中断计数值

    int dir_cal;            // 正负数计算 顺时针置1 逆时针置-1
    float angle;            // 角度
    float pos;              //当前丝杆位置，0为最低，100为最高
}Screws;

#define Step1_ENABLE    HAL_GPIO_WritePin(step1ena_GPIO_Port, step1ena_Pin, GPIO_PIN_RESET);
#define Step1_DISABLE   HAL_GPIO_WritePin(step1ena_GPIO_Port, step1ena_Pin, GPIO_PIN_SET);

#define Step2_ENABLE    HAL_GPIO_WritePin(step2ena_GPIO_Port, step2ena_Pin, GPIO_PIN_RESET);
#define Step2_DISABLE   HAL_GPIO_WritePin(step2ena_GPIO_Port, step2ena_Pin, GPIO_PIN_SET);

#define Step1_CW    HAL_GPIO_WritePin(step1dir_GPIO_Port, step1dir_Pin, GPIO_PIN_RESET);
#define Step1_CCW   HAL_GPIO_WritePin(step1dir_GPIO_Port, step1dir_Pin, GPIO_PIN_SET);

#define Step2_CW    HAL_GPIO_WritePin(step2dir_GPIO_Port, step2dir_Pin, GPIO_PIN_RESET);
#define Step2_CCW   HAL_GPIO_WritePin(step2dir_GPIO_Port, step2dir_Pin, GPIO_PIN_SET);

extern Screws LIFT;        //定时器10
extern Screws RIGHT;       //定时器11

void screw_init(void);

void screws_move(Screws* screws, float targe_distance);

void screws_pos(Screws* screws, float targe_pos);

#endif //INC_2024RC_B_R1_STEPPER_H
