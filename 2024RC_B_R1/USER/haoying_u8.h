//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_HAOYING_U8_H
#define INC_2024RC_B_R1_HAOYING_U8_H

#include "main.h"

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

void HaoYing_U8_Init(void);
void u8_ctrl(int16_t pulse_value);

#endif //INC_2024RC_B_R1_HAOYING_U8_H
