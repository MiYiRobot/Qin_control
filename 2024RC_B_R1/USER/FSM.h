//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_FSM_H
#define INC_2024RC_B_R1_FSM_H

#include "main.h"

typedef enum    //发射速度
{
    far_shoot,  //远射
    near_shoot  //近射
}SHOOT_SPEED;

extern SHOOT_SPEED shoot_speed;

typedef enum {
    init,               //整车初始化
    calibration,        //校准
    auto_ctrl,          //自动控制
    seed_ctrl,          //秧苗控制
    shoot_ctrl,         //发射控制

                        //底盘状态
    high_speed,         //高速模式
    low_speed,          //低速模式

                        //秧苗机构
    seed_disable,       //不使用
    seed_init,          //使用初始化
    peek_deposit,       //取苗储苗
    take_put,           //拿苗放苗

                        //射球机构
    shoot_init,         //射球机构初始化并等待
    load,               //装填
    shooting            //发射
}State;

/*
    seed_init,          //夹苗机构初始化 失能
    predown,            //预先下降      使能
    clip,               //夹紧
    peekup,             //夹起
    deposit,            //存入
    takeout,            //取出
    preup,              //预先上升
*/


extern State robot_state;
extern State chassis_state;
extern State seed_state;
extern State shoot_state;

void robot_fsm(void);
void SWA_judge(void);
void SWB_judge(void);
void SWC_judge(void);
void SWD_judge(void);

#endif //INC_2024RC_B_R1_FSM_H
