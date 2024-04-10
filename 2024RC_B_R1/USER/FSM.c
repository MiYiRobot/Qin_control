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
    case init:          //初始化
        chassis_state = high_speed;
        seed_state = seed_disable;
        shoot_state = shoot_init;
        chassis_init();
        robot_state = calibration;
        break;
    case calibration:     //校准
        ROBOT_CHASSI.Vx_MAX = 0.0f;
        ROBOT_CHASSI.Vy_MAX = 0.0f;
        ROBOT_CHASSI.Vw_MAX = 0.0f;
        if (SWA != 0 && SWB != 0 && SWC != 0 && SWD != 0)   //确认航模初始化成功
        {
            robot_state = auto_ctrl;
//            SWA_judge();
//            SWB_judge();
//            SWC_judge();
//            SWD_judge();
        }
        break;
    case auto_ctrl:         //自动控制
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
    case seed_ctrl:         //秧苗控制 

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
    if (SWA < 1500)    //上拨为底盘高速行驶
    {
        chassis_state = high_speed;
    }
    else if (SWA > 1500)    //下拨为底盘低速行驶
    {
        chassis_state = low_speed;
    }
}

void SWB_judge(void)
{
    if (SWB < 1200)     //SWB上拨为自动控制
    {
        robot_state = auto_ctrl;
    }
    else if (1300 < SWB && SWB < 1700)  //SWB中间档位为秧苗控制
    {
        robot_state = seed_ctrl;
    }
    else if (SWB > 1800)            //SWB下拨为发射结构控制
    {
        robot_state = shoot_ctrl;
    }
}

void SWC_judge(void)
{
    if (SWC < 1200)     //SWC上拨
    {
        if (robot_state == auto_ctrl)   //自动控制，待填充
        {
            //
        }
        else if (robot_state == seed_ctrl)  
        {
            seed_state = seed_init;     //秧苗初始化（电机先停）
        }
        else if (robot_state == shoot_ctrl)
        {
            shoot_state = shoot_init;   //发射结构初始化
        }
    }
    else if (1300 < SWC && SWC < 1700)  //SWC中间档
    {
        if (robot_state == auto_ctrl)
        {
            //
        }
        else if (robot_state == seed_ctrl)
        {
            seed_state = peek_deposit;      //取苗
        }
        else if (robot_state == shoot_ctrl)
        {
            shoot_state = load;         //装球
        }
    }
    else if (SWC > 1800)        //SWC下拨
    {
        if (robot_state == auto_ctrl)
        {
            //
        }
        else if (robot_state == seed_ctrl)
        {
            seed_state = take_put;          //放苗
        }
        else if (robot_state == shoot_ctrl)
        {
            shoot_state = shooting;       //发射  
        }
    }
}

void SWD_judge(void)
{
    if (SWD < 1500)
    {
        
        if (robot_state == auto_ctrl)   //自动控制，待填充
        {
            //
        }
        else if (robot_state == seed_ctrl)  //上拨为6020移动到储苗位置，当夹爪对准苗时上拨
        {
            Seed_Flag.seed_aim_flag = 0;      //夹苗对准苗标志位置0
        }   
        else if (robot_state == shoot_ctrl)  //
        {
            shoot_speed = far_shoot;   //上拨为远射
        }

    }
    else if (SWD > 1500)
    {
                if (robot_state == auto_ctrl)   //自动控制，待填充
        {
            //
        }
        else if (robot_state == seed_ctrl)  //下拨为6020移动到拿苗放苗位置
        {
            Seed_Flag.seed_aim_flag = 1;      //夹苗对准标志位置1
        }
        else if (robot_state == shoot_ctrl)  //
        {
            shoot_speed = near_shoot;   //下拨为近射
        }
    }
}
