#include "FSM.h"
//此代码用来实现stm32端的测试工作

/****************************************************************************************************************************
2023/4/26 Author：Qin Bo
该文件用于编写stm32端的状态机，Robot_Control_Mode函数为机器人的状态控制函数
*****************************************************************************************************************************/

//状态机编写
void Fsm32()
{
    if(SWA!=0&SWB!=0&SWC!=0&SWD!=0)         //判断是否接收到遥控器数据
    {
        Robot_Control_Mode();   //状态机编写       
    }
    else
    {
      R2_CONTROLLER.NEXT_CONTROLLER_STATE = CONTROLLER_OFF;    //上层机构停止
    }
    R2_CONTROLLER.NOW_CONTROLLER_STATE=upper_controller(R2_CONTROLLER.NEXT_CONTROLLER_STATE);     //上层机构控制器
   // liner_actuator();
}

/**
 * @brief 机器人控制模式函数，手动
*/
void Robot_Control_Mode(void)
{ 
    if (SWA >= 900 && SWA <= 1100)  //待机状态
    {
       R2_CONTROLLER.NEXT_CONTROLLER_STATE = CONTROLLER_OFF;    //上层机构停止 
    }
    else if(SWA >= 1900 && SWA <= 2100)   //启动状态
    {
            //取球
            if(SWC>=950 && SWC<=1050)       //SWC上拨，取球
            {
                R2_CONTROLLER.NEXT_CONTROLLER_STATE = TAKE_BALL;  

            }
            else if(SWC>=1450 && SWC<=1550) //SWC中拨，拿球
            {
              // R2_CONTROLLER.NEXT_CONTROLLER_STATE = GET_BALL;
            }
            else if(SWC>1950 && SWC<2050)       //放球
            {
                R2_CONTROLLER.NEXT_CONTROLLER_STATE = SHOOT_BALL;
            } 

            //取球
            if(SWB>=950 && SWB<=1050)       //SWC上拨，取球
            {
                R2_CONTROLLER.filter_ball_state = INVERSE_BALL;

            }
            else if(SWB>=1450 && SWB<=1550) //SWC中拨，拿球
            {
              R2_CONTROLLER.filter_ball_state = LEFT_FILTER_BALL;
            }
            else if(SWB>1950 && SWB<2050)       //放球
            {
                R2_CONTROLLER.filter_ball_state = RIGHT_FILTER_BALL;
            }
                    
    }
}



