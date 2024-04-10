//
// Created by Ray on 2023/11/24.
// Author:  Qin collaborate with YangJianYi and ChengYuFeng  

#include "main.h"
#define locked_rotor_time 10   //堵转时间，若电流值大于所测的电机堵转电流值，超过此时间则判定为电机堵转

//还需改进：左右两个机构标志位和角度需分开写
SEED_CONTROL_STATE Left_Seed_Control;   //左秧苗控制状态结构体
SEED_CONTROL_STATE Right_Seed_Control;  //右秧苗控制状态结构体
SEED_FLAG Seed_Flag = {0};                    //夹苗标志位结构体
//夹秧苗初始化函数
void Init_Seed(void)
{
     Seed_Flag.desposit_first_flag = 0;     //储苗机构第一次进入标志位置0
     Seed_Flag.put_first_flag = 0;          //放苗机构第一次进入标志位置0

    Left_Seed_Control.rotate_motor = &can1motorRealInfo[4];     //左6020转动电机结构体
    Left_Seed_Control.cam_motor = &can1motorRealInfo[6];        //左边凹轮
    Left_Seed_Control.lift_motor = &can1motorRealInfo[5];       //左边抬苗
    Left_Seed_Control.peek_angle = 1350;      //左夹苗6020的角度位置(未测定)
    Left_Seed_Control.deposit_angle = 5300;  //左储苗6020的角度位置（未测定）
    Left_Seed_Control.lift_angle = 30;       //左抬升机构的角度位置（未测定）

    Right_Seed_Control.rotate_motor = &can2motorRealInfo[4];    //右6020转动电机结构体
    Right_Seed_Control.cam_motor = &can2motorRealInfo[6];       //右边凹轮
    Right_Seed_Control.lift_motor = &can2motorRealInfo[5];      //右边抬苗
    Right_Seed_Control.peek_angle = 5600;      //右夹苗6020的角度位置
    Right_Seed_Control.deposit_angle = 1400;  //右储苗6020的角度位置
    Right_Seed_Control.lift_angle = 30;       //右抬升机构的角度位置

    //转圈6020转到储苗位置
    Position_Control(Right_Seed_Control.rotate_motor,Right_Seed_Control.deposit_angle);          //右6020电机位置环控制
    Position_Control(Left_Seed_Control.rotate_motor,Left_Seed_Control.deposit_angle);          //左6020电机位置环控制
    //电池铁放开
    HAL_GPIO_WritePin(GPIOG,Left_Seed_Claw_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG,Right_Seed_Claw_Pin, GPIO_PIN_RESET);

    //凸轮电机锁死
    Speed_Control(Left_Seed_Control.cam_motor,0);          //左凹轮锁死
    Speed_Control(Right_Seed_Control.rotate_motor,0);           //右凹轮锁死

    //抬升机构速度置零
    Speed_Control(Left_Seed_Control.lift_motor,0);          //左抬升机构速度置零
    Speed_Control(Right_Seed_Control.lift_motor,0);         //右抬升机构速度置零
}


//拿苗存苗      
void Deposit_Seed(void)
{
    //初始化状态6020在储苗位置，SWD为上拨状态，若下拨，则移动到云台移动到拿苗位置，若再次上拨，表明已经对准苗，夹爪夹紧、抬升机构上升，6020转到储苗位置
    if(Seed_Flag.desposit_first_flag == 0)       //第一次进入
    {
        Seed_Flag.put_first_flag = 0;          //放苗机构第一次进入标志位置0
        //6020到储苗位置
        Position_Control(Right_Seed_Control.rotate_motor,Right_Seed_Control.deposit_angle);          //右6020电机位置环移动到储苗位置
        Position_Control(Left_Seed_Control.rotate_motor,Left_Seed_Control.deposit_angle);            //左6020电机位置环移动到储苗位置
        //电池铁放开
        HAL_GPIO_WritePin(GPIOG,Left_Seed_Claw_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG,Right_Seed_Claw_Pin, GPIO_PIN_RESET);
        //抬升机构下降
        Homeing_Mode(Right_Seed_Control.lift_motor,200,5500);     //抬升机构回零
        Homeing_Mode(Left_Seed_Control.lift_motor,200,5500);      //抬升机构回零

        if(Seed_Flag.seed_aim_flag == 1 )    //SWD下拨，该标志位置1
        {
           Seed_Flag.desposit_first_flag = 1;         //第一次进入标志位置1
        }      
    }
    else        //第二次以上进入
    {
        if (Seed_Flag.seed_aim_flag == 1)   //SWD下拨，云台移动到拿苗位置
        {
            //6020转到拿苗位置
            Position_Control(Right_Seed_Control.rotate_motor,Right_Seed_Control.peek_angle);          //右6020电机位置环控制
            Position_Control(Left_Seed_Control.rotate_motor,Left_Seed_Control.peek_angle);            //左6020电机位置环控制
            //电池铁放开
            HAL_GPIO_WritePin(GPIOG,Left_Seed_Claw_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOG,Right_Seed_Claw_Pin, GPIO_PIN_RESET);
            //抬升机构下降
            Homeing_Mode(Right_Seed_Control.lift_motor,200,5500);     //抬升机构回零到下面
            Homeing_Mode(Left_Seed_Control.lift_motor,200,5500);      //抬升机构回零到下面

            //SWD下拨时，rotate_change_flag置0
            Seed_Flag.rotate_change_flag = 0;
        }
        else if(Seed_Flag.seed_aim_flag == 0)   //SWD上拨，夹爪夹住，抬升机构上升，6020转到储苗位置
        {
            //SWD上拨一次，rotate_flag就加一      
            if (Seed_Flag.rotate_change_flag == 0)
            {
                Seed_Flag.rotate_flag++;
                Seed_Flag.rotate_change_flag = 1;
            }
            //夹爪夹紧
            HAL_GPIO_WritePin(GPIOG,Right_Seed_Claw_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,Left_Seed_Claw_Pin, GPIO_PIN_SET);

            //抬升机构上升
            Position_Control(Right_Seed_Control.lift_motor,Right_Seed_Control.lift_angle);          //右抬升机构位置环控制
            Right_Seed_Control.lift_motor->HomingMode.done_flag = 0;     //右抬升机构回零成功标志位置0
            Position_Control(Left_Seed_Control.lift_motor,Left_Seed_Control.lift_angle);            //左抬升机构位置环控制
            Left_Seed_Control.lift_motor->HomingMode.done_flag = 0;      //左抬升机构回零成功标志位置0

            //判断是否抬升成功
            if(Left_Seed_Control.lift_motor->REAL_ANGLE >= (Left_Seed_Control.lift_angle - 2)&& Left_Seed_Control.lift_motor->REAL_ANGLE <= (Left_Seed_Control.lift_angle + 2))
            {
                //抬升成功后6020转到储苗位置
                Position_Control(Left_Seed_Control.rotate_motor,Left_Seed_Control.deposit_angle);            //左6020电机位置环控制
            }
            //右边也一样
            if(Right_Seed_Control.lift_motor->REAL_ANGLE >= (Right_Seed_Control.lift_angle - 2) && Right_Seed_Control.lift_motor->REAL_ANGLE <= (Right_Seed_Control.lift_angle + 2))
            {
                //抬升成功后6020转到储苗位置
                Position_Control(Right_Seed_Control.rotate_motor,Right_Seed_Control.deposit_angle);          //右6020电机位置环控制
            }     

             //移动到凹轮电机该转的位置     
            if (Left_Seed_Control.rotate_motor->ANGLE >= (Left_Seed_Control.deposit_angle - 600 ) && Left_Seed_Control.rotate_motor->ANGLE <= (Left_Seed_Control.deposit_angle + 300 ))
            {          
                //凹轮电机转动
                Position_Control(Left_Seed_Control.cam_motor,122*Seed_Flag.rotate_flag);         //左凹轮电机转动                  
            }
            //右边也一样
            if (Right_Seed_Control.rotate_motor->ANGLE >= (Right_Seed_Control.deposit_angle - 1000 ) && Right_Seed_Control.rotate_motor->ANGLE <= (Right_Seed_Control.deposit_angle + 1000 ))
            {           //移动到凹轮电机该转的位置
                    //凹轮电机转动
                Position_Control(Right_Seed_Control.cam_motor,122*Seed_Flag.rotate_flag);         //右凹轮电机转动                  
            }        
        }
    } 
}

//放苗函数
void Put_Seed(void)
{
        if(Seed_Flag.seed_aim_flag == 0)    //SWD上拨状态,6020转到储苗位置，抬升机构上升，夹爪到指定位置夹紧
        {
            Seed_Flag.rotate_change_flag = 0;      //凹轮电机转动角度改变标志位置0

            //夹爪放开
            HAL_GPIO_WritePin(GPIOG,Left_Seed_Claw_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOG,Right_Seed_Claw_Pin, GPIO_PIN_RESET);

            //抬升机构上升
            Position_Control(Right_Seed_Control.lift_motor,Right_Seed_Control.lift_angle);          //右抬升机构位置环控制
            Right_Seed_Control.lift_motor->HomingMode.done_flag = 0;     //右抬升机构回零成功标志位置0
            Position_Control(Left_Seed_Control.lift_motor,Left_Seed_Control.lift_angle);            //左抬升机构位置环控制
            Left_Seed_Control.lift_motor->HomingMode.done_flag = 0;      //左抬升机构回零成功标志位置0

            //6020转到储苗位置
            //判断是否抬升成功
            if(Left_Seed_Control.lift_motor->REAL_ANGLE >= (Left_Seed_Control.lift_angle - 2)&& Left_Seed_Control.lift_motor->REAL_ANGLE <= (Left_Seed_Control.lift_angle + 2))
            {
                //抬升成功后6020转到储苗位置
                Position_Control(Left_Seed_Control.rotate_motor,Left_Seed_Control.deposit_angle);            //左6020电机位置环控制
            }
            //右边也一样
            if(Right_Seed_Control.lift_motor->REAL_ANGLE >= (Right_Seed_Control.lift_angle - 2) && Right_Seed_Control.lift_motor->REAL_ANGLE <= (Right_Seed_Control.lift_angle + 2))
            {
                //抬升成功后6020转到储苗位置
                Position_Control(Right_Seed_Control.rotate_motor,Right_Seed_Control.deposit_angle);          //右6020电机位置环控制
            }
        }
        else if (Seed_Flag.seed_aim_flag == 1)  //SWD下拨，夹爪夹紧，6020转到放苗位置，凹槽电机同时开始转动，到达位置后抬升机构下降
        {
            //SWD上拨一次(标志位已置零)，rotate_flag就减一      
            if (Seed_Flag.rotate_change_flag == 0)
            {
                Seed_Flag.rotate_flag--;
                Seed_Flag.rotate_change_flag = 1;
            }
            //夹爪夹紧
            HAL_GPIO_WritePin(GPIOG,Left_Seed_Claw_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,Right_Seed_Claw_Pin, GPIO_PIN_SET);

            //6020转到放苗位置
            Position_Control(Left_Seed_Control.rotate_motor,Left_Seed_Control.peek_angle);            //左6020电机位置环控制 
            Position_Control(Right_Seed_Control.rotate_motor,Right_Seed_Control.peek_angle);          //右6020电机位置环控制
            //凹轮电机转动
            Position_Control(Left_Seed_Control.cam_motor,122*Seed_Flag.rotate_flag);         //左凹轮电机转动
            Position_Control(Right_Seed_Control.cam_motor,122*Seed_Flag.rotate_flag);         //右凹轮电机转动
            //判断到达位置，抬升机构下降
            if (Left_Seed_Control.rotate_motor->ANGLE >= (Left_Seed_Control.peek_angle - 50 ) && Left_Seed_Control.rotate_motor->ANGLE <= (Left_Seed_Control.peek_angle + 50 ))
            {
                //抬升机构下降
                Homeing_Mode(Left_Seed_Control.lift_motor,200,5500);      //左抬升机构回零到下面
            }
            //右边也一样
            if (Right_Seed_Control.rotate_motor->ANGLE >= (Right_Seed_Control.peek_angle - 50 ) && Right_Seed_Control.rotate_motor->ANGLE <= (Right_Seed_Control.peek_angle + 50 ))
            {
                //抬升机构下降
                Homeing_Mode(Right_Seed_Control.lift_motor,200,5500);     //右抬升机构回零到下面
            }    
        }
}

//测试6020单圈模式   角度范围0 ~~ 8191(仅仅是测试，没什么用)
void seed_control(void)
{

    if (SWD > 900 && SWD < 1100)
    {      
        Velocity_Planning_setpos(&can1motorRealInfo[5],1100,5196,200,1000,100,0.1,0.05);
        Velocity_Planning_MODE(&can1motorRealInfo[5]);
      // Position_Control(&can1motorRealInfo[5],5396);          //左6020电机位置环控制
    }
    else if(SWD < 2100 && SWD > 1900)
    {
        Velocity_Planning_setpos(&can1motorRealInfo[5],5196,1100,200,1000,100,0.1,0.2);
        Velocity_Planning_MODE(&can1motorRealInfo[5]);
       // Position_Control(&can1motorRealInfo[5],1500);
    } 
}


//----------云台相关----------------------------
GIMBAL Gimbal;   //云台结构体
int16_t Theta;  //车体相对于目标点的切角
int16_t Alpha;  //自转补偿
//---------------------------------------------
//云台控制，author: YangJiangYi
void Gimbal_controller(void)
{
    Gimbal.TAN = (Gimbal.target_point[0] - ROBOT_CHASSI.world_x)/(Gimbal.target_point[1] - ROBOT_CHASSI.world_y);   //正切角计算
    Gimbal.Theta = atan(Gimbal.TAN);
    Gimbal.Theta = (Gimbal.Theta*180/3.14);
    Gimbal.Descripe = (float)25/13; //减速比，大概。需要问机械，有待更改
    Gimbal.gimbal_angle = can2motorRealInfo[3].REAL_ANGLE/Gimbal.Descripe;

    Theta = (int16_t)Gimbal.Theta;
    Alpha = (int16_t)ROBOT_CHASSI.world_w;
}



uint8_t claw_state = 1;     //夹爪状态，堵转时为0，为堵转为1
int shoot_flag  = 0, shoot_count = 0,aim_count = 0,u8_count = 0;   //射球标志位， 0 为不射，1为射
/**
 * @brief 射球机构初始化    发射机构停止，云台回正，夹爪回到最下面，放球
 * @param  NULL
 * @return NULL
*/
void Init_Ball(void)
{    
    //云台速度及瞄准位置赋值
      Gimbal.target_point[0] = -6000;      //X
      Gimbal.target_point[1] = 8900;     //Y
      Gimbal.revole_rpm = 350;
    aim_count = 0;
    //发射机构停止
    u8_ctrl(0);
    Speed_Control(&can2motorRealInfo[0],0);
    Speed_Control(&can2motorRealInfo[1],0);
    shoot_flag = 1;    //射球标置位置1
    shoot_count = 0;  //射球计数器置0 
    //云台手动控制模式
        Gimbal.aim_flag = 0;       //瞄准成功标志位置0
        Gimbal.centor_flag = 0;    //回正标志位置0  
        Gimbal_controller();
            if(Gimbal.gimbal_angle < 120 && Gimbal.gimbal_angle > -120)
            {
                if(YaoGan_RIGHT_Y !=0)
                {
                 if(YaoGan_RIGHT_Y-1500>100)
                    Speed_Control(&can2motorRealInfo[3],Gimbal.revole_rpm);
                else if(YaoGan_RIGHT_Y-1500<-100)
                    Speed_Control(&can2motorRealInfo[3],-Gimbal.revole_rpm);
                else
                    Speed_Control(&can2motorRealInfo[3],0);
                }      
            }
            else
            {
               if(YaoGan_RIGHT_Y !=0)
               {
                 Speed_Control(&can2motorRealInfo[3],0);
                if(can2motorRealInfo[5].REAL_ANGLE > 120)
                {
                    if(YaoGan_RIGHT_Y-1500<-100)
                        Speed_Control(&can2motorRealInfo[3],-Gimbal.revole_rpm);
                }
                else
                {   
                    if(YaoGan_RIGHT_Y-1500>100)
                        Speed_Control(&can2motorRealInfo[3],Gimbal.revole_rpm);
                }
               }                    
            }

    //夹爪回到最下面
    if(test_rise_time_up(ABS(can2motorRealInfo[2].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //堵转时夹爪停止下降       
   //夹爪下降
    if (claw_state)             //还没堵转
    {
        Speed_Control(&can2motorRealInfo[2],-200);     //夹爪下来
    }
    else    //检测到堵转
    {
        Speed_Control(&can2motorRealInfo[2],0); //速度调零
        can2motorRealInfo[2].REAL_ANGLE = 0;    //位置置零
    }
    //电池铁放开
    HAL_GPIO_WritePin(GPIOG,Ball_Claw_Pin, GPIO_PIN_RESET);
}


/**
 * @brief 射球函数   发射机构启动，云台瞄准，夹爪下降
 * @param  NULL
 * @return NULL
*/

void Shoot_Ball()
{   
    //云台瞄准
    Gimbal_controller();
    Gimbal.centor_flag = 0;     //回正标志位置0
    if(Gimbal.gimbal_angle <= 120 && Gimbal.gimbal_angle >= -120)   
    {
        if(Alpha+Theta > Gimbal.gimbal_angle+1)
        {  
            Speed_Control(&can2motorRealInfo[3],Gimbal.revole_rpm);
        }
        else if(Alpha+Theta < Gimbal.gimbal_angle-1)
        {
            Speed_Control(&can2motorRealInfo[3],-Gimbal.revole_rpm);
        }
        else
        {
            Speed_Control(&can2motorRealInfo[3],0);     //成功瞄准
            Gimbal.aim_flag = 1;    
        }
    }
    else
    {
        if(Gimbal.gimbal_angle > 120)
        {
            Vel_Torque_Control(&can2motorRealInfo[3],2000,-50);    //限流
        }
        else
        {   
            Vel_Torque_Control(&can2motorRealInfo[3],2000,50);     
        }
    }

    if (Gimbal.aim_flag)    //成功瞄准
    {
        //发射
        if(shoot_flag)
        {
            aim_count++;
               if (shoot_speed == far_shoot )      u8_ctrl(12);       //远射
               else if(shoot_speed == near_shoot ) u8_ctrl(7);    //近射
          if(aim_count>2000 ) 
          {
                Speed_Control(&can2motorRealInfo[0],-7000);
                Speed_Control(&can2motorRealInfo[1],7000);    //履带转动
                shoot_count++;
          }
     
        if(shoot_count > 3000) shoot_flag = 0;
        }
        else
        {
                u8_ctrl(0);    //停球 
                Speed_Control(&can2motorRealInfo[0],0);
                Speed_Control(&can2motorRealInfo[1],0);     
        }
    }
    
    if(test_rise_time_up(ABS(can2motorRealInfo[2].CURRENT),5500,locked_rotor_time)) claw_state = 0;   //堵转时夹爪停止下降       
   //夹爪下降
    if (claw_state)             //还没堵转
    {
        Speed_Control(&can2motorRealInfo[2],-200);     //夹爪下来
    }
    else    //检测到堵转
    {
        Speed_Control(&can2motorRealInfo[2],0); //速度调零
        can2motorRealInfo[2].REAL_ANGLE = 0;    //位置置零
    }
    
}

/**
 * @brief 装球函数    发射机构停止，云台回正，夹球，夹爪上去，放球
 * @param  NULL
 * @return NULL
*/
void Load_Ball(void)
{
    Gimbal_controller();
    Gimbal.aim_flag = 0;   //瞄准成功标志位置0
    aim_count= 0;
    //云台回正
    if(0 > Gimbal.gimbal_angle+1)
    {          
        Speed_Control(&can2motorRealInfo[3],Gimbal.revole_rpm);
    }
    else if(0 < Gimbal.gimbal_angle-1)
    {
        Speed_Control(&can2motorRealInfo[3],-Gimbal.revole_rpm);
    }
    else
    {
        Speed_Control(&can2motorRealInfo[3],0);
        Gimbal.centor_flag = 1;    //回正标志位赋值
    }

    //发射机构停止
    u8_ctrl(0);
    Speed_Control(&can2motorRealInfo[0],0);
    Speed_Control(&can2motorRealInfo[1],0); 

    if (can2motorRealInfo[2].REAL_ANGLE > 138 && can2motorRealInfo[2].REAL_ANGLE < 142) //判断是否到达顶点，如果判断过快加延时，也要加个判断云台是否回正
    {
        if (Gimbal.centor_flag)         //如果云台已经回正
        {
            //夹爪松开放球
            HAL_GPIO_WritePin(GPIOG,Ball_Claw_Pin, GPIO_PIN_RESET);
            shoot_flag = 1;    //射球标置位置1
             shoot_count = 0;  //射球计数器置0           
        }

    }
    else//没到顶点时夹紧
    {
        //夹爪夹紧夹球
        HAL_GPIO_WritePin(GPIOG,Ball_Claw_Pin, GPIO_PIN_SET);
    }
    //位置环上去 
    Position_Control(&can2motorRealInfo[2],140);        
    claw_state = 1;     //状态改为未堵转状态
}


 uint16_t time_up = 0;
/**
  * @brief  检测是否堵转，堵转则返回1，否则返回0
  * @param  电流，堵转电流，堵转时间
  * @retval 
  */
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time)
{ 
  if(current<boundary_current) time_up=0;
  if(current>boundary_current) time_up++;
  if(time_up>=boundary_time)return 1;
  return 0; 
}
