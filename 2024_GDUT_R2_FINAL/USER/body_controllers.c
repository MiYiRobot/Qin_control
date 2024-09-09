#include "body_controllers.h"
R2_CONTROLLER_STR R2_CONTROLLER = {0};
#define STOP_ANGLE 128
int cnt1 = 0;
int16_t cnt2 = 0;
int cnt3 = 0;
int finish_flag = 0; // 放球完成标志位
int jiaqiu_flag = 0;
int taishen_flag = 0;
int8_t jia_Reset = 0; // 夹球重开标志位
int8_t Jia_Reset_Open = 0; //夹爪重夹打开标志位
int Jia_cnt = 0;
int Tai_cnt = 0;
int16_t lock_cnt;
int16_t clear_cnt;
uint8_t wushai_flag = 0;        //误筛标志位
uint16_t wushai_cnt;             
/**
 * @brief 上层构控制器，该控制机构采用3个滚筒5065电机和3个摩擦轮3508电机
 * @param 机构的下个状态
 * @return 机构的当前状态
 */
CONTROLLER_STATE upper_controller(CONTROLLER_STATE NEXT_STATE)
{
    for (int i = 0; i < 3; i++)
        VESC_MOTO_INFO[i].MOTOR_MODE = VESC_SPEED; // 速度控制
    //      for(int i=0; i<3; i++)
    //          VESC_MOTO_INFO[i].MOTOR_MODE = VESC_DUTY;       //占空比控制
    switch (NEXT_STATE)
    {
    case TAKE_BALL: // 吸球状态
        take_ball();
      //  motor_controller(-000, 000, 10000, 000, -000, -000,-000,-000);
   // motor_controller(-2500, 2300, 5000, -1000, -1200, -3000,-20000,-1250);
        return TAKE_BALL;
    case SHOOT_BALL: // 出球状态
        motor_controller(0, 0, 0, 0, 0, 0,0,0);

        // 检测到堵转
        if (MOTO_REAL_INFO[1].CURRENT >= 4000)
        { // 完全展开夹爪，放球完成
            Jia_cnt++;
        }
        else
        {
            Jia_cnt = 0;
        }
        if (Jia_cnt >= 10)
            finish_flag = 1;

        if (finish_flag) // 放球完成
        {
            //上下位机标志位赋值       
            R2_CONTROLLER.ball_on_up = 0;
            R2_CONTROLLER.useful_ball_status = NO_USEFULBALL;   
            Vel_Torque_Control(&MOTO_REAL_INFO[1], 1000, 100); // 暂停张开夹爪
            R2_CONTROLLER.Ball_sta = BALL_OUTSIDE;
        }
        else
        {
            Vel_Torque_Control(&MOTO_REAL_INFO[1], 7000, 2000); // 打开夹爪放球
        }
        return SHOOT_BALL;
    case CONTROLLER_OFF: // 控制器关闭状态
        motor_controller(0, 0, 0, 0, 0, 0,0,0);
        Speed_Control(&MOTO_REAL_INFO[0], 0);
        Speed_Control(&MOTO_REAL_INFO[1], 0);
        return CONTROLLER_OFF;
    default:
        {
            motor_controller(0, 0, 0, 0, 0, 0,0,0);
            Speed_Control(&MOTO_REAL_INFO[0], 0);
        Speed_Control(&MOTO_REAL_INFO[1], 0);
            return CONTROLLER_ERROR;
        }
    }
}

void motor_controller(int16_t right_3508_rpm, int16_t left_3508_rpm, int16_t back_motor_rpm,
                      int16_t Below_Roller_rpm, int16_t Middle_Roller_rpm, int16_t Up_Roller_rpm,int16_t filler_3508_rpm,int16_t front_2006_rpm)
{
    Speed_Control(&MOTO_REAL_INFO[4], right_3508_rpm);
    Speed_Control(&MOTO_REAL_INFO[5], left_3508_rpm);
    Speed_Control(&MOTO_REAL_INFO[6], back_motor_rpm);
    VESC_MOTO_INFO[0].TARGET_RPM = Below_Roller_rpm;
    VESC_MOTO_INFO[1].TARGET_RPM = Middle_Roller_rpm;
    Speed_Control(&MOTO_REAL_INFO[3],Up_Roller_rpm);
   //Speed_Control(&MOTO_REAL_INFO[2],filler_3508_rpm);
    VESC_MOTO_INFO[2].TARGET_RPM = filler_3508_rpm;
    Speed_Control(&MOTO_REAL_INFO[2],front_2006_rpm);
}

// 夹爪拍下
void Claw_PutDown()
{
    Vel_Torque_Control(&MOTO_REAL_INFO[1], 5000, 0); // 夹爪已经张到最大了，速度置零

    if (taishen_flag == 0) // 还没拍下来
    {
        Vel_Torque_Control(&MOTO_REAL_INFO[0], 7000, -1000); // 拍下来          111111111111111111111111111
         if (ABS(MOTO_REAL_INFO[0].CURRENT) >= 5000)
        { // 检测到拍到到尽头
                    Tai_cnt++;
        }
        else
        {
            Tai_cnt = 0;
        }
        if (Tai_cnt >= 20)
        {
            taishen_flag = 1; // 拍到最下面了
        }

    }
    else
    {                                     // 抬升完成
        MOTO_REAL_INFO[0].REAL_ANGLE = 0; // 消除累计误差
        MOTO_REAL_INFO[0].Motor_Mode = CURRENT_MODE;
        MOTO_REAL_INFO[0].TARGET_CURRENT = 0;
        MOTO_REAL_INFO[1].Motor_Mode = CURRENT_MODE;
        MOTO_REAL_INFO[1].TARGET_CURRENT = 0;      
        jiaqiu_flag = 0;                  // 夹球标志位置零
       // R2_CONTROLLER.Ball_sta = BALL_OUTSIDE; // 球已经在车外了
        cnt1 = 0;
        cnt2 = 0;
        cnt3 = 0;
        Jia_cnt = 0;
        Jia_Reset_Open = 0;
       jia_Reset = 0;
        finish_flag = 0; // 放球完成标志位置零
        R2_CONTROLLER.ball_on_up = 0;           //球在上面标志位置零
        R2_CONTROLLER.useful_ball_status = NO_USEFULBALL;
    }
}
uint16_t time1 = 0;
uint16_t time1_flag = 0;
uint16_t Jia_Reset_Open2 = 0;
// 夹球并抬上去
void Get_Ball()
{
    if (cnt2 >= 230) // 光电门检测到延时70ms减速
    {
        if (jia_Reset) // 夹球重开了
        {       
            if(Jia_Reset_Open)   //夹爪已经张到最大
            {

                //重新夹球
                if (jiaqiu_flag == 0)       //还没夹到球
                {    
                     cnt1 = 0;
                    motor_controller(00, 00, 000, 000, 3000, 2800,0,0); // 还没夹到球慢慢转给，球一个适当的高度
                    Vel_Torque_Control(&MOTO_REAL_INFO[1], 7000, -2000); // 夹球 

                }
                else // 夹到球后滚筒停止
                {
                    motor_controller(0, 0, 0, 0, 0, 0,0,0);
                    //位置环开转
                    if (Position_Control(&MOTO_REAL_INFO[0], STOP_ANGLE))
                    {
                        Tai_cnt = 0;
                        finish_flag = 0;
                        taishen_flag = 0; // 还没拍到最下面
                        Jia_cnt = 0;
                        Vel_Torque_Control(&MOTO_REAL_INFO[1], 5000, -2000); // 按死夹爪
                        // 上下位机标志位赋值
                        R2_CONTROLLER.Ball_sta = BALL_IN_CLAW;
                    }
                    else
                    {
                         Vel_Torque_Control(&MOTO_REAL_INFO[1], 5000, -2000); 
                    }
                }
                

                if (R2_CONTROLLER.claw_state == CLAW_CLOSE) // 检测夹到球       11111111111
                {
                    jiaqiu_flag = 1;
                }                
            }
            else        //夹爪还没张到最大
            {
                Vel_Torque_Control(&MOTO_REAL_INFO[1], 7000, 2000);        //重新打开夹爪

                motor_controller(00, 00, 000, 000, 3000, 1200,0,0);   //滚筒把球转上去
                if (MOTO_REAL_INFO[1].CURRENT >= 5000)         //夹爪已经开到最大
                {
                    Jia_Reset_Open = 1;
                }
            }


        }
        else // 不用重开
        {

            if (jiaqiu_flag == 0)
            {                                              // 还没夹到球慢慢转给，给球一个适当的高度
                  if (R2_CONTROLLER.filter_ball_state == LEFT_FILTER_BALL)   
                        motor_controller(1200, -1200, 000, 0000, 2000, 1800,6500,0); // 暂时不转
                 else if (R2_CONTROLLER.filter_ball_state == RIGHT_FILTER_BALL)  
                        motor_controller(1200, -1200, 000, 0000, 2000, 1800,-6500,0);
                 else 
                        motor_controller(1200, -1200, 000, 0000, 2000, 1800,-6500,0);
            }
            else // 夹到球后滚筒停止
            {
                motor_controller(0, 0, 0, 0, 0, 0,0,0);
            }

            if (cnt3 >= 130) // 开夹
            {

                if (jiaqiu_flag == 1) // 检测到夹到球
                {
                    if (Position_Control(&MOTO_REAL_INFO[0], STOP_ANGLE))
                    {
                        Tai_cnt = 0;
                        finish_flag = 0;
                        taishen_flag = 0; // 还没拍到最下面
                        Jia_cnt = 0;
                        Jia_Reset_Open = 0;   
                        Vel_Torque_Control(&MOTO_REAL_INFO[1], 5000, -2000); // 按死夹爪
                        // 上下位机标志位赋值
                        R2_CONTROLLER.Ball_sta = BALL_IN_CLAW;
                    }
                    else
                    {
                        Vel_Torque_Control(&MOTO_REAL_INFO[1], 5000, -2000); // 按死夹爪
                    }
                    
                    
                }
                else
                {
                    
                    Vel_Torque_Control(&MOTO_REAL_INFO[1], 5000, -2000); // 夹球     1111111111111111
                    // 微动开关还没检测到，但有可能没夹到，加个延时，延时到了重开
                     if (cnt1 >= 400)       //开夹超过一定时间后还没夹到就代表得重开了
                    {
                        // 重开
                        jia_Reset = 1;
                    }
                    else
                    {
                        cnt1++;
                    }
                    
                }
                //微动开关一触发就开始翻转
                if (R2_CONTROLLER.claw_state == CLAW_CLOSE) // 检测夹到球       11111111111
                {
                    Jia_cnt++;
                }
                if(Jia_cnt>=50)  jiaqiu_flag = 1;
            }
            else
            {
                cnt3++;
            }
        }
    }
    else
    {
        cnt2++;
    }
}

int flag_ball = 0;
uint8_t ball_move_flag = 0;     //紫球离开标志位
uint16_t ball_move_cnt = 0;
//球的状态判断
void ball_judge()
{
    //下面那关电门检测到东西，场地灯光原因，加了层保护
        if(R2_CONTROLLER.ball_position == BALL_inner || R2_CONTROLLER.ball_position == BALL_BOTH)
        {
             //下面那传感器检测到有效球
            if (enter_ball.ball_color == BLUE_BALL || enter_ball.ball_color == RED_BALL)
            {
                R2_CONTROLLER.useful_ball_status = USEFULBALL_ENTER;
            } 
        }

        //下面传感器检测到有效球后，中间和上面传感器检测不到无效球
        if(R2_CONTROLLER.useful_ball_status == USEFULBALL_ENTER)
        {
            if(out_ball.ball_color != PURPLE_BALL && middle_ball.ball_color != PURPLE_BALL) ball_move_cnt++;    
           else        ball_move_cnt = 0;
            if(ball_move_cnt >= 300)
            ball_move_flag = 1;
        }  

        if(R2_CONTROLLER.useful_ball_status == USEFULBALL_MIDDLE)
        {
            if(out_ball.ball_color != PURPLE_BALL && middle_ball.ball_color != PURPLE_BALL) ball_move_cnt++;    
           else        ball_move_cnt = 0;
            if(ball_move_cnt >= 100)
            ball_move_flag = 1;
        }
        
        //中间那个光电门检测到有东西
        if(R2_CONTROLLER.ball_position == BALL_middle || R2_CONTROLLER.ball_position == BALL_BOTH)
        {
            //中间那传感器检测到有效球
            if (out_ball.ball_color == RED_BALL || out_ball.ball_color == BLUE_BALL)
            {
                R2_CONTROLLER.useful_ball_status = USEFULBALL_MIDDLE;
            }    
        }
        
        //最上面的光电门检测到
        if(R2_CONTROLLER.useful_ball_status == USEFULBALL_MIDDLE)
        {
            if (R2_CONTROLLER.Gate_state == UP)
            {
                R2_CONTROLLER.ball_on_up = 1;     //球在上面的标志位赋值     
            }
        }

        //还没堵转时,判断电流过大时，设为堵转状态
        if(R2_CONTROLLER.lock_flag == 0)
        {
            //检测两个摩擦带有一个堵转
            if(ABS(MOTO_REAL_INFO[4].CURRENT) >= 10000 || ABS(MOTO_REAL_INFO[5].CURRENT) >= 10000)
                lock_cnt++;
            else    lock_cnt = 0;
            if(lock_cnt >=80)   R2_CONTROLLER.lock_flag =1;
        }
        else        //已经堵转了，判断电流小于一定值，设为不堵转状态
        {
            if(ABS(MOTO_REAL_INFO[4].CURRENT) <= 6500 || ABS(MOTO_REAL_INFO[5].CURRENT) <= 6500)
                clear_cnt++;
            else
            clear_cnt= 0;
            if(clear_cnt >=10)
                 R2_CONTROLLER.lock_flag =0;
        }      
}
uint16_t mis_cnt = 0;

//筛球控制器
void  take_ball()
{
        if (finish_flag) // 放球成功之后，夹爪还没放下来
        {
            Claw_PutDown();
        }
        else            //
        {
            if(R2_CONTROLLER.ball_on_up)    //最上面的关电门检测到球和吸球刚进去时的状态 
            {
                 ball_move_flag = 0;
                 ball_move_cnt = 0; 
                 wushai_cnt = 0;
                mis_cnt = 0;
                 Get_Ball();
            }
            else            //有效球还没到最上面
            {
                ball_judge();
                //进球口还没检测到有效球
                if(R2_CONTROLLER.useful_ball_status == NO_USEFULBALL)
                {
                    //继续开始爽吸，和爽筛
                    if (R2_CONTROLLER.filter_ball_state == LEFT_FILTER_BALL)        //左筛
                    {
//                       if(ball_stop_flag)   motor_controller(-2300, 2000, 000, 3000, -1200, -3000,-15000,-200);
//                       else
                        
                         motor_controller(-2600, 2600, 10000, -3000, 1200, -3000,6500,-1250);
                    }
                    else if (R2_CONTROLLER.filter_ball_state == RIGHT_FILTER_BALL)  //右筛
                    {
//                         if(ball_stop_flag)   motor_controller(-2300, 2000, 000, 3000, -1200, -3000,15000,-200);
//                       else
                         motor_controller(-2600, 2600, 10000, -3000, 1200, -3000,-6500,-1250);
                    }
                    else //其他情况反转
                    {
                         motor_controller(1800, -1800, 10000, -3000, 1200, -3000,-6500,-1250);
                    }

                    wushai_cnt = 0;
                    mis_cnt = 0;
                     ball_move_flag = 0;
                 ball_move_cnt = 0; 
                     R2_CONTROLLER.Ball_sta = BALL_OUTSIDE; 
                    
                }
                else if (R2_CONTROLLER.useful_ball_status == USEFULBALL_ENTER)  //进球口检测到有效球
                {
                    if(mis_cnt > 1000)          //误识别标志位
                         R2_CONTROLLER.useful_ball_status = NO_USEFULBALL;
                    else
                    {
                        //有效球进来的标志位赋值
                         R2_CONTROLLER.Ball_sta = BALL_HAVE_TAKEN; 
                        if(ball_move_flag)      //紫球已经离开了
                        {
                                               //摩擦带停转，继续爽筛
                            if (R2_CONTROLLER.filter_ball_state == LEFT_FILTER_BALL)        //左筛
                            {
                                motor_controller(00, 0, 10000, -2000, 1600, -3000,6500,-1300);
                            }
                            else if (R2_CONTROLLER.filter_ball_state == RIGHT_FILTER_BALL)  //右筛
                            {
                                 motor_controller(0, 0, 10000, -2000, 1600, -3000,-6500,-1300);
                            }
                            else //其他情况视为左筛
                            {
                                //摩擦带，摩擦带，鱼线电机，下面5065，中间5065，上面3508，筛球，下面3508屁股
                                 motor_controller(0, 0, 10000, -2000, 1600, -3000,-6500,-1300);
                            }
                          mis_cnt++;
                        }
                        else            //紫球还没离开，继续筛球
                        {
                            
                             if (R2_CONTROLLER.filter_ball_state == LEFT_FILTER_BALL)        //左筛
                                  motor_controller(-800, 800, 000, -3000, 1200, -3000,6500,-500);
                             else if (R2_CONTROLLER.filter_ball_state == RIGHT_FILTER_BALL)  //右筛
                                  motor_controller(-800, 800, 000, -3000, 1200, -3000,-6500,-500);      
                             else 
                                  motor_controller(-800, 800, 000, -3000, 1200, -3000,-6500,-500);     
                        }
                    }

                }
                else if(R2_CONTROLLER.useful_ball_status == USEFULBALL_MIDDLE)    //出球口检测到有效球
                {
              
                    if(wushai_cnt > 1500)        //误筛，切换状态，重新吸球
                             R2_CONTROLLER.useful_ball_status = NO_USEFULBALL;
                    else       //还没误筛
                    {    
                         //有效球进来的标志位赋值
                         R2_CONTROLLER.Ball_sta = BALL_HAVE_TAKEN; 
                         if(ball_move_flag) //紫球已经离开了
                         {  
                             //直接上去,但筛球的那个滚轮不变
                            if (R2_CONTROLLER.filter_ball_state == LEFT_FILTER_BALL)        //左筛
                            {
                                motor_controller(1200, -1200, 10000, -3000, 4000, 2500,6500,-400);
                            }
                            else if (R2_CONTROLLER.filter_ball_state == RIGHT_FILTER_BALL)  //右筛
                            {
                                motor_controller(1200, -1200, 10000, -3000, 4000, 2500,-6500,-400);
                            }
                            else //其他情况视为左筛
                            {
                                motor_controller(1200, -1200, 10000, -3000, 4000, 2500,-6500,-400);
                            }
                                 //出球口检测到有效球后，上面那光电门没有检测到，计数器计数，计数到达一定值后证明误筛了
                              wushai_cnt++;
                        }
                         else       //紫球还没离开，蓝球已经进去下面了
                         {
                             //蓝球不准上去
                             if (R2_CONTROLLER.filter_ball_state == LEFT_FILTER_BALL)        //左筛
                                  motor_controller(800, -800, 000, -000, 1200, -3000,6500,-300);
                             else if (R2_CONTROLLER.filter_ball_state == RIGHT_FILTER_BALL)  //右筛
                                  motor_controller(800, -800, 000, -000, 1200, -3000,-6500,-300);      
                             else 
                                  motor_controller(800, -800, 000, -000, 1200, -3000,-6500,-300);  
                         }
                    }  
                }
            }    
        }
}

/*
// 吸球
void take_ball()
{
    // New Controller
    switch (R2_CONTROLLER.Take_ball_state)
    {
    case NORMAL_TAKE_BALL: // 正常吸球

        if (finish_flag) // 放球成功之后，夹爪还没放下来
        {
            Claw_PutDown();
        }
        else // 放球完成标志位为0，
        {            
            // 球在车外
            if (R2_CONTROLLER.Gate_state == UP) // 最上面的光电门检测到
            {
                if (have_ball_inside_flag) // 球已经进来了
                {
                    Get_Ball();
                }
                else // 球还没进来,吸球
                {
                     MOTO_REAL_INFO[0].REAL_ANGLE = 0; // 消除累计误差 
                    if (middle_ball.ball_color == NO_BALL) // 中间识别不到球
                    {
                        // 左右两边都为无效球或没球
                        if ((enter_ball.ball_color == NO_BALL || enter_ball.ball_color == PURPLE_BALL) && (out_ball.ball_color == NO_BALL || out_ball.ball_color == PURPLE_BALL))
                        {
                            // 紫无
                            // 直接反转
                            flag_ball = 0;
                            motor_controller(1200, -1200, 0, 0, 0, 0);
                        }
                        else if (((enter_ball.ball_color == BLUE_BALL || enter_ball.ball_color == RED_BALL) && (out_ball.ball_color == NO_BALL)) || ((out_ball.ball_color == BLUE_BALL || out_ball.ball_color == RED_BALL) && (enter_ball.ball_color == NO_BALL)))
                        {
                            // 中间识别不到球，两边有一个识别到有效球，直接吸球
                            // 红无
                            flag_ball = 1;
                            motor_controller(-2200, 2200, 3000, 3000, 0, 0);
                        }
                        else if ((enter_ball.ball_color == BLUE_BALL || enter_ball.ball_color == RED_BALL) && (out_ball.ball_color == BLUE_BALL || out_ball.ball_color == RED_BALL))
                        {
                            // 红红
                            flag_ball = 2;
                            motor_controller(-2200, 2200, 3000, 3000, 0, 0);
                        }
                        else if ((enter_ball.ball_color == PURPLE_BALL) && (out_ball.ball_color == BLUE_BALL || out_ball.ball_color == RED_BALL))
                        {
                            // 紫红
                            flag_ball = 5;
                            motor_controller(1200, 1200, 0, 0, 0, 0);
                        }
                        else if ((enter_ball.ball_color == BLUE_BALL || enter_ball.ball_color == RED_BALL) && (out_ball.ball_color == PURPLE_BALL))
                        {
                            // 红紫
                            flag_ball = 6;
                            motor_controller(-1200, -1200, 0, 0, 0, 0);
                        }
                    }
                    else if (middle_ball.ball_color == RED_BALL || middle_ball.ball_color == BLUE_BALL) // 中间识别到球
                    {
                        flag_ball = 3;
                        // 摩擦轮正转，直接吸球
                        motor_controller(-2200, 2200, 3000, 3000, 0, 0);
                    }
                    else if (middle_ball.ball_color == PURPLE_BALL)
                    {
                        flag_ball = 4;
                        motor_controller(1200, -1200, 0, 0, 0, 0);
                    }
                }
            }
            else if (R2_CONTROLLER.Gate_state == BENEATH) // 球在车下面,直接吸到夹爪那
            {
                // 加速转动滚筒，吸球，摩擦轮不动
               // motor_controller(-1200, 1200, 4000, 5000, -2000, 3500);
                motor_controller(1200, -1200, 4000, 5000, -2000, 3500);
            }
            else if (R2_CONTROLLER.Gate_state == MIDDLE) // 球在中间
            {
                // 改变标志位
                have_ball_inside_flag = 1; // 中部检测到球说明里面有球
                // 上下位机标志位赋值
                R2_CONTROLLER.Ball_sta = BALL_HAVE_TAKEN;
                motor_controller(1200, -1200, 0, 00, -3000, 5200);
            }
        }

        break;
    case LEFT_TAKE_BALL:

        if (finish_flag) // 放球成功之后，夹爪还没放下来
        {
            Claw_PutDown();
        }
        else
        {
            if (R2_CONTROLLER.Gate_state == UP) // 球在车外
            {
                if (have_ball_inside_flag) // 球已经进来了
                {
                    Get_Ball();
                }
                else // 球还没进来，吸球
                {
                     MOTO_REAL_INFO[0].REAL_ANGLE = 0; // 消除累计误差 
                    // 无效球，左摩擦轮正转右摩擦轮反转，滚筒不动
                    if (middle_ball.ball_color == PURPLE_BALL)
                    {
                        motor_controller(-1200, -1200, 0, 0, 0, 0);
                    }
                    else if (middle_ball.ball_color == BLUE_BALL || middle_ball.ball_color == RED_BALL)
                    { // 有效球，摩擦带正转，滚筒也转
                        motor_controller(-2200, 2200, 3000, 3000, 0, 0);
                    }
                    else if(middle_ball.ball_color == NO_BALL)
                    {   //中间没球，左边为有效球，右边不是紫球
                         if(enter_ball.ball_color == BLUE_BALL || enter_ball.ball_color == RED_BALL)
                         {
                            if(out_ball.ball_color != PURPLE_BALL)        //正转
                            {
                                motor_controller(-2200, 2200, 3000, 3000, 0, 0);
                            }
                            else
                            {                       //左吸
                                motor_controller(-1200, -1200, 0, 0, 0, 0);
                            }
                         }
                         else
                         {
                                motor_controller(-1200, -1200, 0, 0, 0, 0);
                         }
                    }
                }
            }
            else if (R2_CONTROLLER.Gate_state == BENEATH) // 球在车下面,直接吸到夹爪那
            {
                // 加速转动滚筒，吸球，摩擦轮不动
              //  motor_controller(-1200, 1200, 4000, 5000, -2000, 3500);
                motor_controller(1200, -1200, 4000, 5000, -2000, 3500);
            }
            else if (R2_CONTROLLER.Gate_state == MIDDLE) // 球在中间
            {
                // 改变标志位
                have_ball_inside_flag = 1; // 中部检测到球说明里面有球
                // 上下位机标志位赋值
                R2_CONTROLLER.Ball_sta = BALL_HAVE_TAKEN;
                motor_controller(0, 0, 0, 00, -3000, 5200);
            }
        }
        break;
    case RIGHT_TAKE_BALL:

        if (finish_flag) // 放球成功之后，夹爪还没放下来
        {
            Claw_PutDown();
        }
        else
        {
            if (R2_CONTROLLER.Gate_state == UP) // 球在车外
            {
                if (have_ball_inside_flag) // 球已经进来了
                {
                    Get_Ball();
                }
                else // 球还没进来
                {
                     MOTO_REAL_INFO[0].REAL_ANGLE = 0; // 消除累计误差 
                    // 无效球，右摩擦轮正转左摩擦轮反转，滚筒不动
                    if ( middle_ball.ball_color == PURPLE_BALL)
                    {
                        motor_controller(1200, 1200, 0, 0, 0, 0);
                    }
                    else if (middle_ball.ball_color == BLUE_BALL || middle_ball.ball_color == RED_BALL)
                    { // 有效球，摩擦带正转，滚筒也转
                        motor_controller(-2200, 2200, 3000, 3000, 0, 0);
                    }
                    else if(middle_ball.ball_color == NO_BALL)
                    {   //中间没球，右边为有效球，左边不是紫球
                         if(out_ball.ball_color == BLUE_BALL || out_ball.ball_color == RED_BALL)
                         {
                            if(enter_ball.ball_color != PURPLE_BALL)
                            {
                                motor_controller(-2200, 2200, 3000, 3000, 0, 0);
                            }
                            else
                            {
                                motor_controller(1200, 1200, 0, 0, 0, 0);
                            }
                         }
                         else
                         {
                                motor_controller(1200, 1200, 0, 0, 0, 0);
                         }
                    }
                }
            }
            else if (R2_CONTROLLER.Gate_state == BENEATH)
            {
                // 加速转动滚筒，吸球，摩擦轮不动
             //   motor_controller(-1200, 1200, 4000, 5000, -2000, 3500);
                motor_controller(1200, -1200, 4000, 5000, -2000, 3500);
            }
            else if (R2_CONTROLLER.Gate_state == MIDDLE)
            {
                // 改变标志位
                have_ball_inside_flag = 1; // 中部检测到球说明里面有球
                R2_CONTROLLER.Ball_sta = BALL_HAVE_TAKEN;
                motor_controller(0, 0, 0, 00, -3000, 5200);
            }
        }
        break;
    default: // 错误状态，关闭所有控制器
        motor_controller(0, 0, 0, 0, 0, 0);
        Speed_Control(&MOTO_REAL_INFO[0], 0);
        Speed_Control(&MOTO_REAL_INFO[1], 0);
        break;
    }
}               */

/**
 * @brief 底盘控制器
 */
void Chassis_Controller(void)
{
    if (ROBOT_CHASSI.Chassis_Controller_Flag == 1)
    {
        //     Free_Control();     //手柄端的底盘控制,单片机
        ROBOT_CHASSI.SPEED.Robot_VX = R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_X;
        ROBOT_CHASSI.SPEED.Robot_VY = R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_Y;
        ROBOT_CHASSI.WORLD.World_W = R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_W;
    }
    else
    {
        ROBOT_CHASSI.WORLD.World_X = 0;
        ROBOT_CHASSI.WORLD.World_Y = 0;
        ROBOT_CHASSI.WORLD.World_W = 0;
    }
    Robot_Wheels_RPM_calculate();
}

/**
 * @brief 使用上位机控制机构
 */
void ROS_Control(void)
{
    if (R2_CONTROLLER.ros_stm32_noconnected_flag == 0) // 防止串口硬件连接断开，超时200ms     还未断连
    {
        // 上层机构控制器
        R2_CONTROLLER.NOW_CONTROLLER_STATE = upper_controller(R2_CONTROLLER.NEXT_CONTROLLER_STATE);
    }
    else
    { // 断连则关闭所有控制器
        R2_CONTROLLER.NOW_CONTROLLER_STATE = upper_controller(CONTROLLER_OFF);
        ROBOT_CHASSI.Chassis_Controller_Flag = 0;
    }
}

