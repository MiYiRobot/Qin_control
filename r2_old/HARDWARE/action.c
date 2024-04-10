#include "main.h"


float pos_x;
float pos_y;
float zangle;
//  定义变量存储坐标位置
ACTION_GL_POS ACTION_GL_POS_DATA = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//中断函数获取传感器发送过来的消息
void UART4_IRQHandler(void)
{
      static uint8_t ch;
      static union
      {    //联合体：多个变量共用一个地址      
         uint8_t data[24];
         float ActVal[6];
      }posture;
      
      static uint8_t count = 0;
      static uint8_t i = 0;
     
      if(USART_GetITStatus(UART4,USART_IT_RXNE) == SET)  //成功接收到一组数据
      {
          ch = USART_ReceiveData(UART4);
          switch(count)
          {
              case 0:
              {
                if(ch == 0x0d)    //当接收到0x0d时，计数器不再增加，等待0x0a
                    count++;  //进入下一个
                else
                    count = 0;     //下次重新开始
              }    
               break;
              case 1:
              {
                 if(ch == 0x0a)     //如果串口接收到0x0a时，确定接收到数据，开启下一次接收
                 {
                    i = 0;
                    count++;    //进入下一个
                 }
                 else if(ch== 0x0d)      
                     ;      //ch还是等于0d，下次还是从这里开始
                 else
                     count = 0;      //从头开始
              }
                 break;
              case 2:
              {
                 posture.data[i] = ch;     //前面i = 0,表示接收到第一个数据     
                  i++;      
                  if(i >= 24)    //如果接收到的数据大于24，则重置i
                  {
                    i = 0;      //已经接收了24个数据，存储到posture中
                    count++;   //进入下一个
                  }
              }
              break;
              case 3:      //开始重新判断是否还接收到数据，接收到就更新数据
              {
                   if(ch == 0x0a)   //如果接收到0a
                   {
                      count++;     //进入下一个
                   }
                   else
                       count = 0;   //接收异常，重新开始           
              }
              break;
              case 4:
              {
                   if(ch == 0x0d)    //如果接收到0d
                   {
                     Update_Angle_gl_position(posture.ActVal);   //更新数据
                   }
                    count = 0;    //重新开始，等下一次接收
              }
                break;
              default:
                  count = 0;   //接送异常则重新开始
                  break;
          }
         USART_ClearITPendingBit(UART4, USART_IT_RXNE);   //接收完了当然清除接收完成标志位
      }
}


//更新action全场定位的值
void Update_Angle_gl_position(float value[6])
{
        //存储上一次的值
     ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
     ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
    
      //记录此次更新的值
     ACTION_GL_POS_DATA.ANGLE_Z = value[0];   //偏航角  
     ACTION_GL_POS_DATA.ANGLE_Y = value[1];   //俯仰角
     ACTION_GL_POS_DATA.ANGLE_X = value[2];   //翻滚角
    ACTION_GL_POS_DATA.POS_X = value[3]/1000;     //坐标X    单位：米 
     ACTION_GL_POS_DATA.POS_Y = value[4]/1000;     //坐标Y   单位：米
     ACTION_GL_POS_DATA.W_Z = value[5];       //行驶角速度

    //差分运算，计算偏差
    ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;   //此次坐标 - 上次坐标
    ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
    
     //更新计算出最终位置
    ACTION_GL_POS_DATA.REAL_X += -(ACTION_GL_POS_DATA.DELTA_POS_X);   //看传感器安装位置与地图坐标系方向是否一致，不一致的话加负号
    ACTION_GL_POS_DATA.REAL_Y += -(ACTION_GL_POS_DATA.DELTA_POS_Y);   //真实位置 = 上次位置真实 + 位置偏差
    
    //偏航角直接赋值
    zangle = -(ACTION_GL_POS_DATA.ANGLE_Z);     //看传感器安装位置确定正负
    //变换到底盘中心位置  
    pos_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * sin(zangle * PI / 180);
    pos_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * cos(zangle * PI / 180);       
}

//初始化action
void Action_Init(void)
{
   //初始化串口
    uart4_init(115200);
    
    //初始化坐标
    ACTION_GL_POS_DATA.REAL_X= 0.001f;
    ACTION_GL_POS_DATA.REAL_Y= 0.001f;
}


