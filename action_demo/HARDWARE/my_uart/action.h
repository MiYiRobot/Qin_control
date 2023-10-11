#include "my_uart.h"
#ifndef __ACTION_H
#define __ACTION_H
#include "stm32f4xx.h"                  // Device header

#define PI 3.1415926f
//定位安装误差    （单位：mm）
#define INSTALL_ERROR_X    0.0f
#define INSTALL_ERROR_Y    0.0f

//东大全场定位
typedef struct ACTION_GL_POS
{
  float ANGLE_Z;   //偏航角
  float ANGLE_Y;   //俯仰角（用不上）
  float ANGLE_X;   //翻滚角（用不上）
  float POS_X;
  float POS_Y;
  float W_Z;      //偏航角速度
    
  float LAST_POS_X;   //上一刻x坐标位置
  float LAST_POS_Y;   //上一刻y坐标位置
    
  float DELTA_POS_X;  //x偏差
  float DELTA_POS_Y;  //y偏差
    
  float REAL_X;  //真实x坐标位置
  float REAL_Y;  //真实y坐标位置
    
}ACTION_GL_POS;

extern ACTION_GL_POS ANCTION_GL_POS_DATA;
extern int action_update;
extern float pos_x;
extern float pos_y;
extern float zangle;

void Action_Init(void);
void Update_Angle_gl_position(float value[6]);
void Update_J(float New_J);
void Update_X(float New_X);
void Update_Y(float New_Y);

#endif
