#ifndef __ACTION_H
#define __ACTION_H

#define PI 3.1415926f
//��λ��װ���   ����λ��mm��
#define INSTALL_ERROR_X  0.0f
#define INSTALL_ERROR_Y  0.0f

//����ȫ����λ
typedef struct ACTION_GL_POS
{
    float ANGLE_Z;     //ƫ����
    float ANGLE_Y;     //������   (�ò���)
    float ANGLE_X;     //������   (�ò���)
    float POS_X;       //X����
    float POS_Y;       //Y����
    float W_Z;         //ƫ�����ٶ�
    
    float LAST_POS_X;  //��һ��x����λ��
    float LAST_POS_Y;  //��һ��y����λ��
    
    float DELTA_POS_X; //xƫ��
    float DELTA_POS_Y; //yƫ��
    
    float REAL_X;      //��ʵx����λ�� 
    float REAL_Y;      //��ʵy����λ��

}ACTION_GL_POS;
//��������������յ�������
extern ACTION_GL_POS ACTION_GL_POS_DATA;
extern float pos_x;
extern float pos_y;
extern float zangle;

void Action_Init(void);
void Update_Angle_gl_position(float value[6]);
#endif
