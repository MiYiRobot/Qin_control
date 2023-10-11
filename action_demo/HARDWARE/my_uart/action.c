#include "all.h"
#include <math.h>

float pos_x;
float pos_y;
float zangle;
int action_update = 0;     //ȫ����λ������    
//��������洢����λ��
ACTION_GL_POS ACTION_GL_POS_DATA = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//�жϺ�����ȡ���������͹�������Ϣ
void UART4_IRQHandler(void)
{
      static uint8_t ch;
      static union{    //�����壺�����������һ����ַ      
         uint8_t data[24];
         float ActVal[6];
      }posture;
      
      static uint8_t count = 0;
      static uint8_t i = 0;
     
      if(USART_GetITStatus(UART4,USART_IT_RXNE) == SET)  //�ɹ����յ�һ������
      {
          ch = USART_ReceiveData(UART4);
          switch(count)
          {
              case 0:
              {
                if(ch == 0x0d)    //������յ��س��������յ�0x0dʱ���������������ӣ��ȴ�0x0a
                    count++;  //������һ��
                else
                    count = 0;     //�´����¿�ʼ
              }    
               break;
              case 1:
              {
                 if(ch == 0x0a)     //����,������ڽ��յ�0x0aʱ��ȷ�����յ����ݣ�������һ�ν���
                 {
                    i = 0;
                    count++;    //������һ��
                 }
                 else if(ch== 0x0d)     //�س�  
                     ;      //ch���ǵ��ڻس����´λ��Ǵ����￪ʼ
                 else
                     count = 0;      //��ͷ��ʼ
              }
                 break;
              case 2:
              {
                 posture.data[i] = ch;     //ǰ��i = 0,��ʾ���յ���һ������     
                  i++;      
                  if(i >= 24)    //������յ������ݴ���24��������i
                  {
                    i = 0;      //�Ѿ�������24�����ݣ��洢��posture��
                    count++;   //������һ��
                  }
              }
              break;
              case 3:      //��ʼ�����ж��Ƿ񻹽��յ����ݣ����յ��͸�������
              {
                   if(ch == 0x0a)   //������յ��س�
                   {
                      count++;     //������һ��
                   }
                   else
                       count = 0;   //�����쳣�����¿�ʼ           
              }
              break;
              case 4:
              {
                   if(ch == 0x0d)    //������յ�����
                   {
                     Update_Angle_gl_position(posture.ActVal);   //��������
                   }
                    count = 0;    //���¿�ʼ������һ�ν���
              }
                break;
              default:
                  count = 0;   //�����쳣�����¿�ʼ
                  break;
          }
         USART_ClearITPendingBit(UART4, USART_IT_RXNE);   //�������˵�Ȼ���������ɱ�־λ
      }
}


//����actionȫ����λ��ֵ
void Update_Angle_gl_position(float value[6])
{
        //�洢��һ�ε�ֵ
     ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
     ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
    
      //��¼�˴θ��µ�ֵ
     ACTION_GL_POS_DATA.ANGLE_Z = value[0];   //ƫ����  
     ACTION_GL_POS_DATA.ANGLE_Y = value[1];   //������
     ACTION_GL_POS_DATA.ANGLE_X = value[2];   //������
     ACTION_GL_POS_DATA.POS_X = value[3];     //����X
     ACTION_GL_POS_DATA.POS_Y = value[4];     //����Y 
     ACTION_GL_POS_DATA.W_Z = value[5];       //��ʻ���ٶ�

    //������㣬����ƫ��
    ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;   //�˴����� - �ϴ�����
    ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
    
     //���¼��������λ��
    ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);   //����������װλ�����ͼ����ϵ�����Ƿ�һ�£���һ�µĻ��Ӹ���
    ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);   //��ʵλ�� = �ϴ�λ����ʵ + λ��ƫ��
    
    //ƫ����ֱ�Ӹ�ֵ
    zangle = -(ACTION_GL_POS_DATA.ANGLE_Z);     //����������װλ��ȷ������
    //�任����������λ��  
    pos_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * sin(zangle * PI / 180);
    pos_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * cos(zangle * PI / 180);       
}




//��ʼ��action
void Action_Init(void)
{
   //��ʼ������
    uart4_init(115200);
    
    //��ʼ������
    ACTION_GL_POS_DATA.REAL_X= 0.001f;
    ACTION_GL_POS_DATA.REAL_Y= 0.001f;
}

//------------------�����ǹٷ��ĸ��±�Ǵ��룬�ò���-----------------------------------

void Update_J(float New_J)
{
 int i =0;
 static union
 {
  float J;
  char data[4];
 }New_set;
 
 New_set.J = New_J;
 
 UART_SendString(UART4, "ACTJ");
 for(i=0;i<4;i++)
 {
  USART_SendData(UART4,New_set.data[i]); 
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC) == 0);     //�ȴ����ݷ��ͳɹ�  
 } 
 
 USART_ClearFlag(UART4,USART_FLAG_TC);          //�����ַ�ǰ��ձ�־λ������ȱʧ�ַ����ĵ�һ���ַ���   
 
}

void Update_X(float New_X)      //���µ���ACTION_GL_POS_DATA.REAL_X��û�л��㵽ȫ������
{
 int i =0;
 static union
 {
  float X;
  char data[4];
 }New_set;
 
 New_set.X = New_X;
 
 UART_SendString(UART4, "ACTX");
 for(i=0;i<4;i++)
 {
  USART_SendData(UART4,New_set.data[i]); 
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC) == 0);     //�ȴ����ݷ��ͳɹ�  
 } 
 
 USART_ClearFlag(UART4,USART_FLAG_TC);          //�����ַ�ǰ��ձ�־λ������ȱʧ�ַ����ĵ�һ���ַ���   
 
}

void Update_Y(float New_Y)      //���µ���ACTION_GL_POS_DATA.REAL_Y��û�л��㵽ȫ������
{
 int i =0;
 static union
 {
  float Y;
  char data[4];
 }New_set;
 
 New_set.Y = New_Y;
 
 UART_SendString(UART4, "ACTY");
 for(i=0;i<4;i++)
 {
  USART_SendData(UART4,New_set.data[i]); 
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC) == 0);     //�ȴ����ݷ��ͳɹ�  
 } 
 
 USART_ClearFlag(UART4,USART_FLAG_TC);          //�����ַ�ǰ��ձ�־λ������ȱʧ�ַ����ĵ�һ���ַ���   
}




