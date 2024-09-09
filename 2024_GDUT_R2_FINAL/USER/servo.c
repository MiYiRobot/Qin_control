#include "main.h"
#include "tim.h"
//设置角度
//void Servo_SetAngle(float Angle)
//{
//    float compare =Angle/180*2000 + 500;
//    __HAL_TIM_SET_COMPARE(&htim13,TIM_CHANNEL_1,compare);
//        
//}

//float YAW_angle =0;
//void PWM_SERVO()
//{
//    YAW_angle = (81 - R2_CONTROLLER.robot_yaw);
//    Servo_SetAngle(YAW_angle);
//}


unsigned char send_servo_buf[2+4] = {0};
unsigned char date1,date2,date3;//分别发送舵机的角度
uint16_t servo_flag = 0;//0雨刮器不动，1雨刮器动

void UART_Servo(unsigned char servonum,unsigned char angle)
{
  servonum = 64 + servonum;
    date1 = angle/100 + 48;
    date2 = (angle%100)/10 + 48;
    date3 = angle%10 + 48;
    send_servo_buf[0] = 0x24;
    send_servo_buf[1] = servonum;
    send_servo_buf[2] = date1;
    send_servo_buf[3] = date2;
    send_servo_buf[4] = date3;
    send_servo_buf[5] = 0x23;
    USART3_Send_String(send_servo_buf, sizeof(send_servo_buf), USART3);
   // HAL_UART_AbortTransmit(&htim2)
//    HAL_UART_Transmit(&huart1,&send_servo_buf[0],1,0xffff);
//    HAL_UART_Transmit(&huart1,&send_servo_buf[1],1,0xffff);
//    HAL_UART_Transmit(&huart1,&send_servo_buf[2],1,0xffff);
//    HAL_UART_Transmit(&huart1,&send_servo_buf[3],1,0xffff);
//    HAL_UART_Transmit(&huart1,&send_servo_buf[4],1,0xffff);
//    HAL_UART_Transmit(&huart1,&send_servo_buf[5],1,0xffff);
 //   HAL_Delay(100);
}


float yuguaangle = 0;

void servo_init(){
//    //摄像头角度
    UART_Servo(1,0);
    //左雨刮
    UART_Servo(3,0);
    //右雨刮
    UART_Servo(5,180);
//    Servo_SetAngle(90);
    UART_Servo(7,180);
    servo_flag = 0;
    
}

uint16_t servo_cnt = 0;
float angle1 = 179;
float angle3 = 179;
float angle = 0;
//1吸球侧舵机，2放球侧舵机，3左雨刮，4右雨刮
void Servo_Control(void)
{
    //雷达坐标 90°到 -90°
    
    angle = (90 - R2_CONTROLLER.robot_yaw);
    //中间为90°，左右两边摇分别为0°和180°

    
     UART_Servo(1,angle);
    UART_Servo(3,angle);
    
    if(R2_CONTROLLER.clean_ball_flag == 1){
//        if(yuguaangle >= 170){
//            yuguaangle = 0;
//            servo_flag = 0;
//            //反馈雨刮器工作完成
//        }
        //yuguaangle = 180;
        if(servo_cnt >= 30){
            yuguaangle = 0;
            if(servo_cnt >=60){
                servo_cnt = 0;
            }
        }else{
            yuguaangle = 180;
        }
        servo_cnt++;
    }
    if(R2_CONTROLLER.clean_ball_flag == 0){
        yuguaangle = 0;
        servo_cnt = 0;
    }
    UART_Servo(5,yuguaangle);
    UART_Servo(7,(180-yuguaangle));
   
    
}



void USART3_Send_String(unsigned char *p_array, short sendSize, USART_TypeDef *usart)
{ 
	static int length =0;
	while(length<sendSize)
	{  
		while( !(usart->SR&(0x01<<7)) );//发送缓冲区为空
		usart->DR=*p_array;                   
		p_array++;
		length++;
	}
	length =0;
}

	
//串口发送一个字节




