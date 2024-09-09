#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#define START 0x11
#include "stdint.h"
enum FLAG{
	CHECK_HEADER_FLAG, 
	RECEIVE_DATA_FLAG, 
	CHECK_VALUE_FLAG, 
	FINAL_RECEIVE_FLAG
};
typedef struct STIR_BUTTON
{
    uint8_t start_flag;
    uint8_t red_blue_flag;
    uint8_t restart_flag;
    uint8_t if_auto_flag;
}STIR_BUTTON;
extern STIR_BUTTON Button;

unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len);
void USART_Send_String(unsigned char *p, short sendSize, USART_TypeDef *usart);
int STM32_READ_FROM_ROS(unsigned char *next_controller_state,unsigned char *take_ball_flag,float *yaw,unsigned char *clean_ball_flag);
void Usart_Send_Data(uint8_t now_controller_state,uint8_t ball_state,uint8_t arrive_flag,uint8_t lock_flag,uint8_t tough_flag,
 uint8_t start_flag,uint8_t red_blue_flag,uint8_t restart_flag,uint8_t if_auto_flag,uint8_t test_cnt);

#endif
