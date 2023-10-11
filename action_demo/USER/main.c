#include "all.h"
 int count1 = 0;
int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);
	uart_init(115200);
	Action_Init();   //初始化串口和数据
   
  while(1)
	{
		delay_ms(100);
		//printf("Yaw:%f\tX:%f\tY:%f\r\n", zangle, pos_x, pos_y);
		count1++;
        if(count1>50)   count1=0;
	}
}
