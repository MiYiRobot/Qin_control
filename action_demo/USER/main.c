#include "all.h"
 int count1 = 0;
int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);
	uart_init(115200);
	Action_Init();   //��ʼ�����ں�����
   
  while(1)
	{
		delay_ms(100);
		//printf("Yaw:%f\tX:%f\tY:%f\r\n", zangle, pos_x, pos_y);
		count1++;
        if(count1>50)   count1=0;
	}
}
