#include "main.h"

//定时器2获取时钟初始化
void TIM2_GET_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);    //使能TIM2时钟
    
    TIM_TimeBaseInitStructure.TIM_Period = 10000; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler= 84-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;     //不分频
    
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);    //初始化TIM2
    
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);    //使能定时器2更新中断
    TIM_Cmd(TIM2,ENABLE);     //使能定时器2
    
    NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//计时函数
uint16_t Time_Sys[4] = {0};   //记录四个状态，微秒，秒，分钟，小时
uint16_t Microsecond_Cnt = 0;    //微秒计时器
uint16_t TIME_ISR_CNT = 0,LAST_TIME_ISR_CNT = 0;
void TIM2_IRQHandler(void)   //10ms
{
      if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
      {
        LAST_TIME_ISR_CNT = TIME_ISR_CNT;
        TIME_ISR_CNT++;
        Microsecond_Cnt++; 
        if(Microsecond_Cnt >= 100)
        {
            Microsecond_Cnt++;
            Time_Sys[Second]++;
            if(Time_Sys[Second] >= 60)
            {
                Time_Sys[Second] = 0;
                Time_Sys[Minute]++;
                if(Time_Sys[Minute] >= 600)
                {
                     Time_Sys[Minute] = 0;
                     Time_Sys[Hour]++;
                }              
            }    
        }
        Time_Sys[MicroSecond] = Microsecond_Cnt;
        TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
      }
}

//PPM外部中断PF7初始化
void PPM_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	//开启相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource7);
	
	//GPIO设置-下拉输入
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_IN;     //输入模式
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_DOWN;   //下拉输入
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	//EXIT设置-中断模式
	EXTI_InitStruct.EXTI_Line 	 = EXTI_Line7;//PF7
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode 	 = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;//触发方式，上升沿触发
	EXTI_Init(&EXTI_InitStruct);
	
	//NVIC设置-中断配置
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断0
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;//最高优先级 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

static uint16_t PPM_buf[10] = {0};
uint16_t PPM_Databuf[10] = {0};
uint8_t ppm_update_flag = 0;
uint32_t now_ppm_time_send = 0;

void EXTI9_5_IRQHandler(void)
{
    static uint32_t last_ppm_time = 0, now_ppm_time = 0;
    static uint8_t ppm_ready = 0, ppm_sample_cnt = 0;
    static uint16_t ppm_time_delta = 0;   //得到上升沿与下降沿的时间
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)//通道7
    {
        //系统运行时间获取，单位us
        last_ppm_time = now_ppm_time;  //获取上一次的当前时间作为上次时间
        now_ppm_time_send = now_ppm_time = 10000*TIME_ISR_CNT + TIM2->CNT;    //us
        ppm_time_delta = now_ppm_time - last_ppm_time;     //相减得到一个周期时间
        //PPM解析开始
        if(ppm_ready == 1)   //判断帧结束时，开始解析新的一轮PPM
        {
          if(ppm_time_delta >= 2200)  //帧结束电平至少2ms = 2000us,由于部分老版本遥控器、接收器输出PPM信号不标准，当出现解析异常时，尝试该小此值，该情况仅出现一例：使用天地飞老版本遥控器
          {
            ppm_ready = 1;
            ppm_sample_cnt = 0;   //对应的通道计数值
            ppm_update_flag = 1;  //接收更新标志
          }
          else if(ppm_time_delta >=950 && ppm_time_delta <= 2050)   //单个PWM脉宽在1000-2000us，这里设定900-2100，应该是为了提升容错
          {
                PPM_buf[ppm_sample_cnt++] = ppm_time_delta;   //对应通道写入缓冲区
                if(ppm_sample_cnt >=8  )  //单次解析结束0-7表示8个通道。这里应该可以显示10个通道，但实测只能8个
                {
                    memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));  //将接收到的数据复制给另一个数组
                    ppm_sample_cnt=0;    //成功接收完8个通道的数据后，通道计数值清零，等待下次接收                
                }
          }
          else
              ppm_ready = 0;     //脉宽数据错误停止接收          
        }
        else if(ppm_time_delta >= 2200)   //帧结束电平至少2ms = 2000us,或者为第一次接收开始 
        {
            ppm_ready = 1;
            ppm_sample_cnt = 0;
            ppm_update_flag = 0;            
        }
    }
    EXTI_ClearITPendingBit(EXTI_Line7);   //清除LINE7的中断标志位
}

