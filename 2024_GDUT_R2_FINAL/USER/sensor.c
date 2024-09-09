#include "sensor.h"
#include "main.h"
#define GW_GRAY_SERIAL_GPIO_GROUP GPIOB
#define GW_GRAY_SERIAL_GPIO_CLK GPIO_PIN_8 //PB8 ??CLK
#define GW_GRAY_SERIAL_GPIO_DAT GPIO_PIN_9 //PB9 ??DAT
#define GW_GRAY_SERIAL_DELAY_VALUE 500 

//颜色传感器宏定义
uint8_t read_value[3];
COLOR_HSL left_hsl = {0};
COLOR_HSL right_hsl = {0};
COLOR_HSL middle_hsl = {0};

COLOR_RGBC rgb;
COLOR_HSL  hsl1;
COLOR_HSL  hsl2;
Sepan_RGBC rgb_255A;
COLOR_RGBC tcs34725Data1;//没管原先的，另外定义了一个结构
COLOR_RGBC tcs34725Data2;//没管原先的，另外定义了一个结构
uint16_t tcsouttime1 = 0;
uint16_t tcsouttime2 = 0;
uint32_t uwtick;
uint8_t ballflag = 0;//控制挡板的延时开关
#define throw_ball() {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);HAL_Delay(1000);}

 uint8_t sensor_data;
 uint8_t sensor[8];

COLOR_RECOGNITION enter_ball;
COLOR_RECOGNITION out_ball;
COLOR_RECOGNITION middle_ball;

 void delay(uint32_t delay_count)
{
	for (int i = 0; i < delay_count; ++i) {
		__NOP();
	}
}

uint8_t gw_gray_serial_read()
{
	uint8_t ret = 0;

	for (int i = 0; i < 8; ++i) {

		HAL_GPIO_WritePin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_CLK, GPIO_PIN_SET);

		ret |= HAL_GPIO_ReadPin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_DAT) << i;

		HAL_GPIO_WritePin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_CLK, GPIO_PIN_RESET);

		delay(GW_GRAY_SERIAL_DELAY_VALUE);
	}

	return ret;
}

              /*
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
*/
//I2C_HandleTypeDef *I2cx;
//void SwicthI2c(char sw)
//{
//	if(sw==2)I2cx=&hi2c2;
//	else I2cx=&hi2c1;
//}
// 
//void I2C1_Reset()
//{
//  HAL_I2C_MspDeInit(&hi2c1);
//  hi2c1.State = HAL_I2C_STATE_RESET;
//  MX_I2C1_Init();
//}

//void I2C2_Reset()
//{
//  HAL_I2C_MspDeInit(&hi2c2);
//  hi2c2.State = HAL_I2C_STATE_RESET;
//  MX_I2C2_Init();
//}


//使用前需要初始化捏
//TCS34725_Init();
//void tcs34725_1_work()
//{
//    SwicthI2c(1);
//	TCS34725_GetRawData(&tcs34725Data1);
////    if(tcsouttime1 >=2000){
////        I2C1_Reset();
////    }
////    tcsouttime1 = 0;
//    RGBtoHSL(&tcs34725Data1,&hsl1);
//}

//void tcs34725_2_work()
//{
//  SwicthI2c(2);
//	TCS34725_GetRawData(&tcs34725Data2);
////    if(tcsouttime1 >=2000){
////        I2C2_Reset();
////    }
////    tcsouttime2 = 0;
//  RGBtoHSL(&tcs34725Data2,&hsl2);
//}

void ball_recognition(COLOR_HSL* front_hsl,COLOR_HSL* back_hsl)
{
	//左或右扫到紫球，视为紫球
	if(front_hsl->h >= 250|| back_hsl->h >= 250)
    {
	//R2_CONTROLLER.ball_color = PURPLE_BALL;
    }else if((front_hsl->h <= 230 && front_hsl->h >=160) && (back_hsl->h <= 230 && back_hsl->h >=160))
   {
		 //左或右扫到蓝球，视为蓝球
	//R2_CONTROLLER.ball_color= BLUE_BALL;
   }else if((front_hsl->h <= 20 && front_hsl->h >1)&& (back_hsl->h <= 20 && back_hsl->h >1))//视为红球
  {//左或右扫到红球，
//      if(front_hsl->s > 25)
//                R2_CONTROLLER.ball_color = RED_BALL;
//      else if(front_hsl->s > 0 && front_hsl->s <= 20)
//                R2_CONTROLLER.ball_color = PURPLE_BALL; 
	//	R2_CONTROLLER.ball_color= RED_BALL;
  }

}

//void tcs34725_work()
//{
//	tcs34725_1_work();
//	HAL_Delay(3);
//	tcs34725_2_work();
//	ball_recognition(&hsl1,&hsl2);
//}

void yanse_init()
{
    uint8_t SetMode = HSL_order;
	/* 打开模拟值模式, 并且读取模拟数值, 后面可以直接读取 */
	//sw_i2c_mem_read(&i2c1_interface, Write_Add, HSL_order, read_value, 3);
    sw_i2c_write(&i2c1_interface,Write_Add,&SetMode,1);
    sw_i2c_write(&i2c2_interface,Write_Add,&SetMode,1);
    sw_i2c_write(&i2c3_interface,Write_Add,&SetMode,1);
}


void yanse_read1()
{
     sw_i2c_mem_read(&i2c1_interface, Write_Add, HSL_order, read_value, 3);
    //   sw_i2c_read(&i2c1_interface,Read_Add, read_value, 3);
       left_hsl.h = read_value[0];
       left_hsl.s = read_value[1];
       left_hsl.l = read_value[2];
	
}

void yanse_read2()
{
        sw_i2c_mem_read(&i2c2_interface, Write_Add, HSL_order, read_value, 3);
       right_hsl.h = read_value[0];
       right_hsl.s = read_value[1];
       right_hsl.l = read_value[2];
}
uint8_t value[100];
void yanse_read()
{
     sw_i2c_mem_read(&i2c1_interface, Write_Add, HSL_order, read_value, 3);
    //   sw_i2c_read(&i2c1_interface,Read_Add, read_value, 3);
       left_hsl.h = read_value[0];
       left_hsl.s = read_value[1];
       left_hsl.l = read_value[2];
     sw_i2c_mem_read(&i2c2_interface, Write_Add, HSL_order, read_value, 3);
       right_hsl.h = read_value[0];
       right_hsl.s = read_value[1];
       right_hsl.l = read_value[2];
      sw_i2c_mem_read(&i2c3_interface, Write_Add, HSL_order, read_value, 3);
       middle_hsl.h = read_value[0];
       middle_hsl.s = read_value[1];
       middle_hsl.l = read_value[2];
   
   	YanSe_Judge(&out_ball,&left_hsl);
   	YanSe_Judge(&enter_ball,&right_hsl);
   	YanSe_Judge(&middle_ball,&middle_hsl);
}

//颜色判断 
void YanSe_Judge(COLOR_RECOGNITION *ballColor,COLOR_HSL *ball_hsl)
{
		//连续3帧传的球数据正常
		if (ball_hsl->h >=230 && ball_hsl->h <= 250)    //红球
		{
			ballColor->red_cnt++;
			ballColor->blue_cnt = 0;
			ballColor->purple_cnt = 0;
			ballColor->noball_cnt=0;	
		}
		else if (ball_hsl->h >= 135 && ball_hsl->h <= 160)      //蓝球
		{
//            if(ball_hsl->s >=10 && ball_hsl->s <= 235)
//            {
                ballColor->blue_cnt++;
                ballColor->red_cnt = 0;
                ballColor->purple_cnt = 0;
                ballColor->noball_cnt=0;
//            } 
//            else            //防止误测
//            {
//                ballColor->red_cnt = 0;
//                ballColor->blue_cnt = 0;
//                ballColor->purple_cnt = 0;
//                ballColor->noball_cnt++;
//            }
			
		}
		else if (ball_hsl->h >= 170 && ball_hsl->h <= 220)  //紫球
		{
			ballColor->purple_cnt++;
			ballColor->red_cnt = 0;
			ballColor->blue_cnt = 0;
			ballColor->noball_cnt=0;
		}
        else if(ball_hsl->h <= 2)
        {
            if(ball_hsl->s >=20 && ball_hsl->s <= 235)      //可能检测到红球
            {
            ballColor->red_cnt++;
			ballColor->blue_cnt = 0;
			ballColor->purple_cnt = 0;
			ballColor->noball_cnt=0;
            }
            else        //无球
            {
                if(ball_hsl->l >=4)
                {
                    ballColor->red_cnt++;
                    ballColor->blue_cnt = 0;
                    ballColor->purple_cnt = 0;
                    ballColor->noball_cnt=0;
                }
                else
                {
                    ballColor->red_cnt = 0;
                    ballColor->blue_cnt = 0;
                    ballColor->purple_cnt = 0;
                    ballColor->noball_cnt++;
                }
             }
        }
		else
		{
			ballColor->red_cnt = 0;
			ballColor->blue_cnt = 0;
			ballColor->purple_cnt = 0;
			ballColor->noball_cnt++;
		}
	
	if(ballColor->red_cnt >= 5)
	{
		ballColor->ball_color = RED_BALL;
	}
	if (ballColor->blue_cnt >= 5)
	{
		ballColor->ball_color = BLUE_BALL;
	}
	if(ballColor->purple_cnt >= 5)
	{
		ballColor->ball_color = PURPLE_BALL;
	}
	if(ballColor->noball_cnt >= 5)
	{
		ballColor->ball_color = NO_BALL;
	}
}
void huidu_init(){
//	int ret_transmit;
//	uint8_t i2c_cmd;
//	i2c_cmd = GW_GRAY_DIGITAL_MODE;
// 	ret_transmit = HAL_I2C_Master_Transmit(&hi2c1, GW_GRAY_ADDR << 1, &i2c_cmd, 1, 100);
//	if (ret_transmit != HAL_OK) {
//		Error_Handler();
//	}
}

     /*    读取灰度传感器的值                           */
     /*pb8 推挽输出，pb9输入上拉模式                    */
     /*sensor【8】里存放八个传感器的值，1是白线，0是黑线*/
void read_huidu(){
		sensor_data = gw_gray_serial_read();
		SEP_ALL_BIT8(sensor_data,
			sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6], sensor[7]);
}

 uint8_t arrive_flag = 0;
//根据灰度传感器的值进行标志位赋值
void Huidu_Sensor()
{
	//读取灰度传感器的值
	read_huidu();
	//进行判断以及标志位赋值
	if(sensor[3] == 1 && sensor[4] == 1 && sensor[2] == 0 && sensor[5] == 0)	//中间两个照到，真好位于中间
	{
		R2_CONTROLLER.Huidu_Flag = 5;
	}
	else if(sensor[2] == 1 && sensor[1] == 0)	//2号照到，1号没照到，左小偏
	{
		R2_CONTROLLER.Huidu_Flag = 4;
	}
	else if(sensor[1] == 1 && sensor[0] == 0)	//1号照到，0号没照到，左中偏
	{
		R2_CONTROLLER.Huidu_Flag = 3;
	}
	else if(sensor[0] == 1)			//0号照到，左大偏
	{
		R2_CONTROLLER.Huidu_Flag = 2;
	}
	else if(sensor[5] == 1 && sensor[6] == 0)	//5号照到，6号没照到，右小偏
	{
		R2_CONTROLLER.Huidu_Flag = 6;
	}
	else if(sensor[6] == 1 && sensor[7] == 0)	//6号照到，7号没照到，右中偏
	{
		R2_CONTROLLER.Huidu_Flag = 7;
	}
	else if(sensor[7] == 1)		//7号照到，右大偏
	{
		R2_CONTROLLER.Huidu_Flag = 8;
	}
	else		//其他情况异常
	{	
		R2_CONTROLLER.Huidu_Flag = 0;
	}
    
    if(R2_CONTROLLER.Huidu_Flag == 0)
    {
        R2_CONTROLLER.HuiDu_Put = NO_DETECTION; 
    }
    else if(R2_CONTROLLER.Huidu_Flag >= 4 && R2_CONTROLLER.Huidu_Flag <= 6)
    {
            R2_CONTROLLER.HuiDu_Put = ALIGNED;
    }
    else if(R2_CONTROLLER.Huidu_Flag > 6)
    {
          R2_CONTROLLER.HuiDu_Put = PUT_RIGHT;
    }
    else if(R2_CONTROLLER.Huidu_Flag > 0 && R2_CONTROLLER.Huidu_Flag < 4)
    {
            R2_CONTROLLER.HuiDu_Put = PUT_LEFT;
    }    
    
    arrive_flag = 0;
    for(int i = 0; i<8;i++)
    {
        arrive_flag+=sensor[i];
    }
    if(arrive_flag >= 5)
    {
        R2_CONTROLLER.HuiDu_Arrive = 1;
    }
    else
    {
        R2_CONTROLLER.HuiDu_Arrive = 0;
    }
}

//char TCS34725_Read(uint8_t subAddr,unsigned char *dataBuffer, uint16_t bytesNumber)
//{
//	subAddr |= TCS34725_COMMAND_BIT;
//	uint8_t sendadd = (TCS34725_ADDRESS << 1) | 0x00;
//	if(HAL_I2C_Master_Transmit(I2cx,sendadd,&subAddr,1,1000)==0x00)
//	{
//		sendadd = (TCS34725_ADDRESS << 1) | 0x01;
//		if(HAL_I2C_Master_Receive(I2cx,sendadd,dataBuffer,bytesNumber,1000)==0x00)
//		{
//			return 1;
//		}
//	}else{
//               if(tcsouttime1 >=2){
//                I2C1_Reset();
//                //TCS34725_1_Init();
//								 tcsouttime1 = 0;
//                }
//               if(tcsouttime2 >=2){
//                I2C2_Reset();
//                //TCS34725_2_Init();
//								 tcsouttime2 = 0;
//                }
//                tcsouttime1++;
//                tcsouttime2++;
//        }
//	return 0;
//}



//char TCS34725_Write(uint8_t subAddr,uint8_t *dataBuffer,uint16_t bytesNumber)
//{
//	uint8_t sendadd = (TCS34725_ADDRESS << 1) | 0x00;
//	uint8_t sendBuffer[10]={0,};
//	sendBuffer[0]=subAddr|TCS34725_COMMAND_BIT;
//	for(uint8_t i=1;i<=bytesNumber;i++)
//	{
//		sendBuffer[i]=dataBuffer[i-1];
//	}
//	if(HAL_I2C_Master_Transmit(I2cx,sendadd,sendBuffer,bytesNumber+1,1000)==0)
//	{
//		return 1;
//	}
// 
//	return 0;
//}
 
 
//void TCS34725_SetIntegrationTime(uint8_t time)
//{
//	TCS34725_Write(TCS34725_ATIME,&time,1);
//}
//void TCS34725_SetGain(uint8_t gain)
//{
//	TCS34725_Write(TCS34725_CONTROL, &gain, 1);
//}
//void TCS34725_Enable(void)
//{
//	uint8_t cmd = TCS34725_ENABLE_PON;
//	
//	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
//	cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
//	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
//}
// 
//unsigned char TCS34725_1_Init(void)
//{
//	unsigned char id=0;
//	uint16_t number=0;
//    SwicthI2c(1);
//	uint8_t status = TCS34725_STATUS_AVALID;
//	TCS34725_Read(TCS34725_ID,&id,1);
//	printf("Read I2C ID:%02X\r\n",id);
//	if(id==0x4D || id ==0x44)
//	{
//		TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);//转换时间
////		TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
//		TCS34725_SetGain(TCS34725_GAIN_16X);
//		TCS34725_Enable();
//		return 1;
//	}
//	return 0;
//}

//unsigned char TCS34725_2_Init(void)
//{
//	unsigned char id=0;
//	uint16_t number=0;
//    SwicthI2c(2);
//	uint8_t status = TCS34725_STATUS_AVALID;
//	TCS34725_Read(TCS34725_ID,&id,1);
//	printf("Read I2C ID:%02X\r\n",id);
//	if(id==0x4D || id ==0x44)
//	{
//		TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);//转换时间
////		TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
//		TCS34725_SetGain(TCS34725_GAIN_16X);
//		TCS34725_Enable();
//		return 1;
//	}
//	return 0;
//}
 
 
/*******************************************************************************
 * @brief TCS34725获取单个通道数据
 *
 * @return data - 该通道的转换值
*******************************************************************************/
//uint16_t TCS34725_GetChannelData(uint8_t reg)
//{
//	uint8_t tmp[2] = {0,0};
//	uint16_t data;
//	
//	TCS34725_Read(reg, tmp, 2);
//	data = (tmp[1] << 8) | tmp[0];
//	return data;
//}
/*******************************************************************************
 * @brief TCS34725获取各个通道数据
 *
 * @return 1 - 转换完成，数据可用
 *   	   0 - 转换未完成，数据不可用
*******************************************************************************/
//char TCS34725_GetRawData(COLOR_RGBC *rgbc)
//{
//	uint8_t status = TCS34725_STATUS_AVALID;
//	
//	TCS34725_Read(TCS34725_STATUS, &status, 1);
//	
//	if(status & TCS34725_STATUS_AVALID)
//	{
//		rgbc->c = TCS34725_GetChannelData(TCS34725_CDATAL);	
//		rgbc->r = TCS34725_GetChannelData(TCS34725_RDATAL);	
//		rgbc->g = TCS34725_GetChannelData(TCS34725_GDATAL);	
//		rgbc->b = TCS34725_GetChannelData(TCS34725_BDATAL);
//		return 1;
//	}
//	return 0;
//}
/******************************************************************************/
//RGB转HSL
void RGBtoHSL(COLOR_RGBC *Rgb, COLOR_HSL *Hsl)
{
	uint8_t maxVal,minVal,difVal;
	uint8_t r = Rgb->r*100/Rgb->c;   //[0-100]
	uint8_t g = Rgb->g*100/Rgb->c;
	uint8_t b = Rgb->b*100/Rgb->c;
	
	maxVal = max3v(r,g,b);
	minVal = min3v(r,g,b);
	difVal = maxVal-minVal;
	
	//计算亮度
	Hsl->l = (maxVal+minVal)/2;   //[0-100]
	
	if(maxVal == minVal)//若r=g=b,灰度
	{
		Hsl->h = 0; 
		Hsl->s = 0;
	}
	else
	{
		//计算色调
		if(maxVal==r)
		{
			if(g>=b)
				Hsl->h = 60*(g-b)/difVal;
			else
				Hsl->h = 60*(g-b)/difVal+360;
		}
		else
			{
				if(maxVal==g)Hsl->h = 60*(b-r)/difVal+120;
				else
					if(maxVal==b)Hsl->h = 60*(r-g)/difVal+240;
			}
		
		//计算饱和度
		if(Hsl->l<=50)Hsl->s=difVal*100/(maxVal+minVal);  //[0-100]
		else
			Hsl->s=difVal*100/(200-(maxVal+minVal));
	}
}
/******************************************************************************/
//计算RGB的比例和Lux
void RGBto255RGB(COLOR_RGBC *Rgb,Sepan_RGBC *RGB255)
{
	double maxVal;
	double r_255=0.0,g_255=0.0,b_255=0.0;
 
	r_255 = (double)Rgb->r/Rgb->c*255;
	g_255 = (double)Rgb->g/Rgb->c*255;
	b_255 = (double)Rgb->b/Rgb->c*255;
	
	maxVal = max3v(r_255,g_255,b_255);
	r_255 = r_255/maxVal*255;
	g_255 = g_255/maxVal*255;
	b_255 = b_255/maxVal*255;
	
	RGB255->r = (unsigned char)r_255;
	RGB255->g = (unsigned char)g_255;
	RGB255->b = (unsigned char)b_255;
	double lux=(0.299*Rgb->r)+(0.587*Rgb->g)+(0.114*Rgb->b);
	RGB255->Lux = (unsigned short)lux;
	
	if(RGB255->r==255&&RGB255->g==255&&RGB255->b==255)
	{
		if(Rgb->c<255)
		{
			RGB255->r = 0;
			RGB255->g = 0;
			RGB255->b = 0;
		}
	}
}
//
//计算CCT色温
double calculateColorTemperature(COLOR_RGBC *Rgb)
{
	double trimX = 0;
  double trimY = 0;
  double trimZ = 0;
  double coorX = 0, coorY = 0;
  double CCT = 0;
  double n = 0;
  int R = Rgb->r;//255;
  int G = Rgb->g;//231;
  int B = Rgb->b;//131;
 
	//以下公式实现RGB转三刺激值
	trimX = 2.789 * R + 1.7517 * G + 1.1302 * B;
	trimY = 1 * R + 4.5907 * G + 0.0601 * B;
	trimZ = 0 * R + 0.0565 * G + 5.5943 * B;
	//以下公式实现三刺激值转色坐标
	coorX = trimX / (trimX + trimY + trimZ);
	coorY = trimY / (trimX + trimY + trimZ);
	n = (coorX - 0.3320) / (0.1858 - coorY);
	//以下公式实现色坐标转色温
	CCT = 437 * n * n * n + 3601 * n * n + 6831 * n + 5517;
	return CCT;
}



 

