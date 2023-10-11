#include "communication.h"
//设定工作空间名称
using namespace std;
using namespace boost::asio;

//串口相关变量
boost::asio::io_service iosev;   //创建一个io_service
boost::asio::serial_port sp(iosev,"/dev/ttyUSB0");  //构造一个串口，将"/dev/ttyUSB0"转移给实例iosev
boost::system::error_code err; 

//串口发送接收相关常量、变量、共用体对象
const unsigned char header[2] = {0x55,0xaa};    //数据头
const unsigned char ender[2] = {0x0d,0x0a};     //数据尾

//发送左右轮速控制速度结构体
union sendData1
{
    short d;
    unsigned char data[2];
}leftVelSet,rightVelSet;
//发送角度和相对位置
union sendData2
{
    float d;
    unsigned char data[4];
}Distance,Target_Angle;

//接收数据(左轮速、右轮速、角度)共用体
union receiveData1
{
    short d;
    unsigned char data[2];
}RPM1,RPM2;

union receiveData2
{
    float d;
    unsigned char data[4];
}angle;

//串口初始化
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));  //波特率
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none)); //无硬件流控制
    sp.set_option(serial_port::parity(serial_port::parity::none));  //无奇偶校验位
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));  //一位停止位
    sp.set_option(serial_port::character_size(8));   //8位数据位

}

/********************************************************
函数功能：将对机器人的左右轮子控制速度，打包发送给下位机
入口参数：机器人、角速度、控制标志位
出口参数：无
********************************************************/
// void sendData(short data1,short data2,unsigned char crtlFlag)
// {
//     //发送数据缓存区
//     unsigned char buf[11]={0};
//     int i,length;  //标志位

//     leftVelSet.d = data1;
//     rightVelSet.d = data2;

//     //设置消息头
//     for(i=0;i<2;i++)
//     {
//         buf[i] = header[i];      //buf[0]   buf[1]
//     }

//     //设置机器人左右轮速度
//     length = 5;
//     buf[2] = length;   //数据长度   buf[2]
//     for(i = 0;i < 2; i++)
//     {
//         buf[i + 3] = leftVelSet.data[i];    //buf[3]   buf[4]
//         buf[i + 5] = rightVelSet.data[i];   //buf[5]   buf[6]
//     }
//     //预留控制位
//     buf[3 + length - 1] =crtlFlag;    //buf[7]

//     //设置校验位
//     buf[3+length] = serial_get_crc8_value(buf,3 + length); //buf[8]
//     //设置消息尾
//     for(i=0;i<2;i++)
//     {
//         buf[3+length+1+i] = header[i];      //buf[9]   buf[10]
//     }
//     //通过串口发送数据
//     boost::asio::write(sp,boost::asio::buffer(buf));
// }

/********************************************************
函数功能：将球相对机器人的距离和角度，打包发送给下位机
入口参数：机器人、角速度、控制标志位
出口参数：无
********************************************************/
void sendDataFloat(float data1,float data2,unsigned char crtlFlag)
{
    //发送数据缓存区
    unsigned char buf[15]={0};
    int i,length;  //标志位

    Distance.d = data1;
    Target_Angle.d = data2;

    //设置消息头
    for(i=0;i<2;i++)
    {
        buf[i] = header[i];      //buf[0]   buf[1]
    }

    //设置求与机器人的距离与角度
    length = 9;
    buf[2] = length;   //数据长度   buf[2]
    for(i = 0;i < 4; i++)
    {
        buf[i + 3] = Distance.data[i];    //buf[3]  buf[4] buf[5] buf[6]
        buf[i + 7] = Target_Angle.data[i];   //buf[7]  buf[8]  buf[9]  buf[10]
    }
    //预留控制位
    buf[3 + length - 1] =crtlFlag;    //buf[11]

    //设置校验位
    buf[3+length] = serial_get_crc8_value(buf,3 + length); //buf[12]
    //设置消息尾
    for(i=0;i<2;i++)
    {
        buf[3+length+1+i] = header[i];      //buf[13]   buf[14]
    }
    //通过串口发送数据
    boost::asio::write(sp,boost::asio::buffer(buf));
}



/********************************************************
函数功能：从下位机读取数据
入口参数：机器人左轮轮速、右轮轮速、角度，预留控制位
出口参数：bool
********************************************************/
bool receiveData(short &data1,short &data2,float &Angle,unsigned char &ctrlFlag)
{
            char i, length =0;   //标志位
            unsigned char checkSum;  //校验码标志位
            unsigned char buf[150]={0};
            //此段代码可以读取数据的结尾，进而进行数据头部的读取
            //---------------------------------------------------------
            try
            {
                boost::asio::streambuf response;
                boost::asio::read_until(sp,response,"\r\n",err);
                copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
                istream_iterator<unsigned char>(),
                buf);
            }
           catch(boost::system::system_error &err)
           {
            ROS_INFO("read_until error");
           }
           //--------------------------------------------------------------   

           //检查数据头
           if(buf[0] != header[0] || buf[1] != header[1])    //buf[0]  buf[1]
           {
            ROS_ERROR("Received message header error!");
            return false;
           }

           //数据长度
           length = buf[2];     //buf[2]   length = 9=2 + 2 + 4 +1
           ///检查消息检验位
           checkSum = serial_get_crc8_value(buf,3+length);   //buf[11]
           if(checkSum != buf[3+length])
           {
            ROS_ERROR("Receive data check sum error!");
            return false;
           }
            //读取速度值
            for(i = 0;i < 2; i++)
            {               
                RPM1.data[i] = buf[i + 3];   //buf[3]   buf[4]
                RPM2.data[i] = buf[i + 5];   //buf[5]   buf[6]
            }
            //读取角度值
            for(i = 0;i < 4;i++)
            {
                angle.data[i] = buf[7 +i];  //buf[7] buf[8]  buf[9] buf[10]
            }
            //读取数据控制位
            ctrlFlag = buf[length + 3 -1];      //buf[11]

            data1 = RPM1.d;
            data2 = RPM2.d;
            Angle = angle.d;

            return true;
}

//数据冗余检查
unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len)
{
	unsigned char crc = 0;
	unsigned char i;
	while(len--)
	{
		crc ^= *data++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
				crc=(crc>>1)^0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}
