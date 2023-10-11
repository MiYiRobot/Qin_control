#include "communication.h"
//接收数据相关变量
short rpm1=0,rpm2=0;
float angle=0;
unsigned char receiveFlag=0;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"receive_data_from_stm32");

    ros::NodeHandle nh;
    ros::Rate loop_rate(25);
    setlocale(LC_ALL, "");
    //初始化串口
    serialInit();

    while(ros::ok())
    {
        ros::spinOnce();
        //接收数据
        receiveData(rpm1,rpm2,angle,receiveFlag);
        //打印数据
        ROS_INFO("%d, %d, %f,%d\n",rpm1,rpm2,angle,receiveFlag);
        loop_rate.sleep();
    }
    
    return 0;
}




 



