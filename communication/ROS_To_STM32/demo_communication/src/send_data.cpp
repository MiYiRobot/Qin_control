#include "communication.h"

//test send value
float relative_distance=1564.0;
float relative_angle=22.0;
unsigned char sendFlag=2;

int main(int argc, char *argv[])
{
    //ros初始化
    ros::init(argc,argv,"SendData");
    ros::NodeHandle nh;
    ros::Rate loop_rate(25);
    setlocale(LC_ALL, "");
    //初始化串口
    serialInit();
    while (ros::ok())
    {
        ros::spinOnce();
        //发送数据
        sendDataFloat(relative_distance,relative_angle,sendFlag);   
        relative_angle++;
        if(relative_angle>10000)relative_angle=0;
        ROS_INFO("发送成功,distance = %f,angle = %f ",relative_distance,relative_angle);
        loop_rate.sleep();
    }
    return 0;
}
