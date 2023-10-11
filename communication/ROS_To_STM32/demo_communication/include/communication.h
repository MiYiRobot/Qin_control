#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include <ros/ros.h>
#include <boost/asio.hpp>

extern void serialInit();
unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len);
extern void sendData(short data1,short data2,unsigned char crtlFlag);
extern void sendDataFloat(float data1,float data2,unsigned char crtlFlag);
extern bool receiveData(short &data1,short &data2,float &Angle,unsigned char &ctrlFlag);

#endif
