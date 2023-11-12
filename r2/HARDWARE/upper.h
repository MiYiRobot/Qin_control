#ifndef __UPPER_H
#define __UPPER_H

#include "stdint.h"

void upper_control(void);
uint8_t test_rise_time(int current,int boundary_current,int boundary_time);
uint8_t test_rise_time_up(int current,int boundary_current,int boundary_time);
#endif



