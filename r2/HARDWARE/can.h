#ifndef  __can_H
#define  __can_H

#include "stm32f4xx_can.h"
#include "move.h"

void CAN1_init(void);
void M3508AngleIntegral(M3508_REAL_INFO*M3508_MOTOR);
void M3508AngleIntegral4(M3508_REAL_INFO *M3508_MOTOR);
#endif


