//
// Created by Ray on 2023/11/24.
//

#ifndef INC_2024RC_B_R1_ACTION_H
#define INC_2024RC_B_R1_ACTION_H

#include "main.h"

extern UART_HandleTypeDef huart4;

#define INSTALL_ERROR_X		0.0
#define INSTALL_ERROR_Y		0.0

typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;
	float POS_X;
	float POS_Y;
	float W_Z;

	float LAST_POS_X;
	float LAST_POS_Y;

	float DELTA_POS_X;
	float DELTA_POS_Y;
	
	float REAL_X;
	float REAL_Y;
} ACTION_GL_POS;

extern ACTION_GL_POS ACTION_GL_POS_DATA;
void action_data_analyse(void);
void Update_Action_gl_position(float value[6]);

#endif //INC_2024RC_B_R1_ACTION_H