/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdarg.h> 
#include <string.h>

#include "action.h"
#include "air_joy.h"
#include "chassis.h"
#include "FSM.h"
#include "haoying_u8.h"
#include "pid.h"
#include "rm_motor.h"
#include "upper.h"
#include "usr_can.h"
#include "stepper.h"
#include "config.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ball_Claw_Pin GPIO_PIN_2
#define Ball_Claw_GPIO_Port GPIOG
#define Left_Seed_Claw_Pin GPIO_PIN_3
#define Left_Seed_Claw_GPIO_Port GPIOG
#define Right_Seed_Claw_Pin GPIO_PIN_4
#define Right_Seed_Claw_GPIO_Port GPIOG
#define step2dir_Pin GPIO_PIN_5
#define step2dir_GPIO_Port GPIOG
#define step2ena_Pin GPIO_PIN_6
#define step2ena_GPIO_Port GPIOG
#define step1dir_Pin GPIO_PIN_7
#define step1dir_GPIO_Port GPIOG
#define step1ena_Pin GPIO_PIN_8
#define step1ena_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
