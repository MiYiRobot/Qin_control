/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_PRINTF_ENABLE 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for FSM_switch */
osThreadId_t FSM_switchHandle;
const osThreadAttr_t FSM_switch_attributes = {
  .name = "FSM_switch",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Seed */
osThreadId_t SeedHandle;
const osThreadAttr_t Seed_attributes = {
  .name = "Seed",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void FSM_Function(void *argument);
void Chassis_Function(void *argument);
void Seed_Function(void *argument);
void Shoot_Function(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of FSM_switch */
  FSM_switchHandle = osThreadNew(FSM_Function, NULL, &FSM_switch_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(Chassis_Function, NULL, &Chassis_attributes);

  /* creation of Seed */
  SeedHandle = osThreadNew(Seed_Function, NULL, &Seed_attributes);

  /* creation of Shoot */
  ShootHandle = osThreadNew(Shoot_Function, NULL, &Shoot_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_FSM_Function */
/**
  * @brief  Function implementing the FSM_switch thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_FSM_Function */
void FSM_Function(void *argument)
{
  /* USER CODE BEGIN FSM_Function */
  /* Infinite loop */
  for(;;)
  {
    robot_fsm();
    Motor_Control();
    osDelay(1);
  }
  /* USER CODE END FSM_Function */
}

/* USER CODE BEGIN Header_Chassis_Function */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Function */
void Chassis_Function(void *argument)
{
  /* USER CODE BEGIN Chassis_Function */
  /* Infinite loop */
  for(;;)
  {
    if (chassis_state == high_speed)
    {
      ROBOT_CHASSI.Vx_MAX = 2.0f;
      ROBOT_CHASSI.Vy_MAX = 2.0f;
	  ROBOT_CHASSI.Vw_MAX = 2.0f;
    }
    else if (chassis_state == low_speed)
    {
      ROBOT_CHASSI.Vx_MAX = 0.5f;
      ROBOT_CHASSI.Vy_MAX = 0.5f;
	    ROBOT_CHASSI.Vw_MAX = 0.6f;
    }
    Free_Control();
    Robot_Wheels_RPM_calculate();
    osDelay(1);
  }
  /* USER CODE END Chassis_Function */
}

/* USER CODE BEGIN Header_Seed_Function */
/**
* @brief Function implementing the Seed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Seed_Function */
void Seed_Function(void *argument)
{
  /* USER CODE BEGIN Seed_Function */
  /* Infinite loop */
  for(;;)
  {
    switch (seed_state)
    {
    case seed_disable:
      /*云台回收，步进电机下降，夹爪松开，凸轮不转*/
      break;
    case seed_init:
      /*云台伸展，步进电机下降，夹爪松开，凸轮不转*/
        Init_Seed();
      break;
    case peek_deposit:
        Deposit_Seed();
      break;
    case take_put:
        Put_Seed();
      break;
    }
    osDelay(1);
  }
  /* USER CODE END Seed_Function */
}

/* USER CODE BEGIN Header_Shoot_Function */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Function */
void Shoot_Function(void *argument)
{
  /* USER CODE BEGIN Shoot_Function */
  /* Infinite loop */
  for(;;)
  {
    switch (shoot_state)
    {
    case shoot_init:
      /*夹爪下松，云台正，推射收，摩擦带摩擦轮停*/
      Init_Ball();
      break;
    case load:
      /*云台回正，发射机构停止，夹球,夹爪上去，放球*/
      Load_Ball();
      break;
    case shooting:
      /*云台转动，发射，夹爪下降*/
      Shoot_Ball();
      break;
    }
    osDelay(1);
  }
  /* USER CODE END Shoot_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

