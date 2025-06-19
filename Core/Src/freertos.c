/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Calculator */
osThreadId_t CalculatorHandle;
const osThreadAttr_t Calculator_attributes = {
  .name = "Calculator",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for Motor_Control */
osThreadId_t Motor_ControlHandle;
const osThreadAttr_t Motor_Control_attributes = {
  .name = "Motor_Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for USER */
osThreadId_t USERHandle;
const osThreadAttr_t USER_attributes = {
  .name = "USER",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for Sensor */
osThreadId_t SensorHandle;
const osThreadAttr_t Sensor_attributes = {
  .name = "Sensor",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void T_Calculator(void *argument);
void T_Motor_Control(void *argument);
void T_USER(void *argument);
void T_Sensor(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Calculator */
  CalculatorHandle = osThreadNew(T_Calculator, NULL, &Calculator_attributes);

  /* creation of Motor_Control */
  Motor_ControlHandle = osThreadNew(T_Motor_Control, NULL, &Motor_Control_attributes);

  /* creation of USER */
  USERHandle = osThreadNew(T_USER, NULL, &USER_attributes);

  /* creation of Sensor */
  SensorHandle = osThreadNew(T_Sensor, NULL, &Sensor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_T_Calculator */
/**
* @brief Function implementing the Calculator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_T_Calculator */
void T_Calculator(void *argument)
{
  /* USER CODE BEGIN T_Calculator */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END T_Calculator */
}

/* USER CODE BEGIN Header_T_Motor_Control */
/**
* @brief Function implementing the Motor_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_T_Motor_Control */
void T_Motor_Control(void *argument)
{
  /* USER CODE BEGIN T_Motor_Control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END T_Motor_Control */
}

/* USER CODE BEGIN Header_T_USER */
/**
* @brief Function implementing the USER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_T_USER */
void T_USER(void *argument)
{
  /* USER CODE BEGIN T_USER */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END T_USER */
}

/* USER CODE BEGIN Header_T_Sensor */
/**
* @brief Function implementing the Sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_T_Sensor */
void T_Sensor(void *argument)
{
  /* USER CODE BEGIN T_Sensor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END T_Sensor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

