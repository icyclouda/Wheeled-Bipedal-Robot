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
#include "BMI088driver.h"
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "vofa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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

float gyro[3], accel[3], temp, pose[3];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Calculator */
osThreadId_t CalculatorHandle;
uint32_t CalculatorBuffer[ 512 ];
osStaticThreadDef_t CalculatorControlBlock;
const osThreadAttr_t Calculator_attributes = {
  .name = "Calculator",
  .cb_mem = &CalculatorControlBlock,
  .cb_size = sizeof(CalculatorControlBlock),
  .stack_mem = &CalculatorBuffer[0],
  .stack_size = sizeof(CalculatorBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Motor_Control */
osThreadId_t Motor_ControlHandle;
uint32_t Motor_ControlBuffer[ 512 ];
osStaticThreadDef_t Motor_ControlControlBlock;
const osThreadAttr_t Motor_Control_attributes = {
  .name = "Motor_Control",
  .cb_mem = &Motor_ControlControlBlock,
  .cb_size = sizeof(Motor_ControlControlBlock),
  .stack_mem = &Motor_ControlBuffer[0],
  .stack_size = sizeof(Motor_ControlBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for USER */
osThreadId_t USERHandle;
uint32_t USERBuffer[ 128 ];
osStaticThreadDef_t USERControlBlock;
const osThreadAttr_t USER_attributes = {
  .name = "USER",
  .cb_mem = &USERControlBlock,
  .cb_size = sizeof(USERControlBlock),
  .stack_mem = &USERBuffer[0],
  .stack_size = sizeof(USERBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Sensor */
osThreadId_t SensorHandle;
uint32_t SensorBuffer[ 512 ];
osStaticThreadDef_t SensorControlBlock;
const osThreadAttr_t Sensor_attributes = {
  .name = "Sensor",
  .cb_mem = &SensorControlBlock,
  .cb_size = sizeof(SensorControlBlock),
  .stack_mem = &SensorBuffer[0],
  .stack_size = sizeof(SensorBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for VOFA */
osThreadId_t VOFAHandle;
uint32_t myTask06Buffer[ 256 ];
osStaticThreadDef_t myTask06ControlBlock;
const osThreadAttr_t VOFA_attributes = {
  .name = "VOFA",
  .cb_mem = &myTask06ControlBlock,
  .cb_size = sizeof(myTask06ControlBlock),
  .stack_mem = &myTask06Buffer[0],
  .stack_size = sizeof(myTask06Buffer),
  .priority = (osPriority_t) osPriorityRealtime5,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void T_Calculator(void *argument);
void T_Motor_Control(void *argument);
void T_USER(void *argument);
void T_Sensor(void *argument);
void T_VOFA(void *argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* creation of VOFA */
  VOFAHandle = osThreadNew(T_VOFA, NULL, &VOFA_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
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
    for (;;)
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
    osDelay(500);
    bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
    bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);
    bsp_can_init();
    dm_motor_init();
    osDelay(100);
    /* Infinite loop */
    for (;;)
    {
        dm_motor_disable(&hfdcan2, &L_motor[ROLL]);
        dm_motor_disable(&hfdcan1, &R_motor[ROLL]);
        osDelay(1);
        dm_motor_disable(&hfdcan2, &L_motor[PITCH]);
        dm_motor_disable(&hfdcan1, &R_motor[PITCH]);
        osDelay(1);
        dm_motor_disable(&hfdcan2, &L_motor[KNEE]);
        dm_motor_disable(&hfdcan1, &R_motor[KNEE]);
        osDelay(1);
        dm_motor_disable(&hfdcan2, &L_motor[WHEEL]);
        dm_motor_disable(&hfdcan1, &R_motor[WHEEL]);
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
    for (;;)
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
    osDelay(500);
    while (BMI088_init())
    {
        ;
    }
    for (;;)
    {
        BMI088_read(gyro, accel, &temp, pose, 0.01);
        osDelay(10);
    }
  /* USER CODE END T_Sensor */
}

/* USER CODE BEGIN Header_T_VOFA */
/**
* @brief Function implementing the VOFA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_T_VOFA */
void T_VOFA(void *argument)
{
  /* USER CODE BEGIN T_VOFA */
  /* Infinite loop */
  for(;;)
  {
//		vofa_start();
    osDelay(1);
  }
  /* USER CODE END T_VOFA */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

