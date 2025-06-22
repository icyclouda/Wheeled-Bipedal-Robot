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
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct IMU_Dat
{
    float gyro[3];
    float accel[3];
    float pose[3];
    float temp;
};
struct Motor_Dat
{
    float MotorRoll;
    float MotorPitch;
    float MotorKnee;
    float MotorWheel;
};
typedef enum
{
    JointRoll = 3,
    JointPitch = 2,
    JointKnee = 1,
    JointWheel = 1
} Reduction_Ratio;
struct Sys_Dat
{
    int Status; // 0: lost 1: established
    struct IMU_Dat IMU088;
    struct Motor_Dat Motor_Posi_Left;
    struct Motor_Dat Motor_Posi_Right;
    struct Motor_Dat Motor_Speed_Left;
    struct Motor_Dat Motor_Speed_Right;
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float Ctrl_Signal = 0;
extern int Ctrl_Count;
int Lost_Count = 0;
struct Sys_Dat ROBOT;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .cb_mem = &defaultTaskControlBlock,
    .cb_size = sizeof(defaultTaskControlBlock),
    .stack_mem = &defaultTaskBuffer[0],
    .stack_size = sizeof(defaultTaskBuffer),
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Calculator */
osThreadId_t CalculatorHandle;
uint32_t CalculatorBuffer[512];
osStaticThreadDef_t CalculatorControlBlock;
const osThreadAttr_t Calculator_attributes = {
    .name = "Calculator",
    .cb_mem = &CalculatorControlBlock,
    .cb_size = sizeof(CalculatorControlBlock),
    .stack_mem = &CalculatorBuffer[0],
    .stack_size = sizeof(CalculatorBuffer),
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for Motor_Control */
osThreadId_t Motor_ControlHandle;
uint32_t Motor_ControlBuffer[512];
osStaticThreadDef_t Motor_ControlControlBlock;
const osThreadAttr_t Motor_Control_attributes = {
    .name = "Motor_Control",
    .cb_mem = &Motor_ControlControlBlock,
    .cb_size = sizeof(Motor_ControlControlBlock),
    .stack_mem = &Motor_ControlBuffer[0],
    .stack_size = sizeof(Motor_ControlBuffer),
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for USER */
osThreadId_t USERHandle;
uint32_t USERBuffer[128];
osStaticThreadDef_t USERControlBlock;
const osThreadAttr_t USER_attributes = {
    .name = "USER",
    .cb_mem = &USERControlBlock,
    .cb_size = sizeof(USERControlBlock),
    .stack_mem = &USERBuffer[0],
    .stack_size = sizeof(USERBuffer),
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for Sensor */
osThreadId_t SensorHandle;
uint32_t SensorBuffer[512];
osStaticThreadDef_t SensorControlBlock;
const osThreadAttr_t Sensor_attributes = {
    .name = "Sensor",
    .cb_mem = &SensorControlBlock,
    .cb_size = sizeof(SensorControlBlock),
    .stack_mem = &SensorBuffer[0],
    .stack_size = sizeof(SensorBuffer),
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for Commu */
osThreadId_t CommuHandle;
uint32_t myTask06Buffer[256];
osStaticThreadDef_t myTask06ControlBlock;
const osThreadAttr_t Commu_attributes = {
    .name = "Commu",
    .cb_mem = &myTask06ControlBlock,
    .cb_size = sizeof(myTask06ControlBlock),
    .stack_mem = &myTask06Buffer[0],
    .stack_size = sizeof(myTask06Buffer),
    .priority = (osPriority_t)osPriorityRealtime5,
};
/* Definitions for Ctrl */
osMessageQueueId_t CtrlHandle;
uint8_t myQueue01Buffer[16 * sizeof(uint16_t)];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t Ctrl_attributes = {
    .name = "Ctrl",
    .cb_mem = &myQueue01ControlBlock,
    .cb_size = sizeof(myQueue01ControlBlock),
    .mq_mem = &myQueue01Buffer,
    .mq_size = sizeof(myQueue01Buffer)};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void T_Calculator(void *argument);
void T_Motor_Control(void *argument);
void T_USER(void *argument);
void T_Sensor(void *argument);
void T_Commu(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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

    /* Create the queue(s) */
    /* creation of Ctrl */
    CtrlHandle = osMessageQueueNew(16, sizeof(uint16_t), &Ctrl_attributes);

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

    /* creation of Commu */
    CommuHandle = osThreadNew(T_Commu, NULL, &Commu_attributes);

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
        BMI088_read(ROBOT.IMU088.gyro, ROBOT.IMU088.accel, &ROBOT.IMU088.temp, ROBOT.IMU088.pose, 0.01);
        
        ROBOT.Motor_Posi_Left.MotorRoll = L_motor[ROLL].para.pos / JointRoll;
        ROBOT.Motor_Posi_Left.MotorPitch = L_motor[PITCH].para.pos / JointPitch;
        ROBOT.Motor_Posi_Left.MotorKnee = L_motor[KNEE].para.pos / JointKnee;
        ROBOT.Motor_Posi_Left.MotorWheel = L_motor[WHEEL].para.pos / JointWheel;

        ROBOT.Motor_Speed_Left.MotorRoll = L_motor[ROLL].para.vel / JointRoll;
        ROBOT.Motor_Speed_Left.MotorPitch = L_motor[PITCH].para.vel / JointPitch;
        ROBOT.Motor_Speed_Left.MotorKnee = L_motor[KNEE].para.vel / JointKnee;
        ROBOT.Motor_Speed_Left.MotorWheel = L_motor[WHEEL].para.vel / JointWheel;

        ROBOT.Motor_Posi_Right.MotorRoll = R_motor[ROLL].para.pos / JointRoll;
        ROBOT.Motor_Posi_Right.MotorPitch = R_motor[PITCH].para.pos / JointPitch;
        ROBOT.Motor_Posi_Right.MotorKnee = R_motor[KNEE].para.pos / JointKnee;
        ROBOT.Motor_Posi_Right.MotorWheel = R_motor[WHEEL].para.pos / JointWheel;

        ROBOT.Motor_Speed_Right.MotorRoll = R_motor[ROLL].para.vel / JointRoll;
        ROBOT.Motor_Speed_Right.MotorPitch = R_motor[PITCH].para.vel / JointPitch;
        ROBOT.Motor_Speed_Right.MotorKnee = R_motor[KNEE].para.vel / JointKnee;
        ROBOT.Motor_Speed_Right.MotorWheel = R_motor[WHEEL].para.vel / JointWheel;
        osDelay(1);
    }
    /* USER CODE END T_Sensor */
}

/* USER CODE BEGIN Header_T_Commu */
/**
 * @brief Function implementing the Commu thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_T_Commu */
void T_Commu(void *argument)
{
    /* USER CODE BEGIN T_Commu */
    static int lastCount = 0;
    /* Infinite loop */
    for (;;)
    {
        if (Ctrl_Count == lastCount)
        {
            if (Lost_Count++ > 150)
                Lost_Count = 150;
        }
        else
        {

            Lost_Count = 0;
        }
        if (Lost_Count > 100)
            ROBOT.Status = 0;
        else
            ROBOT.Status = 1;

        lastCount = Ctrl_Count;
        osDelay(5);
    }
    /* USER CODE END T_Commu */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
