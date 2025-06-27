/*
 * @Author: IcyClouda 2330329778@qq.com
 * @Date: 2025-06-21 02:07:45
 * @LastEditors: IcyClouda 2330329778@qq.com
 * @LastEditTime: 2025-06-28 01:48:50
 * @FilePath: \Wheeled Bipedal Robot\Core\Src\freertos.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
#define USE_Reinforce_Learning 0
#define Debug_NUM 10
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct
{
    float gyro[3];
    float accel[3];
    float pose[3];
    float temp;
} IMU_Dat;

typedef struct
{
    float MotorRoll;
    float MotorPitch;
    float MotorKnee;
    float MotorWheel;
} Motor_Dat;
typedef enum
{
    JointRoll = 3,
    JointPitch = 2,
    JointKnee = 1,
    JointWheel = 1
} Reduction_Ratio;

typedef struct
{
    //--------------pidp
    float kp;
    float ki;
    float kd;
    float f;
    //--------------pidv
    float dat;
    float target;
    float error;
    //----------------cal
    float integral;
    float diff;
    float l_dat;
    float l_target;
    float output;
    float PID_D;
    float PID_I;
    float PID_P;
    float l_output;
    //---------------max
    float max_output;
    float PID_P_MAX;
    float PID_I_MAX;
    float PID_D_MAX;
    float PID_INTG_MAX;
} PID_Para;
typedef enum
{
    Lost = 0,
    Established,
} Communication_Status;
typedef struct
{
    int Status; // 0: lost 1: established
    IMU_Dat IMU088;
    Motor_Dat Motor_Posi_Left;
    Motor_Dat Motor_Posi_Right;
    Motor_Dat Motor_Speed_Left;
    Motor_Dat Motor_Speed_Right;
    PID_Para PID_SPEED_Left[E_JOINT_NUM];
    PID_Para PID_SPEED_Right[E_JOINT_NUM];
    PID_Para PID_POSI_Left[E_JOINT_NUM];
    PID_Para PID_POSI_Right[E_JOINT_NUM];
    Motor_Dat Left_Motor_Torque_Output;
    Motor_Dat Right_Motor_Torque_Output;
    Motor_Dat Left_Motor_Position_Output;
    Motor_Dat Right_Motor_Position_Output;
} Sys_Dat;
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
double PID_MAX(double OUTPUT, double MAX);
void PID_CAL(PID_Para *pid);
void PID_SET_P(PID_Para *pid, float KP, float KI, float KD);
void PID_INPUT_V(PID_Para *pid, float TARGET, float DAT);
void PID_SET_M(PID_Para *pid, float _PID_P_MAX, float _PID_I_MAX, float _PID_D_MAX, float _max_output, float _PID_INTG_MAX);
PID_Para PID_PITCH, PID_SPEED;
// float pitch_target = 0;
float Ctrl_Signal = 0;
extern int Ctrl_Count;
int Lost_Count = 0;

float DebugData[Debug_NUM] = {0}; // debug

Sys_Dat ROBOT;
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
        vTaskDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_T_Calculator */
/**
 * @brief Function implementing the Calculator thread.
 * @param argument: Not used
 * @retval None
 */
float temp_target_left_posi[E_JOINT_NUM] = {0};
float temp_target_right_posi[E_JOINT_NUM] = {0};
/* USER CODE END Header_T_Calculator */
void T_Calculator(void *argument)
{
    /* USER CODE BEGIN T_Calculator */
    TickType_t lasttick = xTaskGetTickCount();
    volatile int ta = 1924;
    PID_SET_P(&PID_PITCH, 0, 0, 0);
    PID_SET_M(&PID_PITCH, 4, 1, 1, 5, 0.5);
    /* Infinite loop */
    for (;;)
    {
        if (USE_Reinforce_Learning)
        {
        }
        else
        {
            if (ROBOT.Status == Established)
            {
                PID_INPUT_V(&PID_PITCH, 0, ROBOT.IMU088.pose[0]);
                PID_CAL(&PID_PITCH);

                ROBOT.Left_Motor_Torque_Output.MotorWheel = -PID_PITCH.output;
                ROBOT.Right_Motor_Torque_Output.MotorWheel = -PID_PITCH.output;

                ROBOT.Left_Motor_Position_Output.MotorRoll = temp_target_left_posi[E_ROLL] * JointRoll;
                ROBOT.Left_Motor_Position_Output.MotorPitch = temp_target_left_posi[E_PITCH] * JointPitch;
                ROBOT.Left_Motor_Position_Output.MotorKnee = temp_target_left_posi[E_KNEE] * JointKnee;
                ROBOT.Left_Motor_Position_Output.MotorWheel = temp_target_left_posi[E_WHEEL] * JointWheel;
                ROBOT.Right_Motor_Position_Output.MotorRoll = temp_target_right_posi[E_ROLL] * JointRoll;
                ROBOT.Right_Motor_Position_Output.MotorPitch = temp_target_right_posi[E_PITCH] * JointPitch;
                ROBOT.Right_Motor_Position_Output.MotorKnee = temp_target_right_posi[E_KNEE] * JointKnee;
                ROBOT.Right_Motor_Position_Output.MotorWheel = temp_target_right_posi[E_WHEEL] * JointWheel;
            }
            else
            {
                temp_target_left_posi[E_ROLL] = ROBOT.Motor_Posi_Left.MotorRoll;
                temp_target_left_posi[E_PITCH] = ROBOT.Motor_Posi_Left.MotorPitch;
                temp_target_left_posi[E_KNEE] = ROBOT.Motor_Posi_Left.MotorKnee;
                temp_target_right_posi[E_ROLL] = ROBOT.Motor_Posi_Right.MotorRoll;
                temp_target_right_posi[E_PITCH] = ROBOT.Motor_Posi_Right.MotorPitch;
                temp_target_right_posi[E_KNEE] = ROBOT.Motor_Posi_Right.MotorKnee;
                ROBOT.Left_Motor_Torque_Output.MotorWheel = 0;
                ROBOT.Right_Motor_Torque_Output.MotorWheel = 0;
            }
            DebugData[0] = ROBOT.Left_Motor_Torque_Output.MotorWheel;
            DebugData[1] = ROBOT.Right_Motor_Torque_Output.MotorWheel;
            DebugData[2] = L_motor->para.tor;
            DebugData[3] = L_motor->para.tor;
            DebugData[4] = ROBOT.IMU088.pose[0];
        }
        vTaskDelay(1);
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
    static int initialized = 0;
    vTaskDelay(500);
    bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
    bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);
    bsp_can_init();

    dm_motor_init();
    vTaskDelay(100);
    dm_motor_disable(&hfdcan2, &L_motor[E_ROLL]);
    dm_motor_disable(&hfdcan1, &R_motor[E_ROLL]);
    vTaskDelay(1);
    dm_motor_disable(&hfdcan2, &L_motor[E_PITCH]);
    dm_motor_disable(&hfdcan1, &R_motor[E_PITCH]);
    vTaskDelay(1);
    dm_motor_disable(&hfdcan2, &L_motor[E_KNEE]);
    dm_motor_disable(&hfdcan1, &R_motor[E_KNEE]);
    vTaskDelay(1);
    dm_motor_disable(&hfdcan2, &L_motor[E_WHEEL]);
    dm_motor_disable(&hfdcan1, &R_motor[E_WHEEL]);
    /* Infinite loop */
    for (;;)
    {
        if (ROBOT.Status == Established)
        {

            if (!initialized)
            {
                dm_motor_enable(&hfdcan2, &L_motor[E_ROLL]);
                dm_motor_enable(&hfdcan1, &R_motor[E_ROLL]);
                vTaskDelay(1);
                dm_motor_enable(&hfdcan2, &L_motor[E_PITCH]);
                dm_motor_enable(&hfdcan1, &R_motor[E_PITCH]);
                vTaskDelay(1);
                dm_motor_enable(&hfdcan2, &L_motor[E_KNEE]);
                dm_motor_enable(&hfdcan1, &R_motor[E_KNEE]);
                vTaskDelay(1);
                dm_motor_enable(&hfdcan2, &L_motor[E_WHEEL]);
                dm_motor_enable(&hfdcan1, &R_motor[E_WHEEL]);
                vTaskDelay(1);
                initialized = 1; // 设置标志位
            }
            // 发送电机控制数据
            if (USE_Reinforce_Learning)
            {
                mit_ctrl(&L_motor[E_ROLL], 0, 0, ROBOT.Left_Motor_Torque_Output.MotorRoll);
                mit_ctrl(&R_motor[E_ROLL], 0, 0, ROBOT.Right_Motor_Torque_Output.MotorRoll);
                mit_ctrl(&L_motor[E_PITCH], 0, 0, ROBOT.Left_Motor_Torque_Output.MotorPitch);
                mit_ctrl(&R_motor[E_PITCH], 0, 0, ROBOT.Right_Motor_Torque_Output.MotorPitch);
                mit_ctrl(&L_motor[E_KNEE], 0, 0, ROBOT.Left_Motor_Torque_Output.MotorKnee);
                mit_ctrl(&R_motor[E_KNEE], 0, 0, ROBOT.Right_Motor_Torque_Output.MotorKnee);
                mit_ctrl(&L_motor[E_WHEEL], 0, 0, ROBOT.Left_Motor_Torque_Output.MotorWheel);
                mit_ctrl(&R_motor[E_WHEEL], 0, 0, ROBOT.Right_Motor_Torque_Output.MotorWheel);
            }
            else
            {
                mit_ctrl(&L_motor[E_ROLL], ROBOT.Left_Motor_Position_Output.MotorRoll, 0, 0);
                mit_ctrl(&R_motor[E_ROLL], -ROBOT.Right_Motor_Position_Output.MotorRoll, 0, 0);
                vTaskDelay(1);
                mit_ctrl(&L_motor[E_PITCH], ROBOT.Left_Motor_Position_Output.MotorPitch, 0, 0);
                mit_ctrl(&R_motor[E_PITCH], -ROBOT.Right_Motor_Position_Output.MotorPitch, 0, 0);
                vTaskDelay(1);
                mit_ctrl(&L_motor[E_KNEE], ROBOT.Left_Motor_Position_Output.MotorKnee, 0, 0);
                mit_ctrl(&R_motor[E_KNEE], -ROBOT.Right_Motor_Position_Output.MotorKnee, 0, 0);
                vTaskDelay(1);
                mit_ctrl(&L_motor[E_WHEEL], 0, 0, -ROBOT.Left_Motor_Torque_Output.MotorWheel);
                mit_ctrl(&R_motor[E_WHEEL], 0, 0, ROBOT.Right_Motor_Torque_Output.MotorWheel);
            }
        }

        else
        {
            initialized = 0;
            dm_motor_disable(&hfdcan2, &L_motor[E_ROLL]);
            dm_motor_disable(&hfdcan1, &R_motor[E_ROLL]);
            vTaskDelay(1);
            dm_motor_disable(&hfdcan2, &L_motor[E_PITCH]);
            dm_motor_disable(&hfdcan1, &R_motor[E_PITCH]);
            vTaskDelay(1);
            dm_motor_disable(&hfdcan2, &L_motor[E_KNEE]);
            dm_motor_disable(&hfdcan1, &R_motor[E_KNEE]);
            vTaskDelay(1);
            dm_motor_disable(&hfdcan2, &L_motor[E_WHEEL]);
            dm_motor_disable(&hfdcan1, &R_motor[E_WHEEL]);
        }
        vTaskDelay(1);
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
        vTaskDelay(1);
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
    vTaskDelay(500);
    while (BMI088_init())
    {
        ;
    }
    for (;;)
    {
        BMI088_read(ROBOT.IMU088.gyro, ROBOT.IMU088.accel, &ROBOT.IMU088.temp, ROBOT.IMU088.pose, 0.001);

        ROBOT.Motor_Posi_Left.MotorRoll = L_motor[E_ROLL].para.pos / JointRoll;
        ROBOT.Motor_Posi_Left.MotorPitch = L_motor[E_PITCH].para.pos / JointPitch;
        ROBOT.Motor_Posi_Left.MotorKnee = L_motor[E_KNEE].para.pos / JointKnee;
        ROBOT.Motor_Posi_Left.MotorWheel = L_motor[E_WHEEL].para.pos / JointWheel;

        ROBOT.Motor_Speed_Left.MotorRoll = L_motor[E_ROLL].para.vel / JointRoll;
        ROBOT.Motor_Speed_Left.MotorPitch = L_motor[E_PITCH].para.vel / JointPitch;
        ROBOT.Motor_Speed_Left.MotorKnee = L_motor[E_KNEE].para.vel / JointKnee;
        ROBOT.Motor_Speed_Left.MotorWheel = L_motor[E_WHEEL].para.vel / JointWheel;

        ROBOT.Motor_Posi_Right.MotorRoll = R_motor[E_ROLL].para.pos / -JointRoll;
        ROBOT.Motor_Posi_Right.MotorPitch = R_motor[E_PITCH].para.pos / -JointPitch;
        ROBOT.Motor_Posi_Right.MotorKnee = R_motor[E_KNEE].para.pos / -JointKnee;
        ROBOT.Motor_Posi_Right.MotorWheel = R_motor[E_WHEEL].para.pos / -JointWheel;

        ROBOT.Motor_Speed_Right.MotorRoll = R_motor[E_ROLL].para.vel / -JointRoll;
        ROBOT.Motor_Speed_Right.MotorPitch = R_motor[E_PITCH].para.vel / -JointPitch;
        ROBOT.Motor_Speed_Right.MotorKnee = R_motor[E_KNEE].para.vel / -JointKnee;
        ROBOT.Motor_Speed_Right.MotorWheel = R_motor[E_WHEEL].para.vel / -JointWheel;
        // vofa_demo();

        for (int i = 0; i < Debug_NUM; i++)
            vofa_send_data(i, DebugData[i]);
        vofa_sendframetail();
        vTaskDelay(1);
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
int flag = 0;
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
            // 1. 可以在此处理数据（示例：设置一些安全标志）
            uint8_t emergency_stop = (received_data.key_state == 0x7F);

            // 2. 创建回传结构体（可以修改或直接回传原数据）
            USB_DataPacket response_data = received_data;

            // 3. 可选：修改部分数据作为确认
            response_data.flag1 |= 0x80; // 设置最高位作为回传标志

            // 4. 发送回传数据
            uint8_t result;
            do
            {
                result = CDC_Transmit_HS((uint8_t *)&response_data,
                                         sizeof(response_data));
                if (result == USBD_BUSY)
                {
                    vTaskDelay(1); // 短暂延时后重试
                }
            } while (result == USBD_BUSY);

            // 5. 重置接收标志
            Lost_Count = 0;
        }
        // if (Lost_Count > 100)
        //     ROBOT.Status = 0;
        // else
        ROBOT.Status = flag;

        lastCount = Ctrl_Count;
        vTaskDelay(5);
    }
    /* USER CODE END T_Commu */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

double PID_MAX(double OUTPUT, double MAX)
{
    if (OUTPUT > MAX)
    {
        OUTPUT = MAX;
    }
    else if (OUTPUT < -MAX)
    {
        OUTPUT = -MAX;
    }
    return OUTPUT;
}
void PID_CAL(PID_Para *pid)
{
    pid->error = pid->target - pid->dat;
    pid->integral = PID_MAX(pid->integral + pid->error, pid->PID_INTG_MAX);
    if (pid->l_target != pid->target)
    {
        pid->integral = 0;
    }
    pid->diff = pid->dat - pid->l_dat;
    pid->PID_P = PID_MAX(pid->kp * pid->error, pid->PID_P_MAX);
    pid->PID_I = PID_MAX(pid->ki * pid->integral, pid->PID_I_MAX);
    pid->PID_D = PID_MAX(pid->kd * pid->diff, pid->PID_D_MAX);
    pid->output = PID_MAX(pid->PID_P + pid->PID_I - pid->PID_D + pid->f, pid->max_output);
    pid->l_target = pid->target;
    pid->l_dat = pid->dat;
    pid->l_output = pid->output;
}
void PID_SET_P(PID_Para *pid, float KP, float KI, float KD)
{
    pid->kp = KP;
    pid->ki = KI;
    pid->kd = KD;
}
void PID_INPUT_V(PID_Para *pid, float TARGET, float DAT)
{
    pid->target = TARGET;
    pid->dat = DAT;
}
void PID_SET_M(PID_Para *pid, float _PID_P_MAX, float _PID_I_MAX, float _PID_D_MAX, float _max_output, float _PID_INTG_MAX)
{
    pid->max_output = _max_output;
    pid->PID_P_MAX = _PID_P_MAX;
    pid->PID_I_MAX = _PID_I_MAX;
    pid->PID_D_MAX = _PID_D_MAX;
    pid->PID_INTG_MAX = _PID_INTG_MAX;
}
/* USER CODE END Application */
