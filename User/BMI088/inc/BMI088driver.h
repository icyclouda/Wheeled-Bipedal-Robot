/*
 * @Author: IcyClouda 2330329778@qq.com
 * @Date: 2025-06-20 16:17:34
 * @LastEditors: IcyClouda 2330329778@qq.com
 * @LastEditTime: 2025-06-29 21:39:49
 * @FilePath: \CtrBoard-H7_IMU\Device\BMI088\inc\BMI088driver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "stdint.h"
#include "main.h"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
// #define BMI088_ACCEL_RANGE_6G
// #define BMI088_ACCEL_RANGE_12G
// #define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
// #define BMI088_GYRO_RANGE_1000
// #define BMI088_GYRO_RANGE_500
// #define BMI088_GYRO_RANGE_250
// #define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

typedef struct
{
    float a0, a1, a2;
    float b0, b1, b2;
    float x1, x2; // 输入历史
    float y1, y2; // 输出历史
} SecondOrderLPF;

typedef struct BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} __attribute__((packed)) bmi088_raw_data_t;
typedef enum
{
    P_ROLL = 0,
    P_PITCH,
    P_YAW,
    IMU_NUM
} IMU_alt;
typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;
typedef enum
{
    data_now,
    data_last,
    data_lastlast
} Data_Enum;
enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

extern uint8_t BMI088_init(void);
extern uint8_t bmi088_accel_init(void);
extern uint8_t bmi088_gyro_init(void);
extern void BMI088_read(float gyro[IMU_NUM], float accel[IMU_NUM], float *temperate, float pose[IMU_NUM], float dt);

extern float omega;
extern float b0;
extern float b2;
extern float b1;
extern float a1;
extern float a2;
extern float m;
#endif
