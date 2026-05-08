/*
 * icm.h
 *
 *  Created on: Dec 27, 2025
 *      Author: YASH TATODE
 */


#ifndef ICM20948_H_
#define ICM20948_H_

#include "main.h"
#include <stdbool.h>
#include <math.h>

/* ICM-20948 I2C Address (AD0 = 0) */
#define ICM20948_ADDR           (0x69 << 1)  // 0x69 shifted for HAL

/* Register Banks */
#define REG_BANK_SEL            0x7F

/* USER BANK 0 REGISTERS */
#define UB0_WHO_AM_I            0x00
#define UB0_USER_CTRL           0x03
#define UB0_LP_CONFIG           0x05
#define UB0_PWR_MGMT_1          0x06
#define UB0_PWR_MGMT_2          0x07
#define UB0_INT_PIN_CFG         0x0F
#define UB0_INT_ENABLE          0x10
#define UB0_INT_ENABLE_1        0x11
#define UB0_INT_STATUS          0x19
#define UB0_INT_STATUS_1        0x1A
#define UB0_ACCEL_XOUT_H        0x2D
#define UB0_GYRO_XOUT_H         0x33
#define UB0_TEMP_OUT_H          0x39
#define UB0_FIFO_EN_1           0x66
#define UB0_FIFO_EN_2           0x67
#define UB0_FIFO_RST            0x68
#define UB0_FIFO_MODE           0x69
#define UB0_FIFO_COUNTH         0x70
#define UB0_FIFO_COUNTL         0x71
#define UB0_FIFO_R_W            0x72
#define UB0_DATA_RDY_STATUS     0x74

/* USER BANK 1 REGISTERS */
#define UB1_XA_OFFS_H           0x14
#define UB1_YA_OFFS_H           0x17
#define UB1_ZA_OFFS_H           0x1A
#define UB1_TIMEBASE_CORR_PLL   0x28

/* USER BANK 2 REGISTERS */
#define UB2_GYRO_SMPLRT_DIV     0x00
#define UB2_GYRO_CONFIG_1       0x01
#define UB2_GYRO_CONFIG_2       0x02
#define UB2_XG_OFFS_USRH        0x03
#define UB2_ODR_ALIGN_EN        0x09
#define UB2_ACCEL_SMPLRT_DIV_1  0x10
#define UB2_ACCEL_SMPLRT_DIV_2  0x11
#define UB2_ACCEL_CONFIG        0x14
#define UB2_ACCEL_CONFIG_2      0x15

/* WHO_AM_I Response */
#define ICM20948_WHO_AM_I_VAL   0xEA

/* Power Management */
#define PWR_MGMT_1_RESET        0x80
#define PWR_MGMT_1_SLEEP        0x40
#define PWR_MGMT_1_LP_EN        0x20
#define PWR_MGMT_1_TEMP_DIS     0x08
#define PWR_MGMT_1_CLKSEL_AUTO  0x01

/* Gyroscope Full Scale Range */
typedef enum {
    GYRO_FS_250DPS = 0,   // ±250 dps
    GYRO_FS_500DPS = 1,   // ±500 dps
    GYRO_FS_1000DPS = 2,  // ±1000 dps
    GYRO_FS_2000DPS = 3   // ±2000 dps
} ICM20948_GyroFS_t;

/* Accelerometer Full Scale Range */
typedef enum {
    ACCEL_FS_2G = 0,   // ±2g
    ACCEL_FS_4G = 1,   // ±4g
    ACCEL_FS_8G = 2,   // ±8g
    ACCEL_FS_16G = 3   // ±16g
} ICM20948_AccelFS_t;

/* Digital Low Pass Filter Configuration */
typedef enum {
    DLPF_0 = 0,  // 196.6 Hz (Gyro), 246.0 Hz (Accel)
    DLPF_1 = 1,  // 151.8 Hz (Gyro), 246.0 Hz (Accel)
    DLPF_2 = 2,  // 119.5 Hz (Gyro), 111.4 Hz (Accel)
    DLPF_3 = 3,  // 51.2 Hz (Gyro), 50.4 Hz (Accel)
    DLPF_4 = 4,  // 23.9 Hz (Gyro), 23.9 Hz (Accel)
    DLPF_5 = 5,  // 11.6 Hz (Gyro), 11.5 Hz (Accel)
    DLPF_6 = 6,  // 5.7 Hz (Gyro), 5.7 Hz (Accel)
    DLPF_7 = 7   // 361.4 Hz (Gyro), 473 Hz (Accel)
} ICM20948_DLPF_t;

/* Raw sensor data structure */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} ICM20948_RawData_t;

/* Scaled sensor data structure */
typedef struct {
    float accel_x;  // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;   // deg/s
    float gyro_y;
    float gyro_z;
    float temp;     // °C
} ICM20948_ScaledData_t;

/* Calibration data structure */
typedef struct {
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
    bool is_calibrated;
} ICM20948_CalibData_t;

/* Main device structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;

    ICM20948_GyroFS_t gyro_fs;
    ICM20948_AccelFS_t accel_fs;

    float gyro_scale;
    float accel_scale;

    ICM20948_RawData_t raw_data;
    ICM20948_ScaledData_t scaled_data;
    ICM20948_CalibData_t calib_data;

    uint8_t current_bank;
} ICM20948_t;

/* Function Prototypes */

// Initialization and Configuration
HAL_StatusTypeDef ICM20948_Init(ICM20948_t *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ICM20948_Reset(ICM20948_t *dev);
HAL_StatusTypeDef ICM20948_WakeUp(ICM20948_t *dev);
HAL_StatusTypeDef ICM20948_Sleep(ICM20948_t *dev);
HAL_StatusTypeDef ICM20948_VerifyConnection(ICM20948_t *dev);

// Configuration Functions
HAL_StatusTypeDef ICM20948_SetGyroFullScale(ICM20948_t *dev, ICM20948_GyroFS_t fs);
HAL_StatusTypeDef ICM20948_SetAccelFullScale(ICM20948_t *dev, ICM20948_AccelFS_t fs);
HAL_StatusTypeDef ICM20948_SetGyroDLPF(ICM20948_t *dev, ICM20948_DLPF_t dlpf);
HAL_StatusTypeDef ICM20948_SetAccelDLPF(ICM20948_t *dev, ICM20948_DLPF_t dlpf);
HAL_StatusTypeDef ICM20948_SetSampleRateDivider(ICM20948_t *dev, uint8_t gyro_div, uint16_t accel_div);

// Data Reading Functions
HAL_StatusTypeDef ICM20948_ReadRawData(ICM20948_t *dev);
HAL_StatusTypeDef ICM20948_ReadScaledData(ICM20948_t *dev);
HAL_StatusTypeDef ICM20948_ReadTemperature(ICM20948_t *dev, float *temp);

// Calibration Functions
HAL_StatusTypeDef ICM20948_CalibrateGyro(ICM20948_t *dev, uint16_t samples);
HAL_StatusTypeDef ICM20948_CalibrateAccel(ICM20948_t *dev, uint16_t samples);
HAL_StatusTypeDef ICM20948_SetCalibrationData(ICM20948_t *dev, ICM20948_CalibData_t *calib);
HAL_StatusTypeDef ICM20948_GetCalibrationData(ICM20948_t *dev, ICM20948_CalibData_t *calib);
void ICM20948_ApplyCalibration(ICM20948_t *dev);

// Utility Functions
HAL_StatusTypeDef ICM20948_SelectBank(ICM20948_t *dev, uint8_t bank);
HAL_StatusTypeDef ICM20948_ReadRegister(ICM20948_t *dev, uint8_t bank, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ICM20948_WriteRegister(ICM20948_t *dev, uint8_t bank, uint8_t reg, uint8_t data);
HAL_StatusTypeDef ICM20948_ReadRegisters(ICM20948_t *dev, uint8_t bank, uint8_t reg, uint8_t *data, uint16_t length);

#endif /* ICM20948_H_ */
