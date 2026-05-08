/*
 * adxl.h - ADXL375 High-G Accelerometer Driver
 *
 *  Created on: Feb 12, 2026
 *      Author: YASH TATODE
 *  Fixed v2.0:
 *  - Added ADXL375_GetVerticalVelocity() for Z-only velocity readout
 *  - Updated driver version comment
 */

#ifndef ADXL_H_
#define ADXL_H_

#include "main.h"
#include <stdbool.h>
#include <math.h>

/* ADXL375 I2C Addresses */
#define ADXL375_ADDRESS_ALT_LOW   0x53
#define ADXL375_ADDRESS_ALT_HIGH  0x1D

/* ADXL375 Registers */
#define ADXL375_REG_DEVID          0x00
#define ADXL375_REG_THRESH_SHOCK   0x1D
#define ADXL375_REG_OFSX           0x1E
#define ADXL375_REG_OFSY           0x1F
#define ADXL375_REG_OFSZ           0x20
#define ADXL375_REG_DUR            0x21
#define ADXL375_REG_LATENT         0x22
#define ADXL375_REG_WINDOW         0x23
#define ADXL375_REG_THRESH_ACT     0x24
#define ADXL375_REG_THRESH_INACT   0x25
#define ADXL375_REG_TIME_INACT     0x26
#define ADXL375_REG_ACT_INACT_CTL  0x27
#define ADXL375_REG_SHOCK_AXES     0x2A
#define ADXL375_REG_ACT_SHOCK_STATUS 0x2B
#define ADXL375_REG_BW_RATE        0x2C
#define ADXL375_REG_POWER_CTL      0x2D
#define ADXL375_REG_INT_ENABLE     0x2E
#define ADXL375_REG_INT_MAP        0x2F
#define ADXL375_REG_INT_SOURCE     0x30
#define ADXL375_REG_DATA_FORMAT    0x31
#define ADXL375_REG_DATAX0         0x32
#define ADXL375_REG_DATAX1         0x33
#define ADXL375_REG_DATAY0         0x34
#define ADXL375_REG_DATAY1         0x35
#define ADXL375_REG_DATAZ0         0x36
#define ADXL375_REG_DATAZ1         0x37
#define ADXL375_REG_FIFO_CTL       0x38
#define ADXL375_REG_FIFO_STATUS    0x39

/* Device ID */
#define ADXL375_DEVID              0xE5

/* Power Control Register Bits */
#define ADXL375_PCTL_LINK          0x20
#define ADXL375_PCTL_AUTO_SLEEP    0x10
#define ADXL375_PCTL_MEASURE       0x08
#define ADXL375_PCTL_SLEEP         0x04
#define ADXL375_PCTL_WAKEUP_8HZ   0x00
#define ADXL375_PCTL_WAKEUP_4HZ   0x01
#define ADXL375_PCTL_WAKEUP_2HZ   0x02
#define ADXL375_PCTL_WAKEUP_1HZ   0x03

/* Data Format Register Bits */
#define ADXL375_DATAFORMAT_SELF_TEST   0x80
#define ADXL375_DATAFORMAT_SPI         0x40
#define ADXL375_DATAFORMAT_INT_INVERT  0x20
#define ADXL375_DATAFORMAT_FULL_RES    0x08
#define ADXL375_DATAFORMAT_JUSTIFY     0x04

/* Data Rate Options */
typedef enum {
    ADXL375_DATARATE_0_10_HZ = 0x00,
    ADXL375_DATARATE_0_20_HZ = 0x01,
    ADXL375_DATARATE_0_39_HZ = 0x02,
    ADXL375_DATARATE_0_78_HZ = 0x03,
    ADXL375_DATARATE_1_56_HZ = 0x04,
    ADXL375_DATARATE_3_13_HZ = 0x05,
    ADXL375_DATARATE_6_25HZ  = 0x06,
    ADXL375_DATARATE_12_5_HZ = 0x07,
    ADXL375_DATARATE_25_HZ   = 0x08,
    ADXL375_DATARATE_50_HZ   = 0x09,
    ADXL375_DATARATE_100_HZ  = 0x0A,
    ADXL375_DATARATE_200_HZ  = 0x0B,
    ADXL375_DATARATE_400_HZ  = 0x0C,
    ADXL375_DATARATE_800_HZ  = 0x0D,
    ADXL375_DATARATE_1600_HZ = 0x0E,
    ADXL375_DATARATE_3200_HZ = 0x0F
} adxl375_dataRate_t;

/* Sensor Info */
typedef struct {
    int32_t sensor_id;
    char    name[12];
    int32_t version;
    int32_t type;
    float   max_value;
    float   min_value;
    float   resolution;
} adxl375_sensor_t;

/* Acceleration Event */
typedef struct {
    float    x;
    float    y;
    float    z;
    uint32_t timestamp;
} adxl375_event_t;

/* Device Structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_address;
    adxl375_sensor_t sensor;

    bool  calibrated;
    float offset_x;
    float offset_y;
    float offset_z;

    float    v_x;
    float    v_y;
    float    v_z;
    uint32_t last_time;
    float    damping_factor;
    float    deadband;

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
} ADXL375_t;

/* Function Prototypes */

// Initialization
HAL_StatusTypeDef ADXL375_Init(ADXL375_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address);
bool ADXL375_Begin(ADXL375_t *dev);

// Configuration
HAL_StatusTypeDef  ADXL375_SetDataRate(ADXL375_t *dev, adxl375_dataRate_t dataRate);
HAL_StatusTypeDef  ADXL375_SetRange(ADXL375_t *dev, uint8_t range);
adxl375_dataRate_t ADXL375_GetDataRate(ADXL375_t *dev);

// Data Reading
HAL_StatusTypeDef ADXL375_ReadRaw(ADXL375_t *dev);
bool ADXL375_GetEvent(ADXL375_t *dev, adxl375_event_t *event);
void ADXL375_ProcessData(ADXL375_t *dev);

// Calibration
HAL_StatusTypeDef ADXL375_Calibrate(ADXL375_t *dev, uint16_t samples);
void ADXL375_SetOffsets(ADXL375_t *dev, float offset_x, float offset_y, float offset_z);
void ADXL375_GetOffsets(ADXL375_t *dev, float *offset_x, float *offset_y, float *offset_z);

// Velocity
void  ADXL375_ResetVelocity(ADXL375_t *dev);
void  ADXL375_GetVelocity(ADXL375_t *dev, float *vx, float *vy, float *vz);
float ADXL375_GetVelocityMagnitude(ADXL375_t *dev);
float ADXL375_GetVerticalVelocity(ADXL375_t *dev);   // NEW: Z-axis only

// Utility
void ADXL375_PrintDetails(ADXL375_t *dev);
void ADXL375_PrintDataRate(ADXL375_t *dev);
void ADXL375_GetSensor(ADXL375_t *dev, adxl375_sensor_t *sensor);

// Low-level I2C
HAL_StatusTypeDef ADXL375_WriteRegister(ADXL375_t *dev, uint8_t reg, uint8_t value);
HAL_StatusTypeDef ADXL375_ReadRegister(ADXL375_t *dev, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ADXL375_ReadRegisters(ADXL375_t *dev, uint8_t reg, uint8_t *buffer, uint8_t len);

/* Constants */
#define ADXL375_MG2G_MULTIPLIER    0.049f
#define ADXL375_GRAVITY_EARTH      9.80665f

#endif /* ADXL_H_ */
