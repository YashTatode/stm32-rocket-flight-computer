/*
 * icm.c
 *
 *  Created on: Dec 27, 2025
 *      Author: YASH TATODE
 *  Fixed v2.0:
 *  - FIX BUG 4: SetGyroFullScale() and SetAccelFullScale() now do read-modify-write
 *    Previously: wrote (fs << 1 | 0x01) directly, zeroing out DLPF bits [5:3]
 *                Any future call to SetGyroFullScale() would silently reset DLPF
 *                to 196.6Hz (DLPF_CFG=0), making the sensor much noisier
 *    Now: reads existing register, preserves DLPF bits, only changes FS and FCHOICE
 */

#include "icm.h"

#define ICM20948_I2C_TIMEOUT    100

static const float GYRO_SENSITIVITY[]  = {131.0f, 65.5f, 32.8f, 16.4f};
static const float ACCEL_SENSITIVITY[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};

#define TEMP_SENSITIVITY        333.87f

// ============================================================================
// LOW-LEVEL REGISTER ACCESS
// ============================================================================

HAL_StatusTypeDef ICM20948_SelectBank(ICM20948_t *dev, uint8_t bank) {
    if (dev->current_bank == bank) {
        return HAL_OK;
    }

    uint8_t reg_val = (bank << 4) & 0x30;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->address,
                                                   REG_BANK_SEL, 1,
                                                   &reg_val, 1,
                                                   ICM20948_I2C_TIMEOUT);
    if (status == HAL_OK) {
        dev->current_bank = bank;
        HAL_Delay(1);
    }

    return status;
}

HAL_StatusTypeDef ICM20948_ReadRegister(ICM20948_t *dev, uint8_t bank,
                                        uint8_t reg, uint8_t *data) {
    HAL_StatusTypeDef status = ICM20948_SelectBank(dev, bank);
    if (status != HAL_OK) return status;

    return HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, 1,
                            data, 1, ICM20948_I2C_TIMEOUT);
}

HAL_StatusTypeDef ICM20948_WriteRegister(ICM20948_t *dev, uint8_t bank,
                                         uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef status = ICM20948_SelectBank(dev, bank);
    if (status != HAL_OK) return status;

    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, 1,
                             &data, 1, ICM20948_I2C_TIMEOUT);
}

HAL_StatusTypeDef ICM20948_ReadRegisters(ICM20948_t *dev, uint8_t bank,
                                         uint8_t reg, uint8_t *data,
                                         uint16_t length) {
    HAL_StatusTypeDef status = ICM20948_SelectBank(dev, bank);
    if (status != HAL_OK) return status;

    return HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, 1,
                            data, length, ICM20948_I2C_TIMEOUT);
}

// ============================================================================
// DEVICE CONTROL
// ============================================================================

HAL_StatusTypeDef ICM20948_VerifyConnection(ICM20948_t *dev) {
    uint8_t who_am_i;
    HAL_StatusTypeDef status = ICM20948_ReadRegister(dev, 0, UB0_WHO_AM_I, &who_am_i);
    if (status != HAL_OK) return status;
    return (who_am_i == ICM20948_WHO_AM_I_VAL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef ICM20948_Reset(ICM20948_t *dev) {
    HAL_StatusTypeDef status = ICM20948_WriteRegister(dev, 0, UB0_PWR_MGMT_1,
                                                       PWR_MGMT_1_RESET);
    if (status != HAL_OK) return status;
    HAL_Delay(100);
    dev->current_bank = 0xFF;  // Force re-select after reset
    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_WakeUp(ICM20948_t *dev) {
    return ICM20948_WriteRegister(dev, 0, UB0_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO);
}

HAL_StatusTypeDef ICM20948_Sleep(ICM20948_t *dev) {
    return ICM20948_WriteRegister(dev, 0, UB0_PWR_MGMT_1,
                                  PWR_MGMT_1_SLEEP | PWR_MGMT_1_CLKSEL_AUTO);
}

// ============================================================================
// CONFIGURATION
// ============================================================================

// ============================================================================
// FIX BUG 4: SetGyroFullScale - read-modify-write to preserve DLPF bits
// Old code: reg_val = (fs << 1) | 0x01  → zeros DLPF_CFG[5:3] silently
// New code: read register first, preserve bits [7:6] and [5:3], only change [2:1] and [0]
// Register UB2_GYRO_CONFIG_1 layout:
//   [7:6] = reserved
//   [5:3] = DLPF_CFG
//   [2:1] = GYRO_FS_SEL
//   [0]   = GYRO_FCHOICE (1 = enable DLPF)
// ============================================================================
HAL_StatusTypeDef ICM20948_SetGyroFullScale(ICM20948_t *dev, ICM20948_GyroFS_t fs) {
    uint8_t reg_val;
    HAL_StatusTypeDef status = ICM20948_ReadRegister(dev, 2, UB2_GYRO_CONFIG_1, &reg_val);
    if (status != HAL_OK) return status;

    // Preserve DLPF bits [5:3], update FS_SEL [2:1] and FCHOICE [0]
    reg_val = (reg_val & 0xC7) | ((fs & 0x03) << 1) | 0x01;
    status = ICM20948_WriteRegister(dev, 2, UB2_GYRO_CONFIG_1, reg_val);

    if (status == HAL_OK) {
        dev->gyro_fs    = fs;
        dev->gyro_scale = GYRO_SENSITIVITY[fs];
    }

    return status;
}

// ============================================================================
// FIX BUG 4: SetAccelFullScale - read-modify-write to preserve DLPF bits
// Register UB2_ACCEL_CONFIG layout:
//   [7:6] = reserved
//   [5:3] = DLPF_CFG
//   [2:1] = ACCEL_FS_SEL
//   [0]   = ACCEL_FCHOICE (1 = enable DLPF)
// ============================================================================
HAL_StatusTypeDef ICM20948_SetAccelFullScale(ICM20948_t *dev, ICM20948_AccelFS_t fs) {
    uint8_t reg_val;
    HAL_StatusTypeDef status = ICM20948_ReadRegister(dev, 2, UB2_ACCEL_CONFIG, &reg_val);
    if (status != HAL_OK) return status;

    // Preserve DLPF bits [5:3], update FS_SEL [2:1] and FCHOICE [0]
    reg_val = (reg_val & 0xC7) | ((fs & 0x03) << 1) | 0x01;
    status = ICM20948_WriteRegister(dev, 2, UB2_ACCEL_CONFIG, reg_val);

    if (status == HAL_OK) {
        dev->accel_fs    = fs;
        dev->accel_scale = ACCEL_SENSITIVITY[fs];
    }

    return status;
}

HAL_StatusTypeDef ICM20948_SetGyroDLPF(ICM20948_t *dev, ICM20948_DLPF_t dlpf) {
    uint8_t reg_val;
    HAL_StatusTypeDef status = ICM20948_ReadRegister(dev, 2, UB2_GYRO_CONFIG_1, &reg_val);
    if (status != HAL_OK) return status;

    reg_val = (reg_val & 0xC7) | ((dlpf & 0x07) << 3);
    return ICM20948_WriteRegister(dev, 2, UB2_GYRO_CONFIG_1, reg_val);
}

HAL_StatusTypeDef ICM20948_SetAccelDLPF(ICM20948_t *dev, ICM20948_DLPF_t dlpf) {
    uint8_t reg_val;
    HAL_StatusTypeDef status = ICM20948_ReadRegister(dev, 2, UB2_ACCEL_CONFIG, &reg_val);
    if (status != HAL_OK) return status;

    reg_val = (reg_val & 0xC7) | ((dlpf & 0x07) << 3);
    return ICM20948_WriteRegister(dev, 2, UB2_ACCEL_CONFIG, reg_val);
}

HAL_StatusTypeDef ICM20948_SetSampleRateDivider(ICM20948_t *dev,
                                                uint8_t gyro_div,
                                                uint16_t accel_div) {
    HAL_StatusTypeDef status;

    status = ICM20948_WriteRegister(dev, 2, UB2_GYRO_SMPLRT_DIV, gyro_div);
    if (status != HAL_OK) return status;

    status = ICM20948_WriteRegister(dev, 2, UB2_ACCEL_SMPLRT_DIV_1,
                                    (accel_div >> 8) & 0x0F);
    if (status != HAL_OK) return status;

    return ICM20948_WriteRegister(dev, 2, UB2_ACCEL_SMPLRT_DIV_2,
                                    accel_div & 0xFF);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

HAL_StatusTypeDef ICM20948_Init(ICM20948_t *dev, I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;

    dev->hi2c = hi2c;
    dev->address = ICM20948_ADDR;
    dev->current_bank = 0xFF;
    dev->calib_data.is_calibrated = false;

    HAL_Delay(100);

    status = ICM20948_VerifyConnection(dev);
    if (status != HAL_OK) return status;

    status = ICM20948_Reset(dev);
    if (status != HAL_OK) return status;

    status = ICM20948_WakeUp(dev);
    if (status != HAL_OK) return status;
    HAL_Delay(50);

    // Enable accel + gyro
    status = ICM20948_WriteRegister(dev, 0, UB0_PWR_MGMT_2, 0x00);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Configure gyroscope: ±2000 dps, DLPF 51.2 Hz
    // NOTE: SetGyroFullScale first (initialises FCHOICE=1), then SetGyroDLPF
    //       Both now use read-modify-write so ordering is safe either way
    status = ICM20948_SetGyroFullScale(dev, GYRO_FS_2000DPS);
    if (status != HAL_OK) return status;

    status = ICM20948_SetGyroDLPF(dev, DLPF_3);
    if (status != HAL_OK) return status;

    // Configure accelerometer: ±4g, DLPF 50.4 Hz
    status = ICM20948_SetAccelFullScale(dev, ACCEL_FS_16G);
    if (status != HAL_OK) return status;

    status = ICM20948_SetAccelDLPF(dev, DLPF_3);
    if (status != HAL_OK) return status;

    // Sample rate: ~102 Hz for both gyro and accel
    status = ICM20948_SetSampleRateDivider(dev, 10, 10);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

// ============================================================================
// DATA READING
// ============================================================================

HAL_StatusTypeDef ICM20948_ReadRawData(ICM20948_t *dev) {
    uint8_t buffer[14];

    HAL_StatusTypeDef status = ICM20948_ReadRegisters(dev, 0, UB0_ACCEL_XOUT_H,
                                                      buffer, 14);
    if (status != HAL_OK) return status;

    dev->raw_data.accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    dev->raw_data.accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    dev->raw_data.accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    dev->raw_data.gyro_x  = (int16_t)((buffer[6]  << 8) | buffer[7]);
    dev->raw_data.gyro_y  = (int16_t)((buffer[8]  << 8) | buffer[9]);
    dev->raw_data.gyro_z  = (int16_t)((buffer[10] << 8) | buffer[11]);

    dev->raw_data.temp    = (int16_t)((buffer[12] << 8) | buffer[13]);

    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_ReadScaledData(ICM20948_t *dev) {
    HAL_StatusTypeDef status = ICM20948_ReadRawData(dev);
    if (status != HAL_OK) return status;

    // Convert to m/s²
    dev->scaled_data.accel_x = ((float)dev->raw_data.accel_x / dev->accel_scale) * 9.81f;
    dev->scaled_data.accel_y = ((float)dev->raw_data.accel_y / dev->accel_scale) * 9.81f;
    dev->scaled_data.accel_z = ((float)dev->raw_data.accel_z / dev->accel_scale) * 9.81f;

    // Convert to deg/s
    dev->scaled_data.gyro_x  = (float)dev->raw_data.gyro_x / dev->gyro_scale;
    dev->scaled_data.gyro_y  = (float)dev->raw_data.gyro_y / dev->gyro_scale;
    dev->scaled_data.gyro_z  = (float)dev->raw_data.gyro_z / dev->gyro_scale;

    // Datasheet formula: Temp_degC = TEMP_OUT/333.87 + 21.0
    dev->scaled_data.temp = ((float)dev->raw_data.temp / TEMP_SENSITIVITY) + 21.0f;

    // Apply calibration offsets if available
    if (dev->calib_data.is_calibrated) {
        ICM20948_ApplyCalibration(dev);
    }

    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_ReadTemperature(ICM20948_t *dev, float *temp) {
    uint8_t buffer[2];
    HAL_StatusTypeDef status = ICM20948_ReadRegisters(dev, 0, UB0_TEMP_OUT_H, buffer, 2);
    if (status != HAL_OK) return status;

    int16_t temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    *temp = ((float)temp_raw / TEMP_SENSITIVITY) + 21.0f;

    return HAL_OK;
}

// ============================================================================
// CALIBRATION
// ============================================================================

HAL_StatusTypeDef ICM20948_CalibrateGyro(ICM20948_t *dev, uint16_t samples) {
    if (samples < 100) samples = 100;

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < samples; i++) {
        HAL_StatusTypeDef status = ICM20948_ReadRawData(dev);
        if (status != HAL_OK) return status;

        sum_x += (float)dev->raw_data.gyro_x / dev->gyro_scale;
        sum_y += (float)dev->raw_data.gyro_y / dev->gyro_scale;
        sum_z += (float)dev->raw_data.gyro_z / dev->gyro_scale;

        HAL_Delay(10);
    }

    dev->calib_data.gyro_offset_x = sum_x / samples;
    dev->calib_data.gyro_offset_y = sum_y / samples;
    dev->calib_data.gyro_offset_z = sum_z / samples;
    dev->calib_data.is_calibrated = true;

    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_CalibrateAccel(ICM20948_t *dev, uint16_t samples) {
    if (samples < 100) samples = 100;

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < samples; i++) {
        HAL_StatusTypeDef status = ICM20948_ReadRawData(dev);
        if (status != HAL_OK) return status;

        sum_x += ((float)dev->raw_data.accel_x / dev->accel_scale) * 9.81f;
        sum_y += ((float)dev->raw_data.accel_y / dev->accel_scale) * 9.81f;
        sum_z += ((float)dev->raw_data.accel_z / dev->accel_scale) * 9.81f;

        HAL_Delay(10);
    }

    // X and Y should be 0 at rest; Z should be +9.81 (gravity up)
    // Offsets store the sensor bias so ApplyCalibration can subtract it
    dev->calib_data.accel_offset_x = sum_x / samples;           // bias only (expected ~0)
    dev->calib_data.accel_offset_y = sum_y / samples;           // bias only (expected ~0)
    dev->calib_data.accel_offset_z = (sum_z / samples) - 9.81f; // bias only (removes gravity so offset ≈ 0)
    dev->calib_data.is_calibrated = true;

    return HAL_OK;
}

// ApplyCalibration removes sensor bias only.
// After this, accel_z still reads ~9.81 (gravity is intentionally kept).
// Fusion subtracts 9.81 separately to get net vertical acceleration.
void ICM20948_ApplyCalibration(ICM20948_t *dev) {
    dev->scaled_data.accel_x -= dev->calib_data.accel_offset_x;
    dev->scaled_data.accel_y -= dev->calib_data.accel_offset_y;
    dev->scaled_data.accel_z -= dev->calib_data.accel_offset_z;

    dev->scaled_data.gyro_x  -= dev->calib_data.gyro_offset_x;
    dev->scaled_data.gyro_y  -= dev->calib_data.gyro_offset_y;
    dev->scaled_data.gyro_z  -= dev->calib_data.gyro_offset_z;
}

HAL_StatusTypeDef ICM20948_SetCalibrationData(ICM20948_t *dev,
                                              ICM20948_CalibData_t *calib) {
    dev->calib_data = *calib;
    dev->calib_data.is_calibrated = true;
    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_GetCalibrationData(ICM20948_t *dev,
                                              ICM20948_CalibData_t *calib) {
    *calib = dev->calib_data;
    return HAL_OK;
}
