/*
 * dps310.c
 *
 *  Created on: Dec 9, 2025
 *      Author: YASH TATODE
 *  Fixed v3.0:
 *  - Float coefficient support for higher precision
 *  - Configurable oversampling (default 16x @ 16Hz for 100Hz loop)
 *  - Improved compensation calculations
 *  - Statistics tracking for debugging
 */

#include "dps310.h"
#include <math.h>
#include <string.h>

// Register addresses
#define DPS310_REG_PSR_B2       0x00
#define DPS310_REG_PSR_B1       0x01
#define DPS310_REG_PSR_B0       0x02
#define DPS310_REG_TMP_B2       0x03
#define DPS310_REG_TMP_B1       0x04
#define DPS310_REG_TMP_B0       0x05
#define DPS310_REG_PRS_CFG      0x06
#define DPS310_REG_TMP_CFG      0x07
#define DPS310_REG_MEAS_CFG     0x08
#define DPS310_REG_CFG_REG      0x09
#define DPS310_REG_RESET        0x0C
#define DPS310_REG_PRODUCT_ID   0x0D
#define DPS310_REG_COEF         0x10
#define DPS310_REG_COEF_SRCE    0x28

// MEAS_CFG status bits
#define DPS310_MEAS_PRS_RDY     0x10   // Bit 4: new pressure result ready
#define DPS310_MEAS_TMP_RDY     0x20   // Bit 5: new temperature result ready
#define DPS310_MEAS_SENSOR_RDY  0x40   // Bit 6: sensor init complete
#define DPS310_MEAS_COEF_RDY    0x80   // Bit 7: coefficients ready

// Write a single register
static HAL_StatusTypeDef DPS310_WriteReg(DPS310_t *dev, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address << 1, reg, 1, &data, 1, HAL_MAX_DELAY);
}

// Read multiple registers
static HAL_StatusTypeDef DPS310_ReadRegs(DPS310_t *dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    return HAL_I2C_Mem_Read(dev->hi2c, dev->address << 1, reg, 1, buf, len, HAL_MAX_DELAY);
}

// Convert two's complement to signed integer
static int32_t twos_complement(uint32_t val, uint8_t bits) {
    if (val & ((uint32_t)1 << (bits - 1))) {
        return (int32_t)val - ((int32_t)1 << bits);
    }
    return (int32_t)val;
}

// Read and parse calibration coefficients
static DPS310_Status DPS310_ReadCalibrationCoefficients(DPS310_t *dev) {
    uint8_t coef[18];

    if (DPS310_ReadRegs(dev, DPS310_REG_COEF, coef, 18) != HAL_OK) {
        return DPS310_ERROR;
    }

    // Parse integer coefficients
    uint32_t c0 = ((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F);
    dev->c0 = twos_complement(c0, 12);

    uint32_t c1 = (((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2];
    dev->c1 = twos_complement(c1, 12);

    uint32_t c00 = ((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F);
    dev->c00 = twos_complement(c00, 20);

    uint32_t c10 = (((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7];
    dev->c10 = twos_complement(c10, 20);

    uint32_t c01 = ((uint32_t)coef[8] << 8) | (uint32_t)coef[9];
    dev->c01 = twos_complement(c01, 16);

    uint32_t c11 = ((uint32_t)coef[10] << 8) | (uint32_t)coef[11];
    dev->c11 = twos_complement(c11, 16);

    uint32_t c20 = ((uint32_t)coef[12] << 8) | (uint32_t)coef[13];
    dev->c20 = twos_complement(c20, 16);

    uint32_t c21 = ((uint32_t)coef[14] << 8) | (uint32_t)coef[15];
    dev->c21 = twos_complement(c21, 16);

    uint32_t c30 = ((uint32_t)coef[16] << 8) | (uint32_t)coef[17];
    dev->c30 = twos_complement(c30, 16);

    // Convert to float for higher precision calculations
    dev->c0_f  = (float)dev->c0;
    dev->c1_f  = (float)dev->c1;
    dev->c00_f = (float)dev->c00;
    dev->c10_f = (float)dev->c10;
    dev->c01_f = (float)dev->c01;
    dev->c11_f = (float)dev->c11;
    dev->c20_f = (float)dev->c20;
    dev->c21_f = (float)dev->c21;
    dev->c30_f = (float)dev->c30;

    return DPS310_OK;
}

// Get scale factors based on oversampling settings
static void DPS310_GetScaleFactors(uint8_t oversampling, int32_t *kP_or_kT) {
    switch (oversampling) {
        case 0: *kP_or_kT = 524288;  break;   // 1x
        case 1: *kP_or_kT = 1572864; break;   // 2x
        case 2: *kP_or_kT = 3670016; break;   // 4x
        case 3: *kP_or_kT = 7864320; break;   // 8x
        case 4: *kP_or_kT = 253952;  break;   // 16x
        case 5: *kP_or_kT = 516096;  break;   // 32x
        case 6: *kP_or_kT = 1040384; break;   // 64x
        case 7: *kP_or_kT = 2088960; break;   // 128x
        default: *kP_or_kT = 524288; break;
    }
}

DPS310_Status DPS310_InitCustom(DPS310_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address,
                                DPS310_Oversample_t prs_oversample, DPS310_Rate_t prs_rate,
                                DPS310_Oversample_t tmp_oversample, DPS310_Rate_t tmp_rate) {
    dev->hi2c = hi2c;
    dev->address = address;
    dev->last_temperature = 25.0f;   // Safe defaults
    dev->last_pressure    = 1013.25f;
    dev->busy_count = 0;
    dev->ok_count = 0;
    dev->error_count = 0;

    dev->pressure_oversample = prs_oversample;
    dev->pressure_rate = prs_rate;
    dev->temperature_oversample = tmp_oversample;
    dev->temperature_rate = tmp_rate;

    // Check product ID
    uint8_t id;
    if (DPS310_ReadRegs(dev, DPS310_REG_PRODUCT_ID, &id, 1) != HAL_OK) {
        return DPS310_ERROR;
    }
    if ((id & 0xF0) != 0x10) {
        return DPS310_ERROR;
    }

    // Soft reset
    if (DPS310_WriteReg(dev, DPS310_REG_RESET, 0x09) != HAL_OK) {
        return DPS310_ERROR;
    }
    HAL_Delay(40);

    // Wait for sensor and coefficients to be ready (up to 200ms)
    uint8_t meas_status = 0;
    for (int i = 0; i < 20; i++) {
        if (DPS310_ReadRegs(dev, DPS310_REG_MEAS_CFG, &meas_status, 1) != HAL_OK) {
            return DPS310_ERROR;
        }
        if ((meas_status & (DPS310_MEAS_SENSOR_RDY | DPS310_MEAS_COEF_RDY)) ==
            (DPS310_MEAS_SENSOR_RDY | DPS310_MEAS_COEF_RDY)) {
            break;
        }
        HAL_Delay(10);
    }

    // Read coefficient source
    uint8_t coef_srce;
    if (DPS310_ReadRegs(dev, DPS310_REG_COEF_SRCE, &coef_srce, 1) != HAL_OK) {
        return DPS310_ERROR;
    }

    // Read calibration coefficients
    if (DPS310_ReadCalibrationCoefficients(dev) != DPS310_OK) {
        return DPS310_ERROR;
    }

    // Configure pressure: oversampling + rate
    // Register format: [6:4] = oversampling, [3:0] = rate
    uint8_t prs_cfg = ((prs_oversample & 0x07) << 4) | (prs_rate & 0x0F);
    if (DPS310_WriteReg(dev, DPS310_REG_PRS_CFG, prs_cfg) != HAL_OK) {
        return DPS310_ERROR;
    }

    // Configure temperature: oversampling + rate + coefficient source
    uint8_t tmp_cfg = ((tmp_oversample & 0x07) << 4) | (tmp_rate & 0x0F);
    if (coef_srce & 0x80) {
        tmp_cfg |= 0x80;  // Use external temperature sensor if needed
    }
    if (DPS310_WriteReg(dev, DPS310_REG_TMP_CFG, tmp_cfg) != HAL_OK) {
        return DPS310_ERROR;
    }

    // Get and store scale factors
    DPS310_GetScaleFactors(prs_oversample, &dev->kP);
    DPS310_GetScaleFactors(tmp_oversample, &dev->kT);

    // Convert to float for precision
    dev->kP_f = (float)dev->kP;
    dev->kT_f = (float)dev->kT;

    // Enable continuous pressure + temperature measurement
    if (DPS310_WriteReg(dev, DPS310_REG_MEAS_CFG, 0x07) != HAL_OK) {
        return DPS310_ERROR;
    }

    // Configure shift registers for oversampling > 8x
    // Bit 2: P_SHIFT (shift pressure if oversample > 8x)
    // Bit 3: T_SHIFT (shift temperature if oversample > 8x)
    uint8_t cfg_reg = 0x00;
    if (prs_oversample > DPS310_OVERSAMPLE_8X) cfg_reg |= 0x04;
    if (tmp_oversample > DPS310_OVERSAMPLE_8X) cfg_reg |= 0x08;

    if (DPS310_WriteReg(dev, DPS310_REG_CFG_REG, cfg_reg) != HAL_OK) {
        return DPS310_ERROR;
    }

    HAL_Delay(100);

    // Verify continuous mode is active
    uint8_t verify_meas;
    DPS310_ReadRegs(dev, DPS310_REG_MEAS_CFG, &verify_meas, 1);
    if ((verify_meas & 0x07) != 0x07) {
        // Mode not set correctly, retry
        DPS310_WriteReg(dev, DPS310_REG_MEAS_CFG, 0x07);
        HAL_Delay(50);
    }

    return DPS310_OK;
}

DPS310_Status DPS310_Init(DPS310_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address) {
    // Default configuration: 16x oversampling @ 16 Hz
    // This is balanced for a 100Hz main loop:
    // - Update every ~62.5ms (6 main loop cycles)
    // - Precision: ±0.5 Pa (0.01 hPa) ≈ 0.08m altitude
    // - ~83% BUSY, ~17% OK readings
    return DPS310_InitCustom(dev, hi2c, address,
                            DPS310_OVERSAMPLE_16X, DPS310_RATE_16_HZ,  // Pressure
                            DPS310_OVERSAMPLE_16X, DPS310_RATE_16_HZ); // Temperature
}

DPS310_Status DPS310_ReadTemperaturePressure(DPS310_t *dev, float *temperature, float *pressure) {
    uint8_t meas_status;

    // Read MEAS_CFG to check if new data is ready
    if (DPS310_ReadRegs(dev, DPS310_REG_MEAS_CFG, &meas_status, 1) != HAL_OK) {
        dev->error_count++;
        return DPS310_ERROR;
    }

    // Check if at least one sensor has new data
    bool prs_ready = (meas_status & DPS310_MEAS_PRS_RDY);
    bool tmp_ready = (meas_status & DPS310_MEAS_TMP_RDY);

    if (!prs_ready && !tmp_ready) {
        // No new data yet – return last known good values
        *temperature = dev->last_temperature;
        *pressure    = dev->last_pressure;
        dev->busy_count++;
        return DPS310_BUSY;
    }

    // Read raw sensor data (always read both even if only one is ready)
    uint8_t buf[6];
    if (DPS310_ReadRegs(dev, DPS310_REG_PSR_B2, buf, 6) != HAL_OK) {
        dev->error_count++;
        return DPS310_ERROR;
    }

    // Parse 24-bit pressure (two's complement)
    uint32_t psr_raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    int32_t Praw = twos_complement(psr_raw, 24);

    // Parse 24-bit temperature (two's complement)
    uint32_t tmp_raw = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];
    int32_t Traw = twos_complement(tmp_raw, 24);

    // =========================================================================
    // HIGH PRECISION COMPENSATION - Use float coefficients and scale factors
    // =========================================================================

    // Scale raw values using pre-converted float scale factors
    float Traw_sc = (float)Traw / dev->kT_f;
    float Praw_sc = (float)Praw / dev->kP_f;

    // Compensate temperature (°C) using float coefficients
    float temp_compensated = dev->c0_f * 0.5f + dev->c1_f * Traw_sc;

    // Compensate pressure (Pa) using float coefficients
    // This is a 3rd-order polynomial for high accuracy
    float Pcomp = dev->c00_f +
                  Praw_sc * (dev->c10_f +
                            Praw_sc * (dev->c20_f +
                                      Praw_sc * dev->c30_f)) +
                  Traw_sc * (dev->c01_f +
                            Praw_sc * (dev->c11_f +
                                      Praw_sc * dev->c21_f));

    // Convert Pa to hPa
    float pressure_compensated = Pcomp / 100.0f;

    // Update outputs
    // If only pressure ready, use new pressure + last temperature
    // If only temperature ready, use new temperature + last pressure
    // If both ready, use both new values
    if (prs_ready) {
        *pressure = pressure_compensated;
        dev->last_pressure = pressure_compensated;
    } else {
        *pressure = dev->last_pressure;
    }

    if (tmp_ready) {
        *temperature = temp_compensated;
        dev->last_temperature = temp_compensated;
    } else {
        *temperature = dev->last_temperature;
    }

    dev->ok_count++;
    return DPS310_OK;
}

float DPS310_CalculateAltitude(float pressure, float P0) {
    if (P0 <= 0 || pressure <= 0) {
        return 0.0f;
    }
    // Barometric formula
    return 44330.0f * (1.0f - powf(pressure / P0, 0.1903f));
}

void DPS310_GetStatistics(DPS310_t *dev, uint32_t *ok_count, uint32_t *busy_count,
                          uint32_t *error_count, float *busy_percentage) {
    if (ok_count) *ok_count = dev->ok_count;
    if (busy_count) *busy_count = dev->busy_count;
    if (error_count) *error_count = dev->error_count;

    if (busy_percentage) {
        uint32_t total = dev->ok_count + dev->busy_count;
        if (total > 0) {
            *busy_percentage = 100.0f * (float)dev->busy_count / (float)total;
        } else {
            *busy_percentage = 0.0f;
        }
    }
}

void DPS310_ResetStatistics(DPS310_t *dev) {
    dev->ok_count = 0;
    dev->busy_count = 0;
    dev->error_count = 0;
}
