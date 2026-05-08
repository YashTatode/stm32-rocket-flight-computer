/*
 * dps310.h
 *
 *  Created on: Dec 9, 2025
 *      Author: YASH TATODE
 *  Fixed v3.0:
 *  - Added float coefficients for higher precision
 *  - Added configurable oversampling rates
 *  - Improved BUSY handling
 */

#ifndef DPS310_H_
#define DPS310_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Oversampling configuration presets
typedef enum {
    DPS310_OVERSAMPLE_1X   = 0,  // Fast:   3.6ms, ±5 Pa
    DPS310_OVERSAMPLE_2X   = 1,  //         5.2ms, ±2.5 Pa
    DPS310_OVERSAMPLE_4X   = 2,  //         8.4ms, ±1.2 Pa
    DPS310_OVERSAMPLE_8X   = 3,  // Medium: 14.8ms, ±0.9 Pa
    DPS310_OVERSAMPLE_16X  = 4,  //         27.6ms, ±0.5 Pa
    DPS310_OVERSAMPLE_32X  = 5,  //         53.2ms, ±0.2 Pa
    DPS310_OVERSAMPLE_64X  = 6,  // Precise: 104.4ms, ±0.12 Pa
    DPS310_OVERSAMPLE_128X = 7   // Max:     206.8ms, ±0.06 Pa
} DPS310_Oversample_t;

// Measurement rate configuration
typedef enum {
    DPS310_RATE_1_HZ   = 0,  // 1 measurement/second
    DPS310_RATE_2_HZ   = 1,  // 2 measurements/second
    DPS310_RATE_4_HZ   = 2,  // 4 measurements/second
    DPS310_RATE_8_HZ   = 3,  // 8 measurements/second
    DPS310_RATE_16_HZ  = 4,  // 16 measurements/second
    DPS310_RATE_32_HZ  = 5,  // 32 measurements/second
    DPS310_RATE_64_HZ  = 6,  // 64 measurements/second
    DPS310_RATE_128_HZ = 7   // 128 measurements/second
} DPS310_Rate_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;

    // Integer calibration coefficients (from sensor)
    int32_t c0, c1, c00, c10, c01, c11, c20, c21, c30;

    // Float coefficients (pre-converted for precision)
    float c0_f, c1_f, c00_f, c10_f, c01_f, c11_f, c20_f, c21_f, c30_f;

    // Integer scale factors
    int32_t kT;
    int32_t kP;

    // Float scale factors (pre-converted for precision)
    float kT_f;
    float kP_f;

    // Configuration
    DPS310_Oversample_t pressure_oversample;
    DPS310_Oversample_t temperature_oversample;
    DPS310_Rate_t pressure_rate;
    DPS310_Rate_t temperature_rate;

    // Last valid readings (used when BUSY)
    float last_temperature;
    float last_pressure;

    // Statistics
    uint32_t busy_count;
    uint32_t ok_count;
    uint32_t error_count;
} DPS310_t;

typedef enum {
    DPS310_OK    = 0,
    DPS310_ERROR = 1,
    DPS310_BUSY  = 2   // New measurement not ready yet
} DPS310_Status;

// Initialize DPS310 with HAL I2C and address (0x77 or 0x76)
// Uses default config: 16x oversampling @ 16 Hz (balanced for 100Hz loop)
DPS310_Status DPS310_Init(DPS310_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address);

// Initialize with custom oversampling and rate
DPS310_Status DPS310_InitCustom(DPS310_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address,
                                DPS310_Oversample_t prs_oversample, DPS310_Rate_t prs_rate,
                                DPS310_Oversample_t tmp_oversample, DPS310_Rate_t tmp_rate);

// Read compensated temperature (°C) and pressure (hPa)
// Returns DPS310_OK if new data read, DPS310_BUSY if no new data (returns last values)
DPS310_Status DPS310_ReadTemperaturePressure(DPS310_t *dev, float *temperature, float *pressure);

// Calculate altitude (meters) from pressure relative to P0
float DPS310_CalculateAltitude(float pressure, float P0);

// Get statistics
void DPS310_GetStatistics(DPS310_t *dev, uint32_t *ok_count, uint32_t *busy_count,
                          uint32_t *error_count, float *busy_percentage);

// Reset statistics counters
void DPS310_ResetStatistics(DPS310_t *dev);

#endif
