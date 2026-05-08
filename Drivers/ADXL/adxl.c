/*
 * adxl.c - ADXL375 High-G Accelerometer Driver Implementation
 *
 *  Created on: Feb 12, 2026
 *      Author: YASH TATODE
 *  Fixed v2.0:
 *  - FIX BUG 1: ADXL375_ProcessData() now uses dev->offset_x/y/z correctly
 *    Previously: main.c computed adxl375_offset_x/y/z as separate globals and
 *                manually subtracted them in the log, BUT never called
 *                ADXL375_SetOffsets() to sync them into dev->offset_z.
 *                ProcessData() used dev->offset_z = 0.0 (init value) so it
 *                integrated raw Z (~9.8 m/s²) → velocity exploded to 1000+ m/s
 *    Fix applied in main.c: call ADXL375_SetOffsets() after Calibrate_ADXL375_Improved()
 *    Fix applied here: added gravity subtraction guard in ProcessData as safety net,
 *                      and improved deadband logic for stationary detection
 *
 *  - FIX: Deadband now applied AFTER gravity subtraction so it correctly
 *    rejects small noise around 0 rather than around 9.8
 */

#include "adxl.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern char uart_buf[300];

/* Initialize ADXL375 structure */
HAL_StatusTypeDef ADXL375_Init(ADXL375_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address) {
    dev->hi2c = hi2c;
    dev->i2c_address = address << 1;

    dev->sensor.sensor_id = 12345;
    strcpy(dev->sensor.name, "ADXL375");
    dev->sensor.version   = 3;
    dev->sensor.type      = 1;
    dev->sensor.max_value = 200.0f;
    dev->sensor.min_value = -200.0f;
    dev->sensor.resolution = ADXL375_MG2G_MULTIPLIER;

    dev->calibrated = false;
    dev->offset_x   = 0.0f;
    dev->offset_y   = 0.0f;
    dev->offset_z   = 0.0f;

    dev->v_x = 0.0f;
    dev->v_y = 0.0f;
    dev->v_z = 0.0f;
    dev->last_time      = HAL_GetTick();
    dev->damping_factor = 0.99990f;
    dev->deadband       = 0.50f;

    return HAL_OK;
}

/* Begin communication with ADXL375 */
bool ADXL375_Begin(ADXL375_t *dev) {
    uint8_t devid;

    if (ADXL375_ReadRegister(dev, ADXL375_REG_DEVID, &devid) != HAL_OK) {
        return false;
    }

    if (devid != ADXL375_DEVID) {
        sprintf(uart_buf, "ERROR: Wrong device ID! Expected 0x%02X, got 0x%02X\r\n",
                ADXL375_DEVID, devid);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
        return false;
    }

    if (ADXL375_WriteRegister(dev, ADXL375_REG_POWER_CTL, ADXL375_PCTL_MEASURE) != HAL_OK) {
        return false;
    }

    if (ADXL375_WriteRegister(dev, ADXL375_REG_DATA_FORMAT, ADXL375_DATAFORMAT_FULL_RES) != HAL_OK) {
        return false;
    }

    if (ADXL375_SetDataRate(dev, ADXL375_DATARATE_400_HZ) != HAL_OK) {
        return false;
    }

    HAL_Delay(10);
    return true;
}

/* Set data rate */
HAL_StatusTypeDef ADXL375_SetDataRate(ADXL375_t *dev, adxl375_dataRate_t dataRate) {
    return ADXL375_WriteRegister(dev, ADXL375_REG_BW_RATE, dataRate);
}

/* Get data rate */
adxl375_dataRate_t ADXL375_GetDataRate(ADXL375_t *dev) {
    uint8_t rate;
    ADXL375_ReadRegister(dev, ADXL375_REG_BW_RATE, &rate);
    return (adxl375_dataRate_t)(rate & 0x0F);
}

/* Read raw acceleration data */
HAL_StatusTypeDef ADXL375_ReadRaw(ADXL375_t *dev) {
    uint8_t buffer[6];

    if (ADXL375_ReadRegisters(dev, ADXL375_REG_DATAX0, buffer, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    dev->raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    dev->raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    dev->raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return HAL_OK;
}

/* Get acceleration event (in m/s²) */
bool ADXL375_GetEvent(ADXL375_t *dev, adxl375_event_t *event) {
    if (ADXL375_ReadRaw(dev) != HAL_OK) {
        return false;
    }

    event->x = (float)dev->raw_x * ADXL375_MG2G_MULTIPLIER * ADXL375_GRAVITY_EARTH;
    event->y = (float)dev->raw_y * ADXL375_MG2G_MULTIPLIER * ADXL375_GRAVITY_EARTH;
    event->z = (float)dev->raw_z * ADXL375_MG2G_MULTIPLIER * ADXL375_GRAVITY_EARTH;
    event->timestamp = HAL_GetTick();

    return true;
}

// ============================================================================
// FIX BUG 1: ADXL375_ProcessData velocity integration
//
// ROOT CAUSE: dev->offset_z was never synced from main.c's adxl375_offset_z global.
// main.c computes offsets manually and calls ADXL375_SetOffsets() after calibration
// (fix in main.c). This function now also correctly:
//   1. Subtracts calibration offsets (dev->offset_x/y/z) from raw reading
//   2. Subtracts gravity from Z (net_z = accel_z - 9.81) before integration
//   3. Applies deadband around ZERO (not around 9.8)
//   4. Only then integrates net acceleration into velocity
//
// With ADXL375_SetOffsets() called in main.c, dev->offset_z ≈ avg_z - 9.81
// so after step 1: accel_z ≈ 9.81 at rest
// after step 2: net_z ≈ 0 at rest → deadband kills it → velocity stays 0 ✅
// ============================================================================
void ADXL375_ProcessData(ADXL375_t *dev) {
    adxl375_event_t event;

    if (!ADXL375_GetEvent(dev, &event)) {
        return;
    }

    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - dev->last_time) / 1000.0f;
    dev->last_time = current_time;

    if (dt > 0.5f || dt < 0.001f) {
        dt = 0.01f;
    }

    // Step 1: Remove sensor calibration bias (dev->offset set by ADXL375_SetOffsets)
    float accel_x = event.x - dev->offset_x;
    float accel_y = event.y - dev->offset_y;
    float accel_z = event.z - dev->offset_z;

    // Step 2: Remove gravity from Z axis to get net vertical acceleration
    // After offset removal, accel_z ≈ +9.81 at rest (gravity pointing down = Z up)
    // Subtract gravity so net_z ≈ 0 at rest, positive = accelerating upward
    float net_z = accel_z - ADXL375_GRAVITY_EARTH;

    // Step 3: Apply deadband around ZERO (after gravity removal)
    // This prevents tiny residual noise from accumulating into velocity drift
    if (fabsf(accel_x) < dev->deadband) accel_x = 0.0f;
    if (fabsf(accel_y) < dev->deadband) accel_y = 0.0f;
    if (fabsf(net_z)   < dev->deadband) net_z   = 0.0f;

    // Step 4: Integrate net acceleration into velocity
    dev->v_x += accel_x * dt;
    dev->v_y += accel_y * dt;
    dev->v_z += net_z   * dt;

    // Step 5: Apply damping to suppress long-term drift
    dev->v_x *= dev->damping_factor;
    dev->v_y *= dev->damping_factor;
    dev->v_z *= dev->damping_factor;
}

/* Calibrate ADXL375 */
HAL_StatusTypeDef ADXL375_Calibrate(ADXL375_t *dev, uint16_t samples) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t valid_samples = 0;
    adxl375_event_t event;

    sprintf(uart_buf, "Starting ADXL375 calibration with %d samples...\r\n", samples);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    for (uint16_t i = 0; i < samples; i++) {
        if (ADXL375_GetEvent(dev, &event)) {
            sum_x += event.x;
            sum_y += event.y;
            sum_z += event.z;
            valid_samples++;
        }
        HAL_Delay(10);
    }

    if (valid_samples < (samples / 2)) {
        sprintf(uart_buf, "ERROR: Only %d/%d valid samples!\r\n", valid_samples, samples);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
        return HAL_ERROR;
    }

    float avg_x = sum_x / valid_samples;
    float avg_y = sum_y / valid_samples;
    float avg_z = sum_z / valid_samples;

    // X/Y offsets = raw bias (expected ~0)
    // Z offset = raw bias minus gravity so after removal accel_z ≈ +9.81 at rest
    dev->offset_x = avg_x;
    dev->offset_y = avg_y;
    dev->offset_z = avg_z - ADXL375_GRAVITY_EARTH;

    dev->calibrated = true;

    sprintf(uart_buf, "Calibration complete! Offsets: X=%.3f Y=%.3f Z=%.3f m/s²\r\n",
            dev->offset_x, dev->offset_y, dev->offset_z);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    return HAL_OK;
}

/* Set calibration offsets manually */
void ADXL375_SetOffsets(ADXL375_t *dev, float offset_x, float offset_y, float offset_z) {
    dev->offset_x   = offset_x;
    dev->offset_y   = offset_y;
    dev->offset_z   = offset_z;
    dev->calibrated = true;
}

/* Get calibration offsets */
void ADXL375_GetOffsets(ADXL375_t *dev, float *offset_x, float *offset_y, float *offset_z) {
    *offset_x = dev->offset_x;
    *offset_y = dev->offset_y;
    *offset_z = dev->offset_z;
}

/* Reset velocity integrator */
void ADXL375_ResetVelocity(ADXL375_t *dev) {
    dev->v_x = 0.0f;
    dev->v_y = 0.0f;
    dev->v_z = 0.0f;
    dev->last_time = HAL_GetTick();
}

/* Get velocity components */
void ADXL375_GetVelocity(ADXL375_t *dev, float *vx, float *vy, float *vz) {
    *vx = dev->v_x;
    *vy = dev->v_y;
    *vz = dev->v_z;
}

/* Get velocity magnitude */
float ADXL375_GetVelocityMagnitude(ADXL375_t *dev) {
    return sqrtf(dev->v_x * dev->v_x + dev->v_y * dev->v_y + dev->v_z * dev->v_z);
}

/* Get vertical velocity (Z only) */
float ADXL375_GetVerticalVelocity(ADXL375_t *dev) {
    return dev->v_z;
}

/* Print sensor details */
void ADXL375_PrintDetails(ADXL375_t *dev) {
    sprintf(uart_buf, "------------------------------------\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Sensor:       %s\r\n", dev->sensor.name);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Driver Ver:   %ld.0 (Fixed v2.0)\r\n", dev->sensor.version);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Unique ID:    %ld\r\n", dev->sensor.sensor_id);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Max Value:    %.2f g\r\n", dev->sensor.max_value);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Min Value:    %.2f g\r\n", dev->sensor.min_value);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Resolution:   %.3f g/LSB\r\n", dev->sensor.resolution);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Vel Damping:  %.5f\r\n", dev->damping_factor);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Deadband:     %.3f m/s² (%.4f g)\r\n",
            dev->deadband, dev->deadband / ADXL375_GRAVITY_EARTH);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "Calibrated:   %s\r\n", dev->calibrated ? "YES" : "NO");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    sprintf(uart_buf, "------------------------------------\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
}

/* Print data rate */
void ADXL375_PrintDataRate(ADXL375_t *dev) {
    adxl375_dataRate_t rate = ADXL375_GetDataRate(dev);
    const char* rate_strings[] = {
        "0.10 Hz", "0.20 Hz", "0.39 Hz", "0.78 Hz",
        "1.56 Hz", "3.13 Hz", "6.25 Hz", "12.5 Hz",
        "25 Hz", "50 Hz", "100 Hz", "200 Hz",
        "400 Hz", "800 Hz", "1600 Hz", "3200 Hz"
    };
    sprintf(uart_buf, "Data Rate: %s\r\n", rate_strings[rate]);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
}

/* Get sensor info */
void ADXL375_GetSensor(ADXL375_t *dev, adxl375_sensor_t *sensor) {
    memcpy(sensor, &dev->sensor, sizeof(adxl375_sensor_t));
}

/* Low-level I2C Functions */

HAL_StatusTypeDef ADXL375_WriteRegister(ADXL375_t *dev, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_address, data, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL375_ReadRegister(ADXL375_t *dev, uint8_t reg, uint8_t *value) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_address, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_address, value, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL375_ReadRegisters(ADXL375_t *dev, uint8_t reg, uint8_t *buffer, uint8_t len) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_address, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_address, buffer, len, HAL_MAX_DELAY);
}
