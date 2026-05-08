/*
 * main.c — Rocket Flight Computer
 *
 * Author  : Yash Tatode
 * Version : v6.1 — All fixes applied including gyration Y-axis + fabsf() for all axes
 *
 * ════════════════════════════════════════════════════════════════
 * HARDWARE
 * ════════════════════════════════════════════════════════════════
 *   ADXL375   ±200 g accel         I2C3  PC9=SDA  PA8=SCL
 *   ICM20948  6-axis IMU           I2C1  PB7=SDA  PB6=SCL
 *   DPS310    Baro / Temp          I2C1  (shared with ICM)
 *   SD card   SPI1                 PA4=CS PA5=SCK PA6=MISO PA7=MOSI
 *   UART      debug 115200 baud    PA2=TX
 *   PC13      Pyro — Drogue / Gyration-1
 *   PC14      Pyro — Gyration-2
 *   PC15      Pyro — Line cutter
 *   PA0       SD status LED
 *   PB9       Buzzer
 *
 * ════════════════════════════════════════════════════════════════
 * FIXES vs v6.0
 * ════════════════════════════════════════════════════════════════
 *  FIX-GY1  Check_Gyration() now includes |icm_ay| in the accel band check.
 *           Handles Z→Y tilt (gravity shifts from az into ay) which v6.0 missed.
 *
 *  FIX-GY2  All three axes use fabsf() so negative readings (e.g. -9.3 m/s²)
 *           are treated as positive — same magnitude, opposite tilt direction.
 *
 *  FIX-GY3  g_icm_ay global added and assigned in the main loop, so Y-axis
 *           accel is available to Check_Gyration() and is logged to SD.
 *
 *  FIX-SD1  SD flight CSV header and SD_WriteFlight() now include icm_ay column
 *           so post-flight analysis of Y-axis tilt is possible.
 *
 * All v6.0 fixes (FIX 1–12) are retained unchanged.
*/

#include "main.h"
#include "adxl.h"
#include "icm.h"
#include "dps310.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* ════════════════════════════════════════════════════════════════
   PERIPHERAL HANDLES
   ════════════════════════════════════════════════════════════════ */
I2C_HandleTypeDef  hi2c1;
I2C_HandleTypeDef  hi2c3;
UART_HandleTypeDef huart2;
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim2;

/* ════════════════════════════════════════════════════════════════
   USER CONFIG ZONE
   ════════════════════════════════════════════════════════════════ */
#define LOOP_DELAY_MS               10U     /* 100 Hz main loop */
#define SD_WRITE_EVERY              1U
#define SD_SYNC_EVERY               5U

/* Calibration sample counts */
#define ADXL_CAL_SAMPLES            200     /* 200 * 5ms  = 1s  @ 400 Hz sensor */
#define ICM_CAL_SAMPLES             200     /* 200 * 10ms = 2s  @ ~100 Hz sensor */
#define DPS_P0_SAMPLES              50      /* 50  * 200ms = 10s */
#define DPS_OFFSET_SAMPLES          50      /* 50  * 200ms = 10s */

#define GRAVITY_MS2                 9.80665f
#define TEMP_LAPSE_RATE             0.0065f /* K/m, ISA standard */
#define LAUNCH_SITE_ELEV_M          0.0f   /* AGL — set to actual MSL if known */

/* ── Deployment thresholds ── */
#define DROGUE_DROP_M               50.0f
#define BACKUP_DROP_M               150.0f
#define BACKUP_VEL_MIN_MS           15.0f
#define GYRATION_ALT_MIN_M          2800.0f
#define GYRATION_ACCEL_LO           8.0f
#define GYRATION_ACCEL_HI           10.0f
#define LINECUTTER_ALT_M            500.0f

#define PYRO_PULSE_MS               1000U
#define GYRATION_GAP_MS             1000U

/* ── Kalman filter noise params ──
 *  R_baro ≈ DPS310 std-dev² ≈ (0.3 m)² = 0.09 at 16x oversample.
 *  Ref: Welch & Bishop, "An Introduction to the Kalman Filter", UNC TR 95-041.
 */
#define KF_ADXL_Q                   0.5f
#define KF_ADXL_R                   0.25f
#define KF_ICM_ACCEL_Q              0.02f
#define KF_ICM_ACCEL_R              0.5f
#define KF_ICM_GYRO_Q               0.005f
#define KF_ICM_GYRO_R               0.1f

#define KF2D_Q_ALT                  0.01f
#define KF2D_Q_VEL                  0.10f
#define KF2D_R_BARO                 0.09f

/* ── Complementary IIR time constant ──
 *  alpha = dt / (dt + tau)  →  dt=0.01s, tau=0.1s  →  alpha ≈ 0.091
 *  ~1.6 Hz low-pass corner, removes jitter without lagging detection. (FIX 10)
 */
#define BARO_COMP_TAU               0.1f

/* ── Pin definitions ── */
#define PYRO_PORT                   GPIOC
#define PYRO_PC13_PIN               GPIO_PIN_13
#define PYRO_PC14_PIN               GPIO_PIN_14
#define PYRO_PC15_PIN               GPIO_PIN_15

#define SD_LED_PORT                 GPIOA
#define SD_LED_PIN                  GPIO_PIN_0

#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  GPIO_PIN_9

/* ── ICM20948 I2C addresses (AD0 pin selects 0x68 or 0x69) ── */
#define ICM_ADDR_LOW                0x68
#define ICM_ADDR_HIGH               0x69
#define ICM_WHOAMI_REG              0x00
#define ICM_WHOAMI_VAL              0xEA    /* ICM-20948 datasheet p.37 */

/* ════════════════════════════════════════════════════════════════
   SENSOR STRUCTS + SHARED BUFFERS
   ════════════════════════════════════════════════════════════════ */
ADXL375_t  adxl375;
ICM20948_t imu;
DPS310_t   dps;

char        uart_buf[300];
static char sd_row[420];   /* slightly larger to accommodate extra icm_ay column */
static char dep_row[200];

/* ════════════════════════════════════════════════════════════════
   SD CARD STATE
   ════════════════════════════════════════════════════════════════ */
static FATFS    sd_fs;
static bool     sd_ok         = false;
static char     sd_flt_fname[20];
static char     sd_dep_fname[20];
static uint32_t sd_row_idx    = 0;
static uint32_t sd_dep_idx    = 0;
static uint32_t sd_write_cnt  = 0;

/* ════════════════════════════════════════════════════════════════
   DEPLOYMENT STATE FLAGS
   ════════════════════════════════════════════════════════════════ */
static bool dep_primary_fired    = false;
static bool dep_backup_fired     = false;
static bool dep_gyration_fired   = false;
static bool dep_linecutter_fired = false;

/* ════════════════════════════════════════════════════════════════
   SHARED FLIGHT STATE GLOBALS
   ════════════════════════════════════════════════════════════════ */
static float g_kal_alt  = 0.0f;
static float g_kal_vel  = 0.0f;
static float g_peak_alt = -9999.0f;

/* ADXL outputs: physical m/s² AFTER offset removal (gravity preserved) */
static float g_adxl_ax  = 0.0f;
static float g_adxl_az  = 0.0f;

/* ICM outputs: physical m/s² AFTER driver calibration (gravity preserved).
 * FIX-GY3: g_icm_ay added so Check_Gyration() can test the Y axis.          */
static float g_icm_ax   = 0.0f;
static float g_icm_ay   = 0.0f;   /* FIX-GY3: was missing in v6.0 */
static float g_icm_az   = 0.0f;

/* ════════════════════════════════════════════════════════════════
   FILTER TYPES + INSTANCES
   ════════════════════════════════════════════════════════════════ */

/*
 * Kalman 1D — scalar random-walk process model.
 * Ref: Welch & Bishop, UNC TR 95-041.
 */
typedef struct { float q, r, x, p; } Kalman1D_t;

static Kalman1D_t kf_adxl_x, kf_adxl_y, kf_adxl_z;
static Kalman1D_t kf_icm_ax, kf_icm_ay, kf_icm_az;
static Kalman1D_t kf_icm_gx, kf_icm_gy, kf_icm_gz;

/*
 * Kalman 2D — state = [altitude, velocity], measurement = baro altitude.
 * State transition: alt(k) = alt(k-1) + vel(k-1)*dt
 *                   vel(k) = vel(k-1)  (constant-velocity prediction)
 * Ref: Schultz, "Application of the Kalman Filter to Rocket Apogee Detection"
 */
typedef struct {
    float alt, vel;
    float P00, P01, P10, P11;
    float Q_alt, Q_vel, R;
    bool  initialized;
} Kalman2D_t;

static Kalman2D_t kf_baro = {0};

/*
 * Median-3 — three-sample sliding window median.
 * Rejects single-sample spike glitches on ADXL375 at high-G events.
 */
typedef struct { float buf[3]; uint8_t idx; } Median3_t;

static Median3_t med_adxl_x = {0};
static Median3_t med_adxl_y = {0};
static Median3_t med_adxl_z = {0};
static Median3_t med_baro   = {0};

/* Complementary IIR state */
static float baro_comp_out  = 0.0f;
static bool  baro_comp_init = false;

/* DPS310 calibration state */
static float dps_P0         = 1013.25f;
static float dps_alt_offset = 0.0f;
static float dps_base_temp  = 15.0f;

/* ════════════════════════════════════════════════════════════════
   FUNCTION PROTOTYPES
   ════════════════════════════════════════════════════════════════ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* ════════════════════════════════════════════════════════════════
   UART HELPER
   ════════════════════════════════════════════════════════════════ */
#ifdef __GNUC__
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

static void UART_Print(const char *s) {
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* ════════════════════════════════════════════════════════════════
   PA0 — SD STATUS LED
   ════════════════════════════════════════════════════════════════ */
static void LED_On(void)     { HAL_GPIO_WritePin(SD_LED_PORT, SD_LED_PIN, GPIO_PIN_SET);   }
static void LED_Off(void)    { HAL_GPIO_WritePin(SD_LED_PORT, SD_LED_PIN, GPIO_PIN_RESET); }
static void LED_Toggle(void) { HAL_GPIO_TogglePin(SD_LED_PORT, SD_LED_PIN);               }

static void LED_Blink(int n, uint32_t on_ms, uint32_t off_ms) {
    for (int i = 0; i < n; i++) {
        LED_On();  HAL_Delay(on_ms);
        LED_Off(); if (i < n - 1) HAL_Delay(off_ms);
    }
}

/* ════════════════════════════════════════════════════════════════
   PB9 — BUZZER
   ════════════════════════════════════════════════════════════════ */
static void Buzzer_On(void)  { HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);   }
static void Buzzer_Off(void) { HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET); }

static void Buzzer_Beep(int n, uint32_t on_ms, uint32_t off_ms) {
    for (int i = 0; i < n; i++) {
        Buzzer_On();  HAL_Delay(on_ms);
        Buzzer_Off(); if (i < n - 1) HAL_Delay(off_ms);
    }
}

static void Buzzer_LongBeep(uint32_t ms) {
    Buzzer_On();
    HAL_Delay(ms);
    Buzzer_Off();
}

/* ════════════════════════════════════════════════════════════════
   SENSOR FAILURE HALT
   3 buzzer beeps + fast LED blink. Never returns.
   ════════════════════════════════════════════════════════════════ */
static void Sensor_Fail_Halt(bool dps_fail, bool icm_fail, bool adxl_fail) {
    LED_Off();
    UART_Print("\r\n!!! CRITICAL SENSOR FAILURE — HALTED !!!\r\n");
    if (dps_fail)  UART_Print("  DPS310   FAILED\r\n");
    if (icm_fail)  UART_Print("  ICM20948 FAILED\r\n");
    if (adxl_fail) UART_Print("  ADXL375  FAILED\r\n");
    Buzzer_Beep(3, 300, 200);
    while (1) { LED_Toggle(); HAL_Delay(200); }
}

/* ════════════════════════════════════════════════════════════════
   FILTER MATH
   ════════════════════════════════════════════════════════════════ */

static void Kalman1D_Init(Kalman1D_t *f, float q, float r) {
    f->q = q;  f->r = r;  f->x = 0.0f;  f->p = 1.0f;
}

static float Kalman1D_Update(Kalman1D_t *f, float z) {
    f->p += f->q;
    float k = f->p / (f->p + f->r);
    f->x  += k * (z - f->x);
    f->p  *= (1.0f - k);
    return f->x;
}

static void Kalman2D_Init(Kalman2D_t *k, float init_alt) {
    k->alt = init_alt; k->vel = 0.0f;
    k->P00 = 1.0f; k->P01 = 0.0f;
    k->P10 = 0.0f; k->P11 = 1.0f;
    k->Q_alt = KF2D_Q_ALT; k->Q_vel = KF2D_Q_VEL; k->R = KF2D_R_BARO;
    k->initialized = true;
}

/*
 * 2D Kalman predict+update step.
 * State: x = [alt, vel]
 * Process model: alt(k) = alt(k-1) + vel(k-1)*dt
 *                vel(k) = vel(k-1)  (constant-velocity prediction)
 * Measurement: z = alt_baro (scalar), H = [1, 0]
 * Process noise Q = diag(Q_alt, Q_vel), Measurement noise R (scalar)
 */
static void Kalman2D_Update(Kalman2D_t *k, float dt, float meas_alt) {
    if (!k->initialized) { Kalman2D_Init(k, meas_alt); return; }

    /* --- Predict --- */
    float ap   = k->alt + k->vel * dt;
    float vp   = k->vel;
    float P00p = k->P00 + dt * (k->P10 + k->P01) + dt * dt * k->P11 + k->Q_alt;
    float P01p = k->P01 + dt * k->P11;
    float P10p = k->P10 + dt * k->P11;
    float P11p = k->P11 + k->Q_vel;

    /* --- Update --- */
    float S  = P00p + k->R;
    if (fabsf(S) < 1e-9f) S = 1e-9f;   /* numerical guard */
    float K0 = P00p / S;
    float K1 = P10p / S;
    float y  = meas_alt - ap;

    k->alt = ap + K0 * y;
    k->vel = vp + K1 * y;
    k->P00 = (1.0f - K0) * P00p;
    k->P01 = (1.0f - K0) * P01p;
    k->P10 = P10p - K1 * P00p;
    k->P11 = P11p - K1 * P01p;
}

/* Sliding window median-3 filter — rejects single-sample spike glitches */
static float Median3_Update(Median3_t *m, float val) {
    m->buf[m->idx] = val;
    m->idx = (m->idx + 1) % 3;
    float s[3] = {m->buf[0], m->buf[1], m->buf[2]}, t;
    if (s[0] > s[1]) { t = s[0]; s[0] = s[1]; s[1] = t; }
    if (s[1] > s[2]) { t = s[1]; s[1] = s[2]; s[2] = t; }
    if (s[0] > s[1]) { t = s[0]; s[0] = s[1]; s[1] = t; }
    return s[1];
}

/*
 * Complementary IIR low-pass filter.
 * alpha = dt / (dt + tau)  — standard RC low-pass discretisation. (FIX 10)
 * Ref: https://en.wikipedia.org/wiki/Low-pass_filter#RC_filter
 */
static float BaroComp_Update(float raw) {
    const float dt    = LOOP_DELAY_MS / 1000.0f;
    const float alpha = dt / (dt + BARO_COMP_TAU);
    if (!baro_comp_init) {
        baro_comp_out  = raw;
        baro_comp_init = true;
        return raw;
    }
    baro_comp_out = alpha * raw + (1.0f - alpha) * baro_comp_out;
    return baro_comp_out;
}

/*
 * Nakka temperature correction.
 * Corrects barometric altitude for non-standard launch-site temperature.
 * Ref: https://www.nakka-rocketry.net/apogee.html
 */
static float Nakka_Correct(float h, float base_c) {
    float Hb = LAUNCH_SITE_ELEV_M;
    float L  = TEMP_LAPSE_RATE;
    float To = base_c + L * Hb + 273.15f;
    float Tb = base_c + 273.15f;
    float d  = To + L * h;
    if (fabsf(d) < 0.001f) return h;
    return h - (h - Hb) * ((To - Tb) / d);
}

/* ════════════════════════════════════════════════════════════════
   SD CARD
   ════════════════════════════════════════════════════════════════ */
static uint16_t SD_NextNumber(void) {
    DIR dir; FILINFO fi; uint16_t mx = 0;
    if (f_opendir(&dir, "/") != FR_OK) return 1;
    while (f_readdir(&dir, &fi) == FR_OK && fi.fname[0]) {
        if (strncmp(fi.fname, "FLT_", 4) == 0) {
            uint16_t n = 0;
            if (sscanf(fi.fname + 4, "%hu", &n) == 1 && n > mx) mx = n;
        }
    }
    f_closedir(&dir);
    return mx + 1;
}

static void SD_Init(void) {
    UART_Print("Initializing SD...\r\n");
    LED_Off();
    if (f_mount(&sd_fs, "", 1) != FR_OK) {
        UART_Print("ERROR: SD mount failed!\r\n");
        sd_ok = false; return;
    }
    uint16_t num = SD_NextNumber();
    sprintf(sd_flt_fname, "FLT_%03d.csv", num);
    sprintf(sd_dep_fname, "DEP_%03d.csv", num);
    sprintf(uart_buf, "SD files: %s | %s\r\n", sd_flt_fname, sd_dep_fname);
    UART_Print(uart_buf);

    FIL f; UINT bw;
    if (f_open(&f, sd_flt_fname, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        UART_Print("ERROR: FLT create failed!\r\n"); sd_ok = false; return;
    }
    /* FIX-SD1: icm_ay added to CSV header */
    const char *fh =
        "idx,"
        "adxl_ax,adxl_ay,adxl_az,adxl_g,"
        "icm_ax,icm_ay,icm_az,icm_gx,icm_gy,icm_gz,icm_g,"
        "temp_c,press_hpa,alt_m,peak_m,vel_ms,"
        "pc13,pc14,pc15\r\n";
    f_write(&f, fh, strlen(fh), &bw); f_sync(&f); f_close(&f);

    if (f_open(&f, sd_dep_fname, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        UART_Print("ERROR: DEP create failed!\r\n"); sd_ok = false; return;
    }
    const char *dh =
        "dep_idx,flt_idx,time_ms,event,"
        "alt_m,vel_ms,peak_m,adxl_ax,adxl_az,icm_ay,pc13,pc14,pc15\r\n";
    f_write(&f, dh, strlen(dh), &bw); f_sync(&f); f_close(&f);

    sd_ok = true;
    sprintf(uart_buf, "SD OK — flight #%d\r\n\r\n", num);
    UART_Print(uart_buf);
}

/* FIX-SD1: icm_ay parameter added */
static void SD_WriteFlight(
        float adxl_ax, float adxl_ay, float adxl_az, float adxl_g,
        float icm_ax,  float icm_ay,  float icm_az,
        float icm_gx,  float icm_gy,  float icm_gz,  float icm_g,
        float temp,    float pressure, float alt,    float peak, float vel,
        uint8_t pc13,  uint8_t pc14,  uint8_t pc15) {
    if (!sd_ok) { LED_Off(); return; }
    FIL f; UINT bw;
    if (f_open(&f, sd_flt_fname, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        sd_ok = false; LED_Off(); return;
    }
    sprintf(sd_row,
        "%05lu,"
        "%+.3f,%+.3f,%+.3f,%.4f,"
        "%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%.4f,"
        "%.2f,%.4f,%.2f,%.2f,%.3f,"
        "%d,%d,%d\r\n",
        sd_row_idx,
        adxl_ax, adxl_ay, adxl_az, adxl_g,
        icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz, icm_g,
        temp, pressure, alt, peak, vel,
        pc13, pc14, pc15);
    FRESULT wr = f_write(&f, sd_row, strlen(sd_row), &bw);
    sd_write_cnt++;
    if (sd_write_cnt % SD_SYNC_EVERY == 0) f_sync(&f);
    f_close(&f);
    if (wr != FR_OK || bw == 0) { sd_ok = false; LED_Off(); return; }
    LED_Toggle();
}

static void SD_WriteDeployment(const char *name,
        float alt, float vel, float peak,
        float ax,  float az,  float ay,
        uint8_t pc13, uint8_t pc14, uint8_t pc15) {
    if (!sd_ok) { LED_Off(); return; }
    FIL f; UINT bw;
    if (f_open(&f, sd_dep_fname, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        sd_ok = false; LED_Off(); return;
    }
    /* FIX-GY3: icm_ay included in deployment log */
    sprintf(dep_row,
        "%05lu,%05lu,%lu,%s,"
        "%.2f,%.3f,%.2f,"
        "%+.3f,%+.3f,%+.3f,%d,%d,%d\r\n",
        sd_dep_idx, sd_row_idx, HAL_GetTick(), name,
        alt, vel, peak, ax, az, ay, pc13, pc14, pc15);
    f_write(&f, dep_row, strlen(dep_row), &bw);
    f_sync(&f); f_close(&f);
    sd_dep_idx++;
}

/* ════════════════════════════════════════════════════════════════
   ICM20948 ADDRESS DETECTION  (FIX 11)
   AD0 LOW→0x68, HIGH→0x69. WHO_AM_I must return 0xEA.
   ════════════════════════════════════════════════════════════════ */
static void ICM_Detect(void) {
    UART_Print("=== I2C1 Scan ===\r\n");
    uint8_t found = 0;
    for (uint8_t a = 1; a < 128; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 3, 20) == HAL_OK) {
            sprintf(uart_buf, "  0x%02X\r\n", a); UART_Print(uart_buf); found++;
        }
    }
    if (!found) {
        UART_Print("  NONE! Check PB6/PB7 pullups.\r\n");
        Sensor_Fail_Halt(false, true, false);
    }
    uint8_t cand[2] = {ICM_ADDR_LOW, ICM_ADDR_HIGH};
    for (int i = 0; i < 2; i++) {
        uint16_t a16 = (uint16_t)(cand[i] << 1);
        if (HAL_I2C_IsDeviceReady(&hi2c1, a16, 3, 20) == HAL_OK) {
            uint8_t reg = ICM_WHOAMI_REG, who = 0;
            HAL_I2C_Master_Transmit(&hi2c1, a16, &reg, 1, 20);
            HAL_I2C_Master_Receive (&hi2c1, a16, &who, 1, 20);
            sprintf(uart_buf, "  0x%02X WHO=0x%02X ", cand[i], who);
            UART_Print(uart_buf);
            if (who == ICM_WHOAMI_VAL) {
                UART_Print("OK ICM-20948\r\n");
                imu.address = (uint8_t)(cand[i] << 1);
                return;
            }
            UART_Print("skip\r\n");
        }
    }
    UART_Print("ICM20948 not found on 0x68 or 0x69!\r\n");
    Sensor_Fail_Halt(false, true, false);
}

/* ════════════════════════════════════════════════════════════════
   SENSOR CALIBRATION
   ════════════════════════════════════════════════════════════════ */

/*
 * ADXL375 Calibration  (FIX 1 + FIX 2)
 *
 * Rocket MUST be upright (nose up) and stationary.
 * ADXL375 Z+ points upward on nose-up PCB → reads +9.81 m/s².
 *
 * offset_x = avg_x            (bias, expected ≈ 0)
 * offset_y = avg_y            (bias, expected ≈ 0)
 * offset_z = avg_z - GRAVITY  (bias only; gravity is left in)
 *
 * Software offsets used — NOT hardware registers 0x1E/1F/20
 * which only have 15.6 mg/LSB and ±2g range (ADXL375 datasheet Table 2).
 */
static void Cal_ADXL375(void) {
    UART_Print("=== ADXL375 CAL ===\r\n");
    UART_Print("  >> Ensure rocket is UPRIGHT (nose up) and STATIONARY <<\r\n");
    HAL_Delay(3000);

    float sx = 0, sy = 0, sz = 0;
    int n = 0;
    adxl375_event_t ev;

    for (int i = 0; i < ADXL_CAL_SAMPLES; i++) {
        if (ADXL375_GetEvent(&adxl375, &ev)) {
            sx += ev.x; sy += ev.y; sz += ev.z;
            n++;
            if (i % 40 == 0) {
                sprintf(uart_buf, "  [%3d/%d] x=%+.3f y=%+.3f z=%+.3f m/s2\r\n",
                        i, ADXL_CAL_SAMPLES, ev.x, ev.y, ev.z);
                UART_Print(uart_buf);
            }
        }
        HAL_Delay(5);
    }

    if (n < (ADXL_CAL_SAMPLES / 2)) {
        UART_Print("WARNING: Low ADXL sample count — using zero offsets.\r\n");
        ADXL375_SetOffsets(&adxl375, 0.0f, 0.0f, 0.0f);
        return;
    }

    float avg_x = sx / n;
    float avg_y = sy / n;
    float avg_z = sz / n;

    /* FIX 2: offset_z = avg_z - GRAVITY so (raw_z - offset_z) = +9.81 at rest */
    ADXL375_SetOffsets(&adxl375, avg_x, avg_y, avg_z - GRAVITY_MS2);
    ADXL375_ResetVelocity(&adxl375);

    sprintf(uart_buf,
            "ADXL375 CAL OK — offX=%+.4f offY=%+.4f offZ=%+.4f m/s2  (n=%d)\r\n"
            "  At rest: ax~0 ay~0 az~+%.4f m/s2\r\n\r\n",
            adxl375.offset_x, adxl375.offset_y, adxl375.offset_z,
            n, GRAVITY_MS2);
    UART_Print(uart_buf);
}

/*
 * DPS310 P0 Calibration  (FIX 5, FIX 6, FIX 7)
 *
 * 50 samples × 200 ms = 10 s window → robust P0.
 * DPS310 at 16x oversample: meas time 27.6 ms, rate 62.5 ms.
 * 200 ms poll delay guarantees fresh data every read. (FIX 6)
 * Silent fallback replaced with 3-beep warning + UART message. (FIX 7)
 */
static void Cal_DPS_P0(void) {
    UART_Print("=== DPS310 P0 CAL (10s) ===\r\n");
    float s = 0;
    int n = 0;
    for (int i = 0; i < DPS_P0_SAMPLES; i++) {
        HAL_Delay(200);
        float t, p;
        DPS310_Status st = DPS310_ReadTemperaturePressure(&dps, &t, &p);
        if ((st == DPS310_OK || st == DPS310_BUSY) && p > 800.0f && p < 1200.0f) {
            s += p;
            n++;
            if (i % 10 == 0) {
                sprintf(uart_buf, "  [%2d/%d] P=%.3f hPa\r\n", i, DPS_P0_SAMPLES, p);
                UART_Print(uart_buf);
            }
        }
    }

    if (n >= 25) {
        dps_P0 = s / n;
        sprintf(uart_buf, "  P0 = %.4f hPa  (%d samples)\r\n\r\n", dps_P0, n);
        UART_Print(uart_buf);
    } else {
        UART_Print("  *** WARNING: P0 cal failed! Only ");
        sprintf(uart_buf, "%d", n); UART_Print(uart_buf);
        UART_Print(" samples. Using 1013.25 hPa default.\r\n");
        UART_Print("  *** CHECK DPS310 wiring before flight!\r\n\r\n");
        Buzzer_Beep(3, 200, 150);
        dps_P0 = 1013.25f;
    }
}

/*
 * DPS310 Altitude Offset Calibration  (FIX 8)
 *
 * Nakka_Correct() applied to each sample BEFORE averaging, so the offset
 * and the main-loop correction are in the same domain and cancel exactly.
 */
static void Cal_DPS_Offset(void) {
    UART_Print("=== DPS310 OFFSET CAL (10s) ===\r\n");
    UART_Print("  >> Rocket must be at launch pad, STATIONARY <<\r\n");
    HAL_Delay(1000);

    float sa = 0, st = 0;
    int n = 0;
    float local_temp = 15.0f;
    bool temp_got = false;

    for (int i = 0; i < DPS_OFFSET_SAMPLES; i++) {
        HAL_Delay(200);
        float t, p;
        DPS310_Status status = DPS310_ReadTemperaturePressure(&dps, &t, &p);
        if (status == DPS310_OK || status == DPS310_BUSY) {
            if (!temp_got && status == DPS310_OK) {
                local_temp = t;
                temp_got = true;
            }
            /* FIX 8: Nakka applied BEFORE averaging */
            float raw_alt   = DPS310_CalculateAltitude(p, dps_P0);
            float nakka_alt = Nakka_Correct(raw_alt, local_temp);
            sa += nakka_alt;
            if (status == DPS310_OK) st += t;
            n++;
            if (i % 10 == 0) {
                sprintf(uart_buf, "  [%2d/%d] raw=%.2fm  nakka=%.2fm  T=%.1fC\r\n",
                        i, DPS_OFFSET_SAMPLES, raw_alt, nakka_alt, local_temp);
                UART_Print(uart_buf);
            }
        }
    }

    if (n >= 25) {
        dps_alt_offset = sa / n;
        dps_base_temp  = temp_got ? (st / n) : 15.0f;
    } else {
        UART_Print("  *** WARNING: Offset cal low samples — using 0m/15C\r\n");
        dps_alt_offset = 0.0f;
        dps_base_temp  = 15.0f;
        Buzzer_Beep(3, 200, 150);
    }

    sprintf(uart_buf,
            "  Offset=%.3f m (Nakka-corrected)  BaseTemp=%.2f C  (n=%d)\r\n\r\n",
            dps_alt_offset, dps_base_temp, n);
    UART_Print(uart_buf);
}

/* ════════════════════════════════════════════════════════════════
   PYRO HELPER
   ════════════════════════════════════════════════════════════════ */
static void Pyro_Fire(uint16_t pin) {
    HAL_GPIO_WritePin(PYRO_PORT, pin, GPIO_PIN_SET);
    HAL_Delay(PYRO_PULSE_MS);
    HAL_GPIO_WritePin(PYRO_PORT, pin, GPIO_PIN_RESET);
}

/* ════════════════════════════════════════════════════════════════
   DEPLOYMENT CHECKS
   ════════════════════════════════════════════════════════════════ */
static void Check_PrimaryDrogue(void) {
    if (dep_primary_fired) return;
    if ((g_peak_alt - g_kal_alt) < DROGUE_DROP_M) return;
    dep_primary_fired = true;
    UART_Print("[DEPLOY] PRIMARY DROGUE — PC13\r\n");
    SD_WriteDeployment("PRIMARY_DROGUE",
        g_kal_alt, g_kal_vel, g_peak_alt,
        g_adxl_ax, g_adxl_az, g_icm_ay, 1, 0, 0);
    Pyro_Fire(PYRO_PC13_PIN);
    Buzzer_LongBeep(1000);
}

static void Check_BackupDrogue(void) {
    if (dep_backup_fired) return;
    if ((g_peak_alt - g_kal_alt) < BACKUP_DROP_M) return;
    if (fabsf(g_kal_vel) <= BACKUP_VEL_MIN_MS) return;
    dep_backup_fired = true;
    UART_Print("[DEPLOY] BACKUP DROGUE — PC14\r\n");
    SD_WriteDeployment("BACKUP_DROGUE",
        g_kal_alt, g_kal_vel, g_peak_alt,
        g_adxl_ax, g_adxl_az, g_icm_ay, 0, 1, 0);
    Pyro_Fire(PYRO_PC14_PIN);
    Buzzer_LongBeep(2000);
}

/*
 * Check_Gyration  (FIX-GY1, FIX-GY2, FIX-GY3)
 *
 * Detects when the rocket has tilted and gravity has shifted from the Z axis
 * into either the X or Y axis (or any combination), indicating gyration.
 *
 * Condition: one or more axes has a gravity-magnitude component in the band
 * [GYRATION_ACCEL_LO, GYRATION_ACCEL_HI] m/s², which for a 1g environment
 * is approximately [8, 10] m/s².  fabsf() is used on all three axes so that
 * both positive and negative tilts are detected symmetrically.
 *
 * v6.0 only checked |ax| and |az|, missing Z→Y tilts (FIX-GY1).
 * v6.0 already used fabsf() on ax/az, now extended to ay (FIX-GY2).
 * g_icm_ay is now assigned in the main loop and available here (FIX-GY3).
 */
static void Check_Gyration(void) {
    if (dep_gyration_fired) return;

    /* FIX-GY1/GY2: all three axes, all with fabsf() */
    float ax     = fabsf(g_icm_ax);
    float ay     = fabsf(g_icm_ay);   /* FIX-GY1: Y axis added */
//

    bool in_band = ((ax >= GYRATION_ACCEL_LO && ax <= GYRATION_ACCEL_HI) ||
                    (ay >= GYRATION_ACCEL_LO && ay <= GYRATION_ACCEL_HI)); //||   /* FIX-GY1 */
//                    (az >= GYRATION_ACCEL_LO && az <= GYRATION_ACCEL_HI));

    if (g_kal_alt < GYRATION_ALT_MIN_M || !in_band) return;

    dep_gyration_fired = true;
    dep_primary_fired  = true;

    UART_Print("[DEPLOY] GYRATION — Charge 1 (PC13)\r\n");
    SD_WriteDeployment("GYRATION_CH1",
        g_kal_alt, g_kal_vel, g_peak_alt,
        g_adxl_ax, g_adxl_az, g_icm_ay, 1, 0, 0);
    Pyro_Fire(PYRO_PC13_PIN);

    HAL_Delay(GYRATION_GAP_MS);

    UART_Print("[DEPLOY] GYRATION — Charge 2 (PC14)\r\n");
    SD_WriteDeployment("GYRATION_CH2",
        g_kal_alt, g_kal_vel, g_peak_alt,
        g_adxl_ax, g_adxl_az, g_icm_ay, 0, 1, 0);
    Pyro_Fire(PYRO_PC14_PIN);
    Buzzer_LongBeep(3000);
}

static void Check_LineCutter(void) {
    if (dep_linecutter_fired) return;
    if (!dep_primary_fired) return;
    if (g_kal_alt > LINECUTTER_ALT_M) return;
    dep_linecutter_fired = true;
    UART_Print("[DEPLOY] LINE CUTTER — PC15\r\n");
    SD_WriteDeployment("LINE_CUTTER",
        g_kal_alt, g_kal_vel, g_peak_alt,
        g_adxl_ax, g_adxl_az, g_icm_ay, 0, 0, 1);
    Pyro_Fire(PYRO_PC15_PIN);
    Buzzer_LongBeep(4500);
}

/* ════════════════════════════════════════════════════════════════
   MAIN
   ════════════════════════════════════════════════════════════════ */
int main(void) {

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_FATFS_Init();

    /* All outputs LOW/safe at boot */
    LED_Off();
    Buzzer_Off();
    HAL_GPIO_WritePin(PYRO_PORT,
        PYRO_PC13_PIN | PYRO_PC14_PIN | PYRO_PC15_PIN,
        GPIO_PIN_RESET);
    /* ── Power-on beep — single 200 ms tone confirms MCU is alive ── */
    Buzzer_Beep(1, 200, 0);

    UART_Print("\r\n");
    UART_Print("╔════════════════════════════════════╗\r\n");
    UART_Print("║   ROCKET FLIGHT COMPUTER v6.1      ║\r\n");
    UART_Print("╚════════════════════════════════════╝\r\n\r\n");

    SD_Init();

    /* ── ADXL375 init ── */
    UART_Print("--- ADXL375 (I2C3, addr=0x53) ---\r\n");
    if (ADXL375_Init(&adxl375, &hi2c3, ADXL375_ADDRESS_ALT_LOW) != HAL_OK ||
        !ADXL375_Begin(&adxl375)) {
        UART_Print("ADXL375 FAILED!\r\n");
        Sensor_Fail_Halt(false, false, true);
    }
    ADXL375_SetDataRate(&adxl375, ADXL375_DATARATE_400_HZ);
    UART_Print("ADXL375 OK — 400 Hz\r\n");
    LED_Blink(1, 200, 0);
    HAL_Delay(300);
    UART_Print("\r\n");

    /* ── ICM20948 init (FIX 11: detect address before init) ── */
    UART_Print("--- ICM20948 (I2C1, scan 0x68/0x69) ---\r\n");
    ICM_Detect();
    if (ICM20948_Init(&imu, &hi2c1) != HAL_OK) {
        UART_Print("ICM20948 FAILED!\r\n");
        Sensor_Fail_Halt(false, true, false);
    }
    UART_Print("ICM20948 OK\r\n");
    LED_Blink(1, 200, 0);
    HAL_Delay(300);
    UART_Print("\r\n");

    /* ── DPS310 init ── */
    UART_Print("--- DPS310 (I2C1, addr=0x77) ---\r\n");
    if (DPS310_Init(&dps, &hi2c1, 0x77) !=  DPS310_OK) {
        UART_Print("DPS310 FAILED!\r\n");
        Sensor_Fail_Halt(true, false, false);
    }
    UART_Print("DPS310 OK — 16x oversample @ 16Hz\r\n");
    LED_Blink(1, 200, 0);
    HAL_Delay(300);
    UART_Print("\r\n");

    /* ── ICM20948 calibration (FIX 4: 2s settle before gyro cal) ──
     *
     * ICM-20948 datasheet (p.11): settle ≥20 ms after power-on.
     * We use 2 s to let physical vibration die out.
     * Gyro calibration before accel to avoid cross-coupling.
     */
    UART_Print("--- ICM20948 CAL ---\r\n");
    UART_Print("  >> Keep rocket UPRIGHT and STILL for 4 seconds <<\r\n");
    HAL_Delay(2000);
    ICM20948_CalibrateGyro(&imu, ICM_CAL_SAMPLES);
    UART_Print("ICM20948 Gyro cal OK\r\n");
    HAL_Delay(500);
    ICM20948_CalibrateAccel(&imu, ICM_CAL_SAMPLES);
    UART_Print("ICM20948 Accel cal OK\r\n");
    sprintf(uart_buf,
        "  Gyro offsets:  X=%+.4f Y=%+.4f Z=%+.4f deg/s\r\n"
        "  Accel offsets: X=%+.4f Y=%+.4f Z=%+.4f m/s2\r\n"
        "  (accel_offset_z ~0 if az=+9.81 at rest)\r\n\r\n",
        imu.calib_data.gyro_offset_x,
        imu.calib_data.gyro_offset_y,
        imu.calib_data.gyro_offset_z,
        imu.calib_data.accel_offset_x,
        imu.calib_data.accel_offset_y,
        imu.calib_data.accel_offset_z);
    UART_Print(uart_buf);

    /* ── DPS310 P0 calibration (FIX 5,6,7) ── */
    Cal_DPS_P0();

    /* ── DPS310 altitude offset calibration (FIX 8) ── */
    Cal_DPS_Offset();

    /* ── ADXL375 calibration (FIX 1,2) ── */
    Cal_ADXL375();

    /* ── Init all Kalman 1D filters ── */
    Kalman1D_Init(&kf_adxl_x, KF_ADXL_Q,      KF_ADXL_R);
    Kalman1D_Init(&kf_adxl_y, KF_ADXL_Q,      KF_ADXL_R);
    Kalman1D_Init(&kf_adxl_z, KF_ADXL_Q,      KF_ADXL_R);
    Kalman1D_Init(&kf_icm_ax, KF_ICM_ACCEL_Q, KF_ICM_ACCEL_R);
    Kalman1D_Init(&kf_icm_ay, KF_ICM_ACCEL_Q, KF_ICM_ACCEL_R);
    Kalman1D_Init(&kf_icm_az, KF_ICM_ACCEL_Q, KF_ICM_ACCEL_R);
    Kalman1D_Init(&kf_icm_gx, KF_ICM_GYRO_Q,  KF_ICM_GYRO_R);
    Kalman1D_Init(&kf_icm_gy, KF_ICM_GYRO_Q,  KF_ICM_GYRO_R);
    Kalman1D_Init(&kf_icm_gz, KF_ICM_GYRO_Q,  KF_ICM_GYRO_R);

    /* ── Kalman 2D seed (FIX 9: 5-reading average with Nakka applied) ── */
    UART_Print("--- Seeding altitude Kalman (FIX 9) ---\r\n");
    {
        float seed_sum = 0.0f;
        int   seed_n   = 0;
        float t, p;
        for (int i = 0; i < 5; i++) {
            HAL_Delay(200);
            DPS310_Status st = DPS310_ReadTemperaturePressure(&dps, &t, &p);
            if (st == DPS310_OK || st == DPS310_BUSY) {
                float raw  = DPS310_CalculateAltitude(p, dps_P0) - dps_alt_offset;
                float corr = Nakka_Correct(raw, dps_base_temp);
                seed_sum += corr;
                seed_n++;
            }
        }
        float seed = (seed_n > 0) ? (seed_sum / seed_n) : 0.0f;
        Kalman2D_Init(&kf_baro, seed);
        g_peak_alt = seed;
        sprintf(uart_buf, "  Seed = %.3f m  (avg of %d readings)\r\n\r\n",
                seed, seed_n);
        UART_Print(uart_buf);
    }

    UART_Print("All calibration done — 1.5s confirmation beep.\r\n\r\n");
    Buzzer_LongBeep(1500);

    UART_Print("══════════════════════════════════════════════\r\n");
    UART_Print("  ALL SYSTEMS GO — 100 Hz LOGGING STARTED    \r\n");
    UART_Print("══════════════════════════════════════════════\r\n\r\n");

    uint32_t last_tick = HAL_GetTick();
    uint32_t loop_cnt  = 0;

    /* ══════════════════════════════════════════════
       MAIN FLIGHT LOOP — 100 Hz
       ══════════════════════════════════════════════ */
    while (1) {

        uint32_t now = HAL_GetTick();
        float dt = (float)(now - last_tick) / 1000.0f;
        if (dt <= 0.0f || dt > 0.5f) dt = LOOP_DELAY_MS / 1000.0f;
        last_tick = now;

        /* ── ADXL375 read + filter ──
         *
         * ADXL375_GetEvent() returns raw physical m/s² (no offset applied).
         * Single explicit offset subtraction here (FIX 1).
         * After: ax,ay ≈ 0 at rest; az ≈ +9.81 at rest (FIX 2).
         * Median-3 → Kalman 1D filter chain on each axis.
         */
        float adxl_ax = 0, adxl_ay = 0, adxl_az = 0, adxl_g = 0;
        adxl375_event_t ev;
        if (ADXL375_GetEvent(&adxl375, &ev)) {
            float rx = ev.x - adxl375.offset_x;
            float ry = ev.y - adxl375.offset_y;
            float rz = ev.z - adxl375.offset_z;
            adxl_ax = Kalman1D_Update(&kf_adxl_x, Median3_Update(&med_adxl_x, rx));
            adxl_ay = Kalman1D_Update(&kf_adxl_y, Median3_Update(&med_adxl_y, ry));
            adxl_az = Kalman1D_Update(&kf_adxl_z, Median3_Update(&med_adxl_z, rz));
            adxl_g  = sqrtf(adxl_ax*adxl_ax + adxl_ay*adxl_ay + adxl_az*adxl_az)
                      / GRAVITY_MS2;
            g_adxl_ax = adxl_ax;
            g_adxl_az = adxl_az;
        }

        /* ── ICM20948 read + filter ──
         *
         * ICM20948_ReadScaledData() internally applies calibration offsets.
         * DO NOT subtract offsets again — that would double-subtract (FIX 3).
         * After: icm_ax/ay ≈ 0 at rest; icm_az ≈ +9.81 at rest.
         *
         * FIX-GY3: g_icm_ay assigned here so Check_Gyration() can use it.
         */
        ICM20948_ReadScaledData(&imu);
        float icm_ax = Kalman1D_Update(&kf_icm_ax, imu.scaled_data.accel_x);
        float icm_ay = Kalman1D_Update(&kf_icm_ay, imu.scaled_data.accel_y);
        float icm_az = Kalman1D_Update(&kf_icm_az, imu.scaled_data.accel_z);
        float icm_gx = Kalman1D_Update(&kf_icm_gx, imu.scaled_data.gyro_x);
        float icm_gy = Kalman1D_Update(&kf_icm_gy, imu.scaled_data.gyro_y);
        float icm_gz = Kalman1D_Update(&kf_icm_gz, imu.scaled_data.gyro_z);
        float icm_g  = sqrtf(icm_ax*icm_ax + icm_ay*icm_ay + icm_az*icm_az)
                       / GRAVITY_MS2;
        g_icm_ax = icm_ax;
        g_icm_ay = icm_ay;   /* FIX-GY3: assign Y axis global */
        g_icm_az = icm_az;

        /* ── DPS310 read + filter chain ──
         *
         * Filter chain (FIX 8, FIX 10, FIX 12):
         *  raw altitude
         *    → subtract Nakka-corrected pad offset  (FIX 8)
         *    → Nakka temperature correction
         *    → Median-3 spike filter
         *    → Complementary IIR low-pass  (FIX 10)
         *    → Kalman 2D [altitude, velocity]
         *
         * DPS310_BUSY treated as valid (returns last reading).  (FIX 12)
         * On DPS310_ERROR: keep running with last Kalman estimate — do not halt.
         */
        float temperature = 0.0f, pressure = 0.0f;
        DPS310_Status baro_status =
            DPS310_ReadTemperaturePressure(&dps, &temperature, &pressure);

        if (baro_status == DPS310_OK || baro_status == DPS310_BUSY) {
            float raw  = DPS310_CalculateAltitude(pressure, dps_P0) - dps_alt_offset;
            float corr = Nakka_Correct(raw, dps_base_temp);
            float med  = Median3_Update(&med_baro, corr);
            float comp = BaroComp_Update(med);
            Kalman2D_Update(&kf_baro, dt, comp);
        }

        g_kal_alt = kf_baro.alt;
        g_kal_vel = kf_baro.vel;
        if (g_kal_alt > g_peak_alt) g_peak_alt = g_kal_alt;

        /* ── Deployment checks ── */
        Check_Gyration();        /* must run before primary — sets dep_primary_fired */
        Check_PrimaryDrogue();
        Check_BackupDrogue();
        Check_LineCutter();

        uint8_t pc13 = (uint8_t)(dep_primary_fired || dep_backup_fired || dep_gyration_fired);
        uint8_t pc14 = (uint8_t)(dep_gyration_fired || dep_backup_fired);
        uint8_t pc15 = (uint8_t)(dep_linecutter_fired);

        /* ── UART telemetry ── */
        sprintf(uart_buf,
            "[%05lu]\r\n"
            "  ADXL : X=%+8.3f Y=%+8.3f Z=%+8.3f (m/s2) G=%.4fg\r\n"
            "  ICMa : X=%+8.3f Y=%+8.3f Z=%+8.3f (m/s2) G=%.4fg\r\n"
            "  ICMg : X=%+8.3f Y=%+8.3f Z=%+8.3f (deg/s)\r\n"
            "  DPS  : T=%.2fC P=%.3fhPa Alt=%7.2fm Peak=%7.2fm Vel=%+7.3fm/s\r\n"
            "  PYRO : PC13=%d PC14=%d PC15=%d\r\n\r\n",
            sd_row_idx,
            adxl_ax, adxl_ay, adxl_az, adxl_g,
            icm_ax, icm_ay, icm_az, icm_g,
            icm_gx, icm_gy, icm_gz,
            temperature, pressure, g_kal_alt, g_peak_alt, g_kal_vel,
            pc13, pc14, pc15);
        UART_Print(uart_buf);

        /* ── SD write ── */
        if (loop_cnt % SD_WRITE_EVERY == 0) {
            /* FIX-SD1: icm_ay passed to SD_WriteFlight */
            SD_WriteFlight(
                adxl_ax, adxl_ay, adxl_az, adxl_g,
                icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz, icm_g,
                temperature, pressure, g_kal_alt, g_peak_alt, g_kal_vel,
                pc13, pc14, pc15);
            sd_row_idx++;
        }

        loop_cnt++;
        HAL_Delay(LOOP_DELAY_MS);
    }
}

/* ════════════════════════════════════════════════════════════════
   PERIPHERAL CONFIGURATION
   ════════════════════════════════════════════════════════════════ */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    o.HSIState            = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState        = RCC_PLL_ON;
    o.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    o.PLL.PLLM = 8; o.PLL.PLLN = 100;
    o.PLL.PLLP = RCC_PLLP_DIV2;
    o.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_I2C3_Init(void) {
    hi2c3.Instance             = I2C3;
    hi2c3.Init.ClockSpeed      = 100000;
    hi2c3.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1     = 0;
    hi2c3.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2     = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void) {
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 49999;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* PA0 — SD status LED */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* PA4 — SPI CS (deselected = HIGH) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    g.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &g);

    /* PB9 — Buzzer (silent = LOW) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);

    /* PC13/14/15 — Pyro (safe = LOW) */
    HAL_GPIO_WritePin(PYRO_PORT,
        PYRO_PC13_PIN | PYRO_PC14_PIN | PYRO_PC15_PIN, GPIO_PIN_RESET);
    g.Pin = PYRO_PC13_PIN | PYRO_PC14_PIN | PYRO_PC15_PIN;
    HAL_GPIO_Init(PYRO_PORT, &g);
}

void Error_Handler(void) {
    __disable_irq();
    Buzzer_Beep(3, 200, 150);
    LED_On();
    while (1) { HAL_GPIO_TogglePin(SD_LED_PORT, SD_LED_PIN); HAL_Delay(100); }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
