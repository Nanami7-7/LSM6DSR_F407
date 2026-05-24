#include "bsp_lsm6dsr.h"
#include "lsm6dsr.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Platform I/O — instance owned by test_lsm6dsr.c */
extern lsm6dsr_io_t lsm6dsr_io;

/* ---- Runtime state (internal) ---- */
static float   bgx, bgy, bgz;          /* gyro bias (dps) */
static int     cal_ok;                  /* calibration success flag */
static double  pitch, roll, yaw;        /* filter state (deg) */
static uint32_t last_tick;              /* last HAL_GetTick() */
static int     initialized;             /* guard for update before init */

/* ---- Adaptive filter state ---- */
static float   ax_buf[BSP_ACC_VAR_WINDOW];
static float   ay_buf[BSP_ACC_VAR_WINDOW];
static float   az_buf[BSP_ACC_VAR_WINDOW];
static int     var_buf_idx;
static int     var_samples;
static float   alpha;                   /* current adaptive alpha */
static float   last_variance;           /* last computed variance (for debug) */

/* ---- Production API cache ---- */
static bsp_lsm6dsr_data_t last_data;
static int     is_stationary;

/* ---- sliding-window accel variance (robust stationary detection) ---- */
static float compute_acc_variance(void)
{
    if (var_samples < BSP_ACC_VAR_WINDOW) return 0.0f;

    float mx = 0, my = 0, mz = 0;
    for (int i = 0; i < BSP_ACC_VAR_WINDOW; i++) {
        mx += ax_buf[i]; my += ay_buf[i]; mz += az_buf[i];
    }
    mx /= BSP_ACC_VAR_WINDOW;
    my /= BSP_ACC_VAR_WINDOW;
    mz /= BSP_ACC_VAR_WINDOW;

    float vx = 0, vy = 0, vz = 0;
    for (int i = 0; i < BSP_ACC_VAR_WINDOW; i++) {
        float dx = ax_buf[i] - mx; vx += dx * dx;
        float dy = ay_buf[i] - my; vy += dy * dy;
        float dz = az_buf[i] - mz; vz += dz * dz;
    }
    vx /= BSP_ACC_VAR_WINDOW;
    vy /= BSP_ACC_VAR_WINDOW;
    vz /= BSP_ACC_VAR_WINDOW;

    last_variance = vx + vy + vz;
    return last_variance;
}

/* ------------------------------------------------------------------ */
void bsp_lsm6dsr_init(void)
{
    /* Full reset + wait */
    lsm6dsr_reset(&lsm6dsr_io);
    HAL_Delay(100);

    /* Debug: hardware info */
    {
        uint8_t whoami = 0;
        lsm6dsr_read_reg(&lsm6dsr_io, LSM6DSR_REG_WHO_AM_I, &whoami);
        printf("  I2C: DevAddr=0x%04X  WHO_AM_I=0x%02X\r\n",
               (unsigned)LSM6DSR_I2C_ADDR, (unsigned)whoami);
    }

    /* Sensor configuration (same as original phase17) */
    lsm6dsr_i3c_disable(&lsm6dsr_io);
    lsm6dsr_set_if_inc(&lsm6dsr_io, 1);
    lsm6dsr_set_bdu(&lsm6dsr_io, 1);
    lsm6dsr_accel_config(&lsm6dsr_io,
        LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(&lsm6dsr_io,
        LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(BSP_CALIB_SETTLE_MS);

    printf("  ACC ODR=104Hz FS=4G  GYRO ODR=104Hz FS=250dps\r\n");

    /* ---- Initialise filter state ---- */
    {
        float ax0, ay0, az0;
        lsm6dsr_read_accel_float(&lsm6dsr_io, &ax0, &ay0, &az0,
                                 LSM6DSR_ACCEL_FS_4G);

        pitch = atan2(-ax0, sqrt(ay0*ay0 + az0*az0)) * 180.0 / M_PI;
        roll  = atan2( ay0, sqrt(ax0*ax0 + az0*az0)) * 180.0 / M_PI;
        yaw   = 0.0;

        /* fill variance window with initial values */
        var_buf_idx = 0;
        var_samples = BSP_ACC_VAR_WINDOW;
        for (int i = 0; i < BSP_ACC_VAR_WINDOW; i++) {
            ax_buf[i] = ax0; ay_buf[i] = ay0; az_buf[i] = az0;
        }
        alpha = BSP_ALPHA_STATIONARY;
        last_variance = 0.0f;
    }

    printf("  Initial pitch=%.2f  roll=%.2f\r\n",
           (double)pitch, (double)roll);

    last_tick = HAL_GetTick();
    memset(&last_data, 0, sizeof(last_data));
    is_stationary = 1;
    initialized = 1;

    bsp_lsm6dsr_calibrate();

    printf("  BSP init done  (cal=%s, bias=%.4f,%.4f,%.4f)  alpha=%.2f\r\n",
           cal_ok ? "OK" : "FAIL",
           (double)bgx, (double)bgy, (double)bgz, alpha);
}
 
/* ------------------------------------------------------------------ */
void bsp_lsm6dsr_calibrate(void)
{
    bgx = 0.0f; bgy = 0.0f; bgz = 0.0f;
    cal_ok = 0;
    {
        int   n_valid = 0;
        float pax, pay, paz;

        lsm6dsr_read_accel_float(&lsm6dsr_io, &pax, &pay, &paz,
                                 LSM6DSR_ACCEL_FS_4G);

        printf("  Calibrating gyro bias (%d samples, keep still)...\r\n",
               BSP_CALIB_SAMPLES);

        for (int i = 0; i < BSP_CALIB_SAMPLES; i++)
        {
            float tax, tay, taz, tgx, tgy, tgz;
            lsm6dsr_read_accel_float(&lsm6dsr_io, &tax, &tay, &taz,
                                     LSM6DSR_ACCEL_FS_4G);
            lsm6dsr_read_gyro_float(&lsm6dsr_io, &tgx, &tgy, &tgz,
                                    LSM6DSR_GYRO_FS_250DPS);

            float mag2 = tax*tax + tay*tay + taz*taz;
            if (fabsf(mag2 - BSP_CALIB_ACC_MAG_REF) < BSP_CALIB_ACC_MAG_TOL
                && fabsf(tax - pax) < BSP_CALIB_ACC_DELTA_MAX
                && fabsf(tay - pay) < BSP_CALIB_ACC_DELTA_MAX
                && fabsf(taz - paz) < BSP_CALIB_ACC_DELTA_MAX)
            {
                bgx += tgx; bgy += tgy; bgz += tgz;
                n_valid++;
            }
            pax = tax; pay = tay; paz = taz;
            HAL_Delay(BSP_CALIB_SAMPLE_DELAY_MS);
        }

        if (n_valid >= BSP_CALIB_SAMPLES / 2)
        {
            bgx /= (float)n_valid;
            bgy /= (float)n_valid;
            bgz /= (float)n_valid;
            cal_ok = 1;
            printf("  Gyro bias: X=%.4f  Y=%.4f  Z=%.4f dps  (%d/%d OK)\r\n",
                   (double)bgx, (double)bgy, (double)bgz,
                   n_valid, BSP_CALIB_SAMPLES);
        }
        else
        {
            bgx = bgy = bgz = 0.0f;
            printf("  Warning: too few stationary samples (%d/%d), bias=0\r\n",
                   n_valid, BSP_CALIB_SAMPLES);
        }
    }

    /* read one frame to print residual */
    {
        float gx0, gy0, gz0;
        lsm6dsr_read_gyro_float(&lsm6dsr_io, &gx0, &gy0, &gz0,
                                LSM6DSR_GYRO_FS_250DPS);
        if (cal_ok) { gx0 -= bgx; gy0 -= bgy; gz0 -= bgz; }
        printf("  GYRO residual after cal: X=%.4f  Y=%.4f  Z=%.4f dps\r\n",
               (double)gx0, (double)gy0, (double)gz0);
    }
}

/* ------------------------------------------------------------------ */
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data)
{
    if (!initialized) return;

    /* ---- timing ---- */
    uint32_t now = HAL_GetTick();
    double dt = (now > last_tick) ? (now - last_tick) * 0.001 : 0.01;
    if (dt > 0.5) dt = 0.01;
    last_tick = now;

    /* ---- read sensors ---- */
    float fax, fay, faz, fgx, fgy, fgz;
    lsm6dsr_read_accel_float(&lsm6dsr_io, &fax, &fay, &faz,
                             LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_read_gyro_float(&lsm6dsr_io, &fgx, &fgy, &fgz,
                            LSM6DSR_GYRO_FS_250DPS);
    lsm6dsr_read_temp(&lsm6dsr_io, &data->temperature);

    /* ---- update sliding window ---- */
    ax_buf[var_buf_idx] = fax;
    ay_buf[var_buf_idx] = fay;
    az_buf[var_buf_idx] = faz;
    var_buf_idx = (var_buf_idx + 1) % BSP_ACC_VAR_WINDOW;
    if (var_samples < BSP_ACC_VAR_WINDOW) var_samples++;

    /* ---- variance-based stationary detection ---- */
    float var_sum = compute_acc_variance();
    int stationary = (var_sum < BSP_ACC_VAR_THRESHOLD);
    /* dual-check: accel magnitude must be near 1G */
    float mag2 = fax*fax + fay*fay + faz*faz;
    if (fabsf(mag2 - BSP_CALIB_ACC_MAG_REF) >= BSP_CALIB_ACC_MAG_TOL) {
        stationary = 0;
    }

    /* ---- bias correction ---- */
    if (cal_ok) {
        fgx -= bgx; fgy -= bgy; fgz -= bgz;

        /* triple-check: gyro magnitude rejects bias-eating pure rotation */
        float gyro_mag2 = fgx*fgx + fgy*fgy + fgz*fgz;
        if (gyro_mag2 > (BSP_GYRO_MOTION_THRESHOLD * BSP_GYRO_MOTION_THRESHOLD)) {
            stationary = 0;
        }

        /* ---- runtime bias tracking (only when stationary) ---- */
        if (stationary) {
            bgx += BSP_BIAS_STATIONARY_RATE * fgx;
            bgy += BSP_BIAS_STATIONARY_RATE * fgy;
            bgz += BSP_BIAS_STATIONARY_RATE * fgz;
        }
    }

    /* ---- smooth alpha transition ---- */
    float target_alpha = stationary ? BSP_ALPHA_STATIONARY : BSP_ALPHA_MOVING;
    if (alpha < target_alpha) {
        alpha += BSP_ALPHA_SMOOTH_STEP;
        if (alpha > target_alpha) alpha = target_alpha;
    } else if (alpha > target_alpha) {
        alpha -= BSP_ALPHA_SMOOTH_STEP;
        if (alpha < target_alpha) alpha = target_alpha;
    }

    /* ---- complementary filter ---- */
    double acc_pitch = atan2(-fax, sqrt(fay*fay + faz*faz)) * 180.0 / M_PI;
    double acc_roll  = atan2( fay, sqrt(fax*fax + faz*faz)) * 180.0 / M_PI;

    pitch = (double)alpha * (pitch + fgy * dt) + (1.0 - (double)alpha) * acc_pitch;
    roll  = (double)alpha * (roll  - fgx * dt) + (1.0 - (double)alpha) * acc_roll;
    yaw  += fgz * dt;

    /* ---- fill result struct ---- */
    data->ax    = fax * 0.00980665f;
    data->ay    = fay * 0.00980665f;
    data->az    = faz * 0.00980665f;
    data->gx    = fgx;
    data->gy    = fgy;
    data->gz    = fgz;
    data->pitch = (float)pitch;
    data->roll  = (float)roll;
    data->yaw   = (float)yaw;

    /* ---- cache for production API ---- */
    last_data = *data;
    is_stationary = stationary;
}

/* ------------------------------------------------------------------ */
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz)
{
    if (bx) *bx = bgx;
    if (by) *by = bgy;
    if (bz) *bz = bgz;
}

float bsp_lsm6dsr_get_last_variance(void)
{
    return last_variance;
}

/* ------------------------------------------------------------------ */
const bsp_lsm6dsr_data_t* bsp_lsm6dsr_get_data(void)
{
    return &last_data;
}

int bsp_lsm6dsr_is_stationary(void)
{
    return is_stationary;
}

/* ------------------------------------------------------------------ */
int bsp_lsm6dsr_vofa_format(char *buf, int buf_size, const bsp_lsm6dsr_data_t *data)
{
    return snprintf(buf, buf_size,
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\r\n",
        (double)data->ax, (double)data->ay, (double)data->az,
        (double)data->gx, (double)data->gy, (double)data->gz,
        data->pitch, data->roll, data->yaw, data->temperature);
}
