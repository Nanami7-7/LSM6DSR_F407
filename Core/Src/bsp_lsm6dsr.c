#include "bsp_lsm6dsr.h"
#include "lsm6dsr.h"
#include "main.h"
#include <stdio.h>
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
static int     stationary_frames;       /* consecutive stationary frames */
static float   prev_ax, prev_ay, prev_az; /* previous accel for delta check */
static int     initialized;             /* guard for update before init */

/* ------------------------------------------------------------------ */
void bsp_lsm6dsr_init(void)
{
    /* Full reset + wait */
    lsm6dsr_reset(&lsm6dsr_io);
    HAL_Delay(100);

    /* Sensor configuration (same as original phase17) */
    lsm6dsr_i3c_disable(&lsm6dsr_io);
    lsm6dsr_set_if_inc(&lsm6dsr_io, 1);
    lsm6dsr_set_bdu(&lsm6dsr_io, 1);
    lsm6dsr_accel_config(&lsm6dsr_io,
        LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(&lsm6dsr_io,
        LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(BSP_CALIB_SETTLE_MS);

    /* ---- Startup Gyro Bias Calibration ---- */
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

    /* ---- Initialise filter state ---- */
    {
        float ax0, ay0, az0, gx0, gy0, gz0;
        lsm6dsr_read_accel_float(&lsm6dsr_io, &ax0, &ay0, &az0,
                                 LSM6DSR_ACCEL_FS_4G);
        lsm6dsr_read_gyro_float(&lsm6dsr_io, &gx0, &gy0, &gz0,
                                LSM6DSR_GYRO_FS_250DPS);
        if (cal_ok) { gx0 -= bgx; gy0 -= bgy; gz0 -= bgz; }

        pitch = atan2(-ax0, sqrt(ay0*ay0 + az0*az0)) * 180.0 / M_PI;
        roll  = atan2( ay0, sqrt(ax0*ax0 + az0*az0)) * 180.0 / M_PI;
        yaw   = 0.0;
    }

    last_tick        = HAL_GetTick();
    stationary_frames = 0;
    prev_ax = prev_ay = prev_az = 0.0f;
    initialized = 1;

    printf("  BSP init done  (cal=%s, bias=%.4f,%.4f,%.4f)\r\n",
           cal_ok ? "OK" : "FAIL",
           (double)bgx, (double)bgy, (double)bgz);
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

    /* ---- bias correction + runtime tracking ---- */
    if (cal_ok)
    {
        float mag2 = fax*fax + fay*fay + faz*faz;
        if (fabsf(mag2 - BSP_CALIB_ACC_MAG_REF) < BSP_CALIB_ACC_MAG_TOL
            && fabsf(fax - prev_ax) < BSP_CALIB_ACC_DELTA_MAX
            && fabsf(fay - prev_ay) < BSP_CALIB_ACC_DELTA_MAX
            && fabsf(faz - prev_az) < BSP_CALIB_ACC_DELTA_MAX)
        {
            if (++stationary_frames >= BSP_STATIONARY_FRAMES)
            {
                bgx += BSP_BIAS_TRACK_RATE * (fgx - bgx);
                bgy += BSP_BIAS_TRACK_RATE * (fgy - bgy);
                bgz += BSP_BIAS_TRACK_RATE * (fgz - bgz);

                if (bgx >  BSP_BIAS_CLAMP) bgx =  BSP_BIAS_CLAMP;
                if (bgx < -BSP_BIAS_CLAMP) bgx = -BSP_BIAS_CLAMP;
                if (bgy >  BSP_BIAS_CLAMP) bgy =  BSP_BIAS_CLAMP;
                if (bgy < -BSP_BIAS_CLAMP) bgy = -BSP_BIAS_CLAMP;
                if (bgz >  BSP_BIAS_CLAMP) bgz =  BSP_BIAS_CLAMP;
                if (bgz < -BSP_BIAS_CLAMP) bgz = -BSP_BIAS_CLAMP;
            }
        }
        else
        {
            stationary_frames = 0;
        }
        prev_ax = fax; prev_ay = fay; prev_az = faz;

        fgx -= bgx;
        fgy -= bgy;
        fgz -= bgz;
    }

    /* ---- complementary filter ---- */
    double acc_pitch = atan2(-fax, sqrt(fay*fay + faz*faz)) * 180.0 / M_PI;
    double acc_roll  = atan2( fay, sqrt(fax*fax + faz*faz)) * 180.0 / M_PI;

    pitch = BSP_FILTER_ALPHA * (pitch + fgy * dt)
          + (1.0 - BSP_FILTER_ALPHA) * acc_pitch;
    roll  = BSP_FILTER_ALPHA * (roll  - fgx * dt)
          + (1.0 - BSP_FILTER_ALPHA) * acc_roll;
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
}

/* ------------------------------------------------------------------ */
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz)
{
    if (bx) *bx = bgx;
    if (by) *by = bgy;
    if (bz) *bz = bgz;
}
