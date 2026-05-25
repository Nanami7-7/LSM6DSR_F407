/**
 * @file    bsp_lsm6dsr.c
 * @brief   BSP 业务层 — 互补滤波器与姿态估计算法实现
 *
 * 核心算法与功能:
 *   - 互补滤波器 (Complementary Filter)：融合 ACC 低频姿态 + GYRO 积分
 *   - 自适应 α：运动时 α=0.99 (近纯 GYRO)，静止时 α=0.30 (ACC 主导收敛)
 *   - 三重静止检测：
 *       1. ACC 方差滑动窗口 (抗振动干扰)
 *       2. ACC 幅值校验 (|mag² - 1G| < tol)
 *       3. GYRO 幅值校验 (|gyro| > threshold → 运动)
 *   - Runtime 偏置跟踪：静止时 bg += rate × fg (X/Y vs Z 独立速率)
 *   - DWT 周期计数器计时 (~6ns 精度，替代 HAL_GetTick 的 1ms 抖动)
 *   - VOFA+ 格式化：10 通道 FireWater 协议
 */
#include "bsp_lsm6dsr.h"
#include "lsm6dsr.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** @brief 平台 I/O 实例 (定义于 test_lsm6dsr.c) */
extern lsm6dsr_io_t lsm6dsr_io;

/* ---- Runtime state (internal) ---- */
static float   bgx, bgy, bgz;          /**< 陀螺偏置 (dps) */
static int     cal_ok;                  /**< 校准成功标志 */
static double  pitch, roll, yaw;        /**< 滤波器状态 (deg) */
static uint32_t last_tick;              /**< 上一帧 DWT CYCCNT (~6ns 分辨率) */
static int     initialized;             /**< init 完成守护标志 */

/* ---- Adaptive filter state ---- */
static float   ax_buf[BSP_ACC_VAR_WINDOW]; /**< ACC X 滑动窗口 */
static float   ay_buf[BSP_ACC_VAR_WINDOW]; /**< ACC Y 滑动窗口 */
static float   az_buf[BSP_ACC_VAR_WINDOW]; /**< ACC Z 滑动窗口 */
static int     var_buf_idx;                /**< 窗口循环索引 */
static int     var_samples;                /**< 已采帧数 */
static float   alpha;                      /**< 当前互补滤波 α */
static float   last_variance;              /**< 上一帧方差 (调试用) */

/* ---- Production API cache ---- */
static bsp_lsm6dsr_data_t last_data; /**< 最新数据缓存 */
static int     is_stationary;         /**< 最新静止状态 */

/**
 * @brief  计算 ACC 3 轴方差总和
 * @return 方差总和 (mg²)，窗口未填满时返回 0
 * @details 滑动窗口方差:
 *          1. 计算窗口内 X/Y/Z 均值
 *          2. 计算每轴方差
 *          3. 返回三轴方差之和
 *          方差小 (< BSP_ACC_VAR_THRESHOLD) → 静止
 */
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
/**
 * @brief  传感器初始化
 * @details 完整初始化序列:
 *          1. SW_RESET → 等待 100ms
 *          2. WHO_AM_I 验证 (printf debug)
 *          3. I3C 禁用 → IF_INC 使能 → BDU 使能
 *          4. ACC 104Hz / ±4G
 *          5. GYRO 104Hz / ±250dps → 稳定等待
 *          6. 读取初始 ACC 计算初始 pitch/roll
 *          7. 填充方差窗口 (初始值)
 *          8. DWT 周期计数器使能 → 清零
 *          9. 调用 bsp_lsm6dsr_calibrate()
 *          10. 打印完成信息
 */
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

    /* switch to DWT cycle counter for sub-us dt precision */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_tick = DWT->CYCCNT;
    memset(&last_data, 0, sizeof(last_data));
    is_stationary = 1;
    initialized = 1;

    bsp_lsm6dsr_calibrate();

    printf("  BSP init done  (cal=%s, bias=%.4f,%.4f,%.4f)  alpha=%.2f\r\n",
           cal_ok ? "OK" : "FAIL",
           (double)bgx, (double)bgy, (double)bgz, alpha);
}
 
/* ------------------------------------------------------------------ */
/**
 * @brief  陀螺零偏校准
 * @details 采集 BSP_CALIB_SAMPLES 帧陀螺数据，每帧用 ACC 双重检测：
 *          - 幅值检测: |mag² - 1G| < BSP_CALIB_ACC_MAG_TOL
 *          - 帧间差分: |ax - ax_prev| < BSP_CALIB_ACC_DELTA_MAX 等
 *          有效帧 ≥ 50% 时才采用均值偏置，否则偏置归零。
 *          校准完成后读取一帧打印残差。
 *
 * @note 可在运行时重复调用（机器狗站定时重新校准）。
 */
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
/**
 * @brief  姿态更新 (核心滤波)
 * @param  data 输出数据结构体指针 (不能为 NULL)
 *
 * @details 每帧依次执行:
 *
 *  **1. 计时**: DWT->CYCCNT 差值 / SystemCoreClock → dt (秒)
 *     首帧或长时间暂停 (>0.5s) 时 dt 强制为 0.01s。
 *
 *  **2. 传感器读取**: ACC (g), GYRO (dps), TEMP (°C)
 *
 *  **3. 方差滑动窗口**: 更新 ACC 缓冲，满窗口时计算三轴方差总和。
 *
 *  **4. 三重静止检测**:
 *     - 方差 < BSP_ACC_VAR_THRESHOLD (抗振动)
 *     - |mag² - 1G| < BSP_CALIB_ACC_MAG_TOL (排除线加速度)
 *     - 偏置补偿后 |gyro| > BSP_GYRO_MOTION_THRESHOLD → 强制运动 (防偏置吃掉旋转)
 *
 *  **5. 偏置校正与跟踪**:
 *     - 校准有效时 fg -= bg
 *     - 静止时 bg += rate × fg (X/Y 用 0.05, Z 用 0.005)
 *
 *  **6. 自适应 α 平滑**:
 *     - 目标 α = 静止 ? BSP_ALPHA_STATIONARY : BSP_ALPHA_MOVING
 *     - 每帧向目标靠近 BSP_ALPHA_SMOOTH_STEP
 *
 *  **7. 互补滤波器**:
 *     - pitch = α × (pitch + gyro_pitch × dt) + (1-α) × acc_pitch
 *     - roll  = α × (roll  - gyro_roll  × dt) + (1-α) × acc_roll
 *     - yaw  += gz × dt  (纯 GYRO 积分，无 ACC 参考)
 *
 *  **8. 结果填充**: data→ax/ay/az 为 m/s²; gx/gy/gz 为偏置补偿后 dps
 */
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data)
{
    if (!initialized) return;

    /* ---- timing (DWT cycle counter, ~6ns resolution) ---- */
    uint32_t now = DWT->CYCCNT;
    double dt = (now - last_tick) / (double)SystemCoreClock;
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
            bgz += BSP_BIAS_STATIONARY_RATE_Z * fgz;
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
/**
 * @brief  获取陀螺零偏
 * @param[out] bx X 轴偏置 (dps)，允许 NULL
 * @param[out] by Y 轴偏置 (dps)，允许 NULL
 * @param[out] bz Z 轴偏置 (dps)，允许 NULL
 */
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz)
{
    if (bx) *bx = bgx;
    if (by) *by = bgy;
    if (bz) *bz = bgz;
}

/**
 * @brief  获取上一帧 ACC 方差总和
 * @return 方差总和 (mg²)
 */
float bsp_lsm6dsr_get_last_variance(void)
{
    return last_variance;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  获取最新缓存数据 (只读)
 * @return 指向内部 last_data 的 const 指针
 * @note   返回的指针在下次 update 调用前有效
 */
const bsp_lsm6dsr_data_t* bsp_lsm6dsr_get_data(void)
{
    return &last_data;
}

/**
 * @brief  查询静止状态
 * @return 1=静止 / 0=运动
 */
int bsp_lsm6dsr_is_stationary(void)
{
    return is_stationary;
}

/* ------------------------------------------------------------------ */
/**
 * @brief  VOFA+ FireWater 10 通道格式化
 * @param[out] buf     输出缓冲区
 * @param[in]  buf_size 缓冲区大小
 * @param[in]  data     IMU 数据指针
 * @return 写入缓冲区的字符数
 *
 * @details 格式: "ax,ay,az,gx,gy,gz,pitch,roll,yaw,temp\\r\\n"
 *          - ax/ay/az: m/s² (%.3f)
 *          - gx/gy/gz: dps (%.3f)
 *          - pitch/roll/yaw: deg (%.2f)
 *          - temp: °C (%.1f)
 *
 *          VOFA+ 选择 FireWater 协议，10 通道自动对应。
 */
int bsp_lsm6dsr_vofa_format(char *buf, int buf_size, const bsp_lsm6dsr_data_t *data)
{
    return snprintf(buf, buf_size,
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\r\n",
        (double)data->ax, (double)data->ay, (double)data->az,
        (double)data->gx, (double)data->gy, (double)data->gz,
        data->pitch, data->roll, data->yaw, data->temperature);
}
