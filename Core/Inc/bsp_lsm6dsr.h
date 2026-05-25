#ifndef BSP_LSM6DSR_H
#define BSP_LSM6DSR_H

#include <stdint.h>

/* ===================================================================
 * Configurable Macros (override via -D compiler flag)
 * =================================================================== */
/* ===================================================================
 * Startup Calibration Macros
 * =================================================================== */
#ifndef BSP_CALIB_SAMPLES
#define BSP_CALIB_SAMPLES              100
#endif
#ifndef BSP_CALIB_SETTLE_MS
#define BSP_CALIB_SETTLE_MS            50
#endif
#ifndef BSP_CALIB_ACC_MAG_REF
#define BSP_CALIB_ACC_MAG_REF          1000000.0f
#endif
#ifndef BSP_CALIB_ACC_MAG_TOL
#define BSP_CALIB_ACC_MAG_TOL          65000.0f
#endif
#ifndef BSP_CALIB_ACC_DELTA_MAX
#define BSP_CALIB_ACC_DELTA_MAX        80.0f
#endif
#ifndef BSP_CALIB_SAMPLE_DELAY_MS
#define BSP_CALIB_SAMPLE_DELAY_MS      9
#endif

/* ===================================================================
 * Adaptive Filter Macros
 * =================================================================== */
#ifndef BSP_ACC_VAR_WINDOW
#define BSP_ACC_VAR_WINDOW            10       /* 方差滑动窗口(帧) */
#endif
#ifndef BSP_ACC_VAR_THRESHOLD
#define BSP_ACC_VAR_THRESHOLD         800.0f   /* 静止判定阈值(mg²总和) */
#endif
#ifndef BSP_ALPHA_MOVING
#define BSP_ALPHA_MOVING              0.99f    /* 运动时α，近纯GYRO */
#endif
#ifndef BSP_ALPHA_STATIONARY
#define BSP_ALPHA_STATIONARY          0.30f    /* 静止时α，快ACC收敛 */
#endif
#ifndef BSP_ALPHA_SMOOTH_STEP
#define BSP_ALPHA_SMOOTH_STEP         0.15f    /* α每帧最大变化 */
#endif
#ifndef BSP_BIAS_STATIONARY_RATE
#define BSP_BIAS_STATIONARY_RATE      0.05f    /* 静止偏置跟踪速率 (X/Y) */
#endif
#ifndef BSP_BIAS_STATIONARY_RATE_Z
#define BSP_BIAS_STATIONARY_RATE_Z    0.005f   /* Z轴静止偏置跟踪速率(慢速，防不对称) */
#endif
#ifndef BSP_GYRO_MOTION_THRESHOLD
#define BSP_GYRO_MOTION_THRESHOLD     5.0f     /* dps — 超过此值判定为运动 */
#endif

/* ===================================================================
 * Data Structure
 * =================================================================== */
typedef struct {
    float ax, ay, az;       /* acceleration   m/s   */
    float gx, gy, gz;       /* angular rate   dps   (bias-compensated) */
    float pitch, roll, yaw; /* orientation    deg   */
    float temperature;      /* temperature    deg C */
} bsp_lsm6dsr_data_t;

/* ===================================================================
 * API
 * =================================================================== */
void bsp_lsm6dsr_init(void);
void bsp_lsm6dsr_calibrate(void);
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data);
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz);
float bsp_lsm6dsr_get_last_variance(void);
const bsp_lsm6dsr_data_t* bsp_lsm6dsr_get_data(void);
int                        bsp_lsm6dsr_is_stationary(void);
int bsp_lsm6dsr_vofa_format(char *buf, int buf_size, const bsp_lsm6dsr_data_t *data);

#endif /* BSP_LSM6DSR_H */
