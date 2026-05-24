#ifndef BSP_LSM6DSR_H
#define BSP_LSM6DSR_H

#include <stdint.h>

/* ===================================================================
 * Configurable Macros (override via -D compiler flag)
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
#ifndef BSP_FILTER_ALPHA
#define BSP_FILTER_ALPHA               0.95f
#endif
#ifndef BSP_STATIONARY_FRAMES
#define BSP_STATIONARY_FRAMES          20
#endif
#ifndef BSP_BIAS_TRACK_RATE
#define BSP_BIAS_TRACK_RATE            0.001f
#endif
#ifndef BSP_BIAS_CLAMP
#define BSP_BIAS_CLAMP                 2.0f
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
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data);
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz);

#endif /* BSP_LSM6DSR_H */
