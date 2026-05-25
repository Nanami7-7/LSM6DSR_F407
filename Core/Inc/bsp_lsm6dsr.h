/**
 * @file    bsp_lsm6dsr.h
 * @brief   BSP 业务层 — 姿态互补滤波器与生产 API
 *
 * 三层架构的中间层，封装完整的 IMU 姿态估计算法：
 *   - 互补滤波器 (pitch/roll/yaw)
 *   - 自适应 α (运动时近纯 GYRO，静止时 ACC 主导)
 *   - 三重静止检测 (方差滑动窗口 + ACC 幅值 + GYRO 幅值)
 *   - Runtime 陀螺偏置跟踪 (X/Y 与 Z 轴独立速率)
 *   - DWT 周期计数器计时 (~6ns 精度)
 *   - VOFA+ 10 通道格式化输出
 *
 * 所有宏通过 #ifndef 定义，允许编译器 -D 覆盖。
 */
#ifndef BSP_LSM6DSR_H
#define BSP_LSM6DSR_H

#include <stdint.h>

/** @defgroup BSP_Calib 校准参数 */
/**@{*/
#ifndef BSP_CALIB_SAMPLES
#define BSP_CALIB_SAMPLES              100     /**< 校准采样帧数 */
#endif
#ifndef BSP_CALIB_SETTLE_MS
#define BSP_CALIB_SETTLE_MS            50      /**< 配置后稳定等待 (ms) */
#endif
#ifndef BSP_CALIB_ACC_MAG_REF
#define BSP_CALIB_ACC_MAG_REF          1000000.0f /**< ACC 幅值参考 (mg²) = 1G */
#endif
#ifndef BSP_CALIB_ACC_MAG_TOL
#define BSP_CALIB_ACC_MAG_TOL          65000.0f   /**< 幅值容差 (mg²) ≈ ±255mg */
#endif
#ifndef BSP_CALIB_ACC_DELTA_MAX
#define BSP_CALIB_ACC_DELTA_MAX        80.0f      /**< 帧间差分阈值 (mg) */
#endif
#ifndef BSP_CALIB_SAMPLE_DELAY_MS
#define BSP_CALIB_SAMPLE_DELAY_MS      9           /**< 采样间隔 (ms) */
#endif
/**@}*/

/** @defgroup BSP_Filter 自适应滤波器参数 */
/**@{*/
#ifndef BSP_ACC_VAR_WINDOW
#define BSP_ACC_VAR_WINDOW            10       /**< 方差滑动窗口大小 (帧) */
#endif
#ifndef BSP_ACC_VAR_THRESHOLD
#define BSP_ACC_VAR_THRESHOLD         800.0f   /**< 静止方差阈值 (mg²总和) */
#endif
#ifndef BSP_ALPHA_MOVING
#define BSP_ALPHA_MOVING              0.99f    /**< 运动时 α，近纯 GYRO (1% ACC) */
#endif
#ifndef BSP_ALPHA_STATIONARY
#define BSP_ALPHA_STATIONARY          0.30f    /**< 静止时 α，ACC 快速收敛 (70% ACC) */
#endif
#ifndef BSP_ALPHA_SMOOTH_STEP
#define BSP_ALPHA_SMOOTH_STEP         0.15f    /**< α 每帧最大变化量 */
#endif
/**@}*/

/** @defgroup BSP_Bias 偏置跟踪参数 */
/**@{*/
#ifndef BSP_BIAS_STATIONARY_RATE
#define BSP_BIAS_STATIONARY_RATE      0.05f    /**< X/Y 轴静止偏置跟踪速率 */
#endif
#ifndef BSP_BIAS_STATIONARY_RATE_Z
#define BSP_BIAS_STATIONARY_RATE_Z    0.005f   /**< Z 轴静止偏置跟踪速率 (慢10倍，防A→B→A误差) */
#endif
#ifndef BSP_GYRO_MOTION_THRESHOLD
#define BSP_GYRO_MOTION_THRESHOLD     5.0f     /**< 陀螺幅值运动阈值 (dps)，超过则强制判为运动 */
#endif
/**@}*/

/** @brief  IMU 姿态数据结构体 (10 通道输出) */
typedef struct {
    float ax, ay, az;       /**< 加速度   m/s²  (G × 9.80665) */
    float gx, gy, gz;       /**< 角速度   dps    (偏置补偿后) */
    float pitch, roll, yaw; /**< 姿态角   deg    (互补滤波) */
    float temperature;      /**< 温度     °C     */
} bsp_lsm6dsr_data_t;

/** @defgroup BSP_API 生产 API */
/**@{*/

/**
 * @brief  传感器初始化
 * @details 执行完整初始化流程:
 *          SW_RESET → I3C 禁用 → IF_INC+BDU → ACC 104Hz/4G →
 *          GYRO 104Hz/250dps → 滤波状态初始化 → DWT 使能 → 校准
 */
void bsp_lsm6dsr_init(void);

/**
 * @brief  陀螺零偏校准
 * @details 采集 BSP_CALIB_SAMPLES 帧，用 ACC 静止检测拒斥运动帧，
 *          有效帧过半时取均值作为偏置。可在运行时重复调用。
 */
void bsp_lsm6dsr_calibrate(void);

/**
 * @brief  姿态更新 (核心滤波)
 * @param  data 输出数据结构体指针
 * @details 每次调用完成一帧滤波:
 *          1. DWT 计时 → dt
 *          2. 读取 ACC/GYRO/TEMP
 *          3. 三重静止检测 (方差 + 幅值 + 陀螺)
 *          4. 偏置校正 + 运行时跟踪
 *          5. 自适应 α 平滑过渡
 *          6. 互补滤波器更新 (pitch/roll/yaw)
 *          7. 缓存 last_data
 */
void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data);

/**
 * @brief  获取当前陀螺零偏
 * @param[out] bx X 轴偏置 (dps)，允许 NULL
 * @param[out] by Y 轴偏置 (dps)，允许 NULL
 * @param[out] bz Z 轴偏置 (dps)，允许 NULL
 */
void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz);

/**
 * @brief  获取上一帧 ACC 方差总和
 * @return 方差总和 (mg²)，用于调试静止检测门限
 */
float bsp_lsm6dsr_get_last_variance(void);

/**
 * @brief  获取最新缓存数据 (只读)
 * @return 指向 last_data 的 const 指针
 */
const bsp_lsm6dsr_data_t* bsp_lsm6dsr_get_data(void);

/**
 * @brief  查询静止状态
 * @return 1=静止 / 0=运动
 */
int bsp_lsm6dsr_is_stationary(void);

/**
 * @brief  VOFA+ FireWater 格式化
 * @param[out] buf     输出缓冲区
 * @param[in]  buf_size 缓冲区大小
 * @param[in]  data     IMU 数据指针
 * @return 写入缓冲区的字符数 (不含 '\0')
 * @details 格式: "ax,ay,az,gx,gy,gz,pitch,roll,yaw,temp\\r\\n"
 *          10 通道逗号分隔，VOFA+ FireWater 协议。
 */
int bsp_lsm6dsr_vofa_format(char *buf, int buf_size, const bsp_lsm6dsr_data_t *data);

/**@}*/

#endif /* BSP_LSM6DSR_H */
