/**
 * @file    lsm6dsr.h
 * @brief   LSM6DSR 驱动层 — 寄存器映射、I/O 抽象、枚举类型与函数原型
 *
 * 三层架构中的最底层，提供平台无关的 LSM6DSR 传感器驱动：
 *   - lsm6dsr_io_t 抽象 I2C/SPI 读写
 *   - 完整的寄存器地址映射
 *   - ACC/GYRO/TEMP 数据读取（raw + float）
 *   - FIFO 全部模式操作
 *   - 自检、功耗模式、BDU/IF_INC 控制
 *
 * 上层（BSP 层）通过 lsm6dsr_io_t 实例调用本层函数。
 */
#ifndef LSM6DSR_H
#define LSM6DSR_H

#include <stdint.h>
#include <stddef.h>

/** @brief Platform I/O abstraction — 注册读写回调函数 */
typedef struct {
    int8_t (*read)(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);   /**< 多字节读回调 */
    int8_t (*write)(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len); /**< 写回调 */
    void *ctx; /**< 平台上下文指针 (如 I2C_HandleTypeDef*) */
} lsm6dsr_io_t;

#ifdef __cplusplus
extern "C" {
#endif

/** @name I2C 地址与器件 ID */
/**@{*/
#define LSM6DSR_I2C_ADDR         (0x6A << 1) /**< LSM6DSR 7-bit I2C 地址 (左移1位后 0xD4) */
#define LSM6DSR_WHO_AM_I_VAL     0x6B        /**< WHO_AM_I 期望值 */
/**@}*/

/** @name 寄存器地址映射 */
/**@{*/
#define LSM6DSR_REG_FIFO_CTRL1      0x07 /**< FIFO 水印发/批速率配置 */
#define LSM6DSR_REG_FIFO_CTRL2      0x08 /**< FIFO 水印高位/停止条件 */
#define LSM6DSR_REG_FIFO_CTRL3      0x09 /**< FIFO 批速率 ACC/GYRO */
#define LSM6DSR_REG_FIFO_CTRL4      0x0A /**< FIFO 模式 */
#define LSM6DSR_REG_WHO_AM_I        0x0F /**< WHO_AM_I (期望 0x6B) */
#define LSM6DSR_REG_CTRL1_XL        0x10 /**< ACC ODR + 满量程 */
#define LSM6DSR_REG_CTRL2_G         0x11 /**< GYRO ODR + 满量程 */
#define LSM6DSR_REG_CTRL3_C         0x12 /**< BOOT/BDU/H_LACTIVE/IF_INC/SW_RESET */
#define LSM6DSR_REG_CTRL4_C         0x13 /**< SLEEP_G/DRDY_MASK/I2C_DISABLE/LPF1_SEL */
#define LSM6DSR_REG_CTRL5_C         0x14 /**< ACC/GYRO 自检、ROUNDING */
#define LSM6DSR_REG_CTRL6_C         0x15 /**< ACC 高性能模式/滤波类型 */
#define LSM6DSR_REG_CTRL7_G         0x16 /**< GYRO 高性能/高通滤波/OIS */
#define LSM6DSR_REG_CTRL8_XL        0x17 /**< ACC 滤波设置 */
#define LSM6DSR_REG_CTRL9_XL        0x18 /**< I3C 禁用等 */
#define LSM6DSR_REG_STATUS_REG      0x1E /**< DRDY 状态 */
#define LSM6DSR_REG_OUT_TEMP_L      0x20 /**< 温度低字节 */
#define LSM6DSR_REG_OUT_TEMP_H      0x21 /**< 温度高字节 */
#define LSM6DSR_REG_OUTX_L_G        0x22 /**< GYRO X 低字节 */
#define LSM6DSR_REG_OUTX_H_G        0x23 /**< GYRO X 高字节 */
#define LSM6DSR_REG_OUTY_L_G        0x24 /**< GYRO Y 低字节 */
#define LSM6DSR_REG_OUTY_H_G        0x25 /**< GYRO Y 高字节 */
#define LSM6DSR_REG_OUTZ_L_G        0x26 /**< GYRO Z 低字节 */
#define LSM6DSR_REG_OUTZ_H_G        0x27 /**< GYRO Z 高字节 */
#define LSM6DSR_REG_OUTX_L_XL       0x28 /**< ACC X 低字节 */
#define LSM6DSR_REG_OUTX_H_XL       0x29 /**< ACC X 高字节 */
#define LSM6DSR_REG_OUTY_L_XL       0x2A /**< ACC Y 低字节 */
#define LSM6DSR_REG_OUTY_H_XL       0x2B /**< ACC Y 高字节 */
#define LSM6DSR_REG_OUTZ_L_XL       0x2C /**< ACC Z 低字节 */
#define LSM6DSR_REG_OUTZ_H_XL       0x2D /**< ACC Z 高字节 */
#define LSM6DSR_REG_FIFO_STATUS1     0x3A /**< FIFO 水印/满/溢出状态 */
#define LSM6DSR_REG_FIFO_STATUS2     0x3B /**< FIFO 智能水印/计数器 */
#define LSM6DSR_REG_FIFO_DATA_OUT_TAG 0x78 /**< FIFO 数据标签 */
#define LSM6DSR_REG_FIFO_DATA_OUT_XL  0x79 /**< FIFO 数据低字节 */
#define LSM6DSR_REG_INT1_CTRL       0x0D /**< INT1 中断控制 */
#define LSM6DSR_REG_INT2_CTRL       0x0E /**< INT2 中断控制 */
/**@}*/

/** @name CTRL3_C 位掩码 */
/**@{*/
#define CTRL3_C_SW_RESET    (1<<0)  /**< 软件复位 */
#define CTRL3_C_BDU         (1<<6)  /**< 块数据更新 (输出寄存器在读取前不更新) */
#define CTRL3_C_IF_INC      (1<<2)  /**< 寄存器地址自动递增 (多字节读取) */
#define CTRL3_C_BOOT        (1<<7)  /**< 重新加载校准参数 */
/**@}*/

#define CTRL9_XL_I3C_DISABLE  (1<<1) /**< 禁用 I3C 接口 */

/** @name STATUS_REG 数据就绪标志 */
/**@{*/
#define STATUS_REG_DRDY_XL  (1<<0) /**< ACC 数据就绪 */
#define STATUS_REG_DRDY_G   (1<<1) /**< GYRO 数据就绪 */
#define STATUS_REG_DRDY_TEMP (1<<2) /**< 温度数据就绪 */
/**@}*/

/** @name FIFO 模式 */
/**@{*/
#define FIFO_CTRL4_FIFO_MODE_MASK   0x07
#define FIFO_MODE_BYPASS            0x00 /**< 旁路模式 */
#define FIFO_MODE_FIFO              0x01 /**< FIFO 模式 (满则停) */
#define FIFO_MODE_CONT_TO_FIFO      0x03 /**< 连续→FIFO 模式 */
#define FIFO_MODE_CONT              0x06 /**< 连续模式 */
/**@}*/

/** @name FIFO 批速率 (BDR) */
/**@{*/
#define LSM6DSR_BDR_NOT_BATCHED  0x00
#define LSM6DSR_BDR_12Hz5        0x01
#define LSM6DSR_BDR_26Hz         0x02
#define LSM6DSR_BDR_52Hz         0x03
#define LSM6DSR_BDR_104Hz        0x04
#define LSM6DSR_BDR_208Hz        0x05
#define LSM6DSR_BDR_416Hz        0x06
#define LSM6DSR_BDR_833Hz        0x07
/**@}*/

/** @name 状态码 */
/**@{*/
typedef enum {
    LSM6DSR_OK             = 0x00, /**< 操作成功 */
    LSM6DSR_ERROR          = 0x01, /**< 通信错误 */
    LSM6DSR_TIMEOUT        = 0x02, /**< 操作超时 */
    LSM6DSR_NOT_FOUND      = 0x03, /**< 器件 ID 不匹配 */
    LSM6DSR_INVALID_PARAM  = 0x04, /**< 无效参数 */
    LSM6DSR_NULL_PTR       = 0x05  /**< 空指针 */
} lsm6dsr_status_t;
/**@}*/

/** @brief ACC 输出数据速率 */
typedef enum {
    LSM6DSR_ACCEL_ODR_OFF     = 0x00, /**< 关闭 */
    LSM6DSR_ACCEL_ODR_12_5HZ  = 0x01, /**< 12.5 Hz */
    LSM6DSR_ACCEL_ODR_26HZ    = 0x02, /**< 26 Hz */
    LSM6DSR_ACCEL_ODR_52HZ    = 0x03, /**< 52 Hz */
    LSM6DSR_ACCEL_ODR_104HZ   = 0x04, /**< 104 Hz */
    LSM6DSR_ACCEL_ODR_208HZ   = 0x05, /**< 208 Hz */
    LSM6DSR_ACCEL_ODR_416HZ   = 0x06, /**< 416 Hz */
    LSM6DSR_ACCEL_ODR_833HZ   = 0x07, /**< 833 Hz */
    LSM6DSR_ACCEL_ODR_1_66KHZ = 0x08, /**< 1.66 kHz */
    LSM6DSR_ACCEL_ODR_3_33KHZ = 0x09, /**< 3.33 kHz */
    LSM6DSR_ACCEL_ODR_6_66KHZ = 0x0A  /**< 6.66 kHz */
} lsm6dsr_accel_odr_t;

/** @brief ACC 满量程 */
typedef enum {
    LSM6DSR_ACCEL_FS_2G  = 0x00, /**< ±2G */
    LSM6DSR_ACCEL_FS_4G  = 0x02, /**< ±4G */
    LSM6DSR_ACCEL_FS_8G  = 0x03, /**< ±8G */
    LSM6DSR_ACCEL_FS_16G = 0x01  /**< ±16G */
} lsm6dsr_accel_fs_t;

/** @brief GYRO 输出数据速率 (复用 ACC 枚举) */
typedef lsm6dsr_accel_odr_t lsm6dsr_gyro_odr_t;
#define LSM6DSR_GYRO_ODR_OFF      LSM6DSR_ACCEL_ODR_OFF
#define LSM6DSR_GYRO_ODR_12_5HZ  LSM6DSR_ACCEL_ODR_12_5HZ
#define LSM6DSR_GYRO_ODR_26HZ    LSM6DSR_ACCEL_ODR_26HZ
#define LSM6DSR_GYRO_ODR_52HZ    LSM6DSR_ACCEL_ODR_52HZ
#define LSM6DSR_GYRO_ODR_104HZ   LSM6DSR_ACCEL_ODR_104HZ
#define LSM6DSR_GYRO_ODR_208HZ   LSM6DSR_ACCEL_ODR_208HZ
#define LSM6DSR_GYRO_ODR_416HZ   LSM6DSR_ACCEL_ODR_416HZ
#define LSM6DSR_GYRO_ODR_833HZ   LSM6DSR_ACCEL_ODR_833HZ
#define LSM6DSR_GYRO_ODR_1_66KHZ LSM6DSR_ACCEL_ODR_1_66KHZ
#define LSM6DSR_GYRO_ODR_3_33KHZ LSM6DSR_ACCEL_ODR_3_33KHZ
#define LSM6DSR_GYRO_ODR_6_66KHZ LSM6DSR_ACCEL_ODR_6_66KHZ

/** @brief GYRO 满量程 */
typedef enum {
    LSM6DSR_GYRO_FS_250DPS  = 0,  /**< ±250 dps */
    LSM6DSR_GYRO_FS_500DPS  = 4,  /**< ±500 dps */
    LSM6DSR_GYRO_FS_1000DPS = 8,  /**< ±1000 dps */
    LSM6DSR_GYRO_FS_2000DPS = 12  /**< ±2000 dps */
} lsm6dsr_gyro_fs_t;

/** @brief 三轴数据结构 (int16 raw) */
typedef struct {
    int16_t x; /**< X 轴 */
    int16_t y; /**< Y 轴 */
    int16_t z; /**< Z 轴 */
} lsm6dsr_axis_t;

/** @name 灵敏度常量 (mg/LSB 或 dps/LSB) */
/**@{*/
#define LSM6DSR_ACCEL_SENS_2G   0.061f   /**< ±2G  灵敏度: 0.061 mg/LSB */
#define LSM6DSR_ACCEL_SENS_4G   0.122f   /**< ±4G  灵敏度: 0.122 mg/LSB */
#define LSM6DSR_ACCEL_SENS_8G   0.244f   /**< ±8G  灵敏度: 0.244 mg/LSB */
#define LSM6DSR_ACCEL_SENS_16G  0.488f   /**< ±16G 灵敏度: 0.488 mg/LSB */

#define LSM6DSR_GYRO_SENS_250DPS   0.00875f  /**< ±250  dps 灵敏度: 0.00875 dps/LSB */
#define LSM6DSR_GYRO_SENS_500DPS  0.01750f  /**< ±500  dps 灵敏度: 0.0175 dps/LSB */
#define LSM6DSR_GYRO_SENS_1000DPS 0.03500f  /**< ±1000 dps 灵敏度: 0.035 dps/LSB */
#define LSM6DSR_GYRO_SENS_2000DPS 0.07000f  /**< ±2000 dps 灵敏度: 0.07 dps/LSB */
/**@}*/

/** @name 温度传感器 */
/**@{*/
#define LSM6DSR_TEMP_SENSITIVITY   256.0f /**< 温度灵敏度 (LSB/°C) */
#define LSM6DSR_TEMP_OFFSET        25.0f /**< 25°C 时输出为 0 */
/**@}*/

/** @name FIFO 标签解码 */
/**@{*/
#define FIFO_TAG_SENSOR(tag)     ((tag) & 0x1F)
#define FIFO_TAG_CNT(tag)        (((tag) >> 5) & 0x03)
#define FIFO_TAG_PARITY(tag)     (((tag) >> 7) & 0x01)
/**
 * @brief 判断 FIFO 标签是否为 GYRO
 * @note  LSM6DSR FIFO tag 编码与 ST 文档不同:
 *        bit4=0 → GYRO, bit4=1 → ACC
 */
#define FIFO_TAG_IS_GYRO(tag)   (!((tag) & 0x10))
#define FIFO_TAG_IS_ACC(tag)    (((tag) & 0x10) != 0)

#define FIFO_TAG_GYRO  0
#define FIFO_TAG_ACC   1
/**@}*/

/** @name 自检模式 */
/**@{*/
#define LSM6DSR_XL_ST_DISABLE   0 /**< ACC 自检关闭 */
#define LSM6DSR_XL_ST_POSITIVE  1 /**< ACC 正向自检 */
#define LSM6DSR_XL_ST_NEGATIVE  2 /**< ACC 负向自检 */

#define LSM6DSR_GY_ST_DISABLE   0 /**< GYRO 自检关闭 */
#define LSM6DSR_GY_ST_POSITIVE  1 /**< GYRO 正向自检 */
#define LSM6DSR_GY_ST_NEGATIVE  3 /**< GYRO 负向自检 */
/**@}*/

/** @name 功耗模式控制 */
/**@{*/
#define CTRL6_C_XL_HM_MODE  (1<<4) /**< ACC 高性能模式 */
#define CTRL7_G_GY_HM_MODE  (1<<7) /**< GYRO 高性能模式 */
/**@}*/

/**
 * @brief FIFO 条目传感器类型
 */
typedef enum {
    LSM6DSR_FIFO_SENSOR_GYRO = 1, /**< GYRO 数据 */
    LSM6DSR_FIFO_SENSOR_ACC  = 2  /**< ACC 数据 */
} lsm6dsr_fifo_sensor_t;

/* ===================================================================
 * 函数原型
 * =================================================================== */

/** @name 寄存器 I/O */
/**@{*/
lsm6dsr_status_t lsm6dsr_write_reg(lsm6dsr_io_t *io, uint8_t reg, uint8_t val);
lsm6dsr_status_t lsm6dsr_read_reg(lsm6dsr_io_t *io, uint8_t reg, uint8_t *val);
lsm6dsr_status_t lsm6dsr_read_multi(lsm6dsr_io_t *io, uint8_t reg, uint8_t *data, uint16_t len);
lsm6dsr_status_t lsm6dsr_read_multi_bytewise(lsm6dsr_io_t *io, uint8_t reg, uint8_t *data, uint16_t len);
/**@}*/

/** @name 器件控制 */
/**@{*/
lsm6dsr_status_t lsm6dsr_verify_id(lsm6dsr_io_t *io);
lsm6dsr_status_t lsm6dsr_reset(lsm6dsr_io_t *io);
lsm6dsr_status_t lsm6dsr_boot(lsm6dsr_io_t *io);
lsm6dsr_status_t lsm6dsr_i3c_disable(lsm6dsr_io_t *io);
/**@}*/

/** @name ACC 数据 */
/**@{*/
lsm6dsr_status_t lsm6dsr_accel_config(lsm6dsr_io_t *io, lsm6dsr_accel_odr_t odr, lsm6dsr_accel_fs_t fs);
lsm6dsr_status_t lsm6dsr_read_accel_raw(lsm6dsr_io_t *io, lsm6dsr_axis_t *accel);
lsm6dsr_status_t lsm6dsr_read_accel_float(lsm6dsr_io_t *io, float *ax, float *ay, float *az, lsm6dsr_accel_fs_t fs);
/**@}*/

/** @name GYRO 数据 */
/**@{*/
lsm6dsr_status_t lsm6dsr_gyro_config(lsm6dsr_io_t *io, lsm6dsr_gyro_odr_t odr, lsm6dsr_gyro_fs_t fs);
lsm6dsr_status_t lsm6dsr_read_gyro_raw(lsm6dsr_io_t *io, lsm6dsr_axis_t *gyro);
lsm6dsr_status_t lsm6dsr_read_gyro_float(lsm6dsr_io_t *io, float *wx, float *wy, float *wz, lsm6dsr_gyro_fs_t fs);
/**@}*/

/** @name 温度 */
/**@{*/
lsm6dsr_status_t lsm6dsr_read_temp(lsm6dsr_io_t *io, float *temp_celsius);
/**@}*/

/** @name FIFO 操作 */
/**@{*/
lsm6dsr_status_t lsm6dsr_fifo_init(lsm6dsr_io_t *io, uint16_t threshold, uint8_t bdr_xl, uint8_t bdr_gy);
lsm6dsr_status_t lsm6dsr_fifo_set_mode(lsm6dsr_io_t *io, uint8_t mode);
lsm6dsr_status_t lsm6dsr_fifo_read_tag_data(lsm6dsr_io_t *io, uint8_t *tag, uint8_t *data);
uint16_t lsm6dsr_fifo_get_level(lsm6dsr_io_t *io);
uint8_t lsm6dsr_fifo_wtm_flag(lsm6dsr_io_t *io);
uint8_t lsm6dsr_fifo_ovr_flag(lsm6dsr_io_t *io);
uint8_t lsm6dsr_fifo_full_flag(lsm6dsr_io_t *io);
lsm6dsr_status_t lsm6dsr_fifo_flush(lsm6dsr_io_t *io);
lsm6dsr_status_t lsm6dsr_fifo_set_wtm(lsm6dsr_io_t *io, uint16_t threshold);
lsm6dsr_status_t lsm6dsr_read_fifo_entry(lsm6dsr_io_t *io,
                                          lsm6dsr_fifo_sensor_t *sensor,
                                          lsm6dsr_axis_t *data);
/**@}*/

/** @name 数据就绪 (DRDY) */
/**@{*/
lsm6dsr_status_t lsm6dsr_get_drdy(lsm6dsr_io_t *io, uint8_t *accel_drdy, uint8_t *gyro_drdy);
/**@}*/

/** @name BDU / IF_INC 控制 */
/**@{*/
lsm6dsr_status_t lsm6dsr_set_bdu(lsm6dsr_io_t *io, uint8_t enable);
lsm6dsr_status_t lsm6dsr_set_if_inc(lsm6dsr_io_t *io, uint8_t enable);
/**@}*/

/** @name 自检 */
/**@{*/
lsm6dsr_status_t lsm6dsr_xl_self_test(lsm6dsr_io_t *io, uint8_t mode);
lsm6dsr_status_t lsm6dsr_gy_self_test(lsm6dsr_io_t *io, uint8_t mode);
/**@}*/

/** @name 功耗模式 */
/**@{*/
lsm6dsr_status_t lsm6dsr_xl_set_hm_mode(lsm6dsr_io_t *io, uint8_t enable);
lsm6dsr_status_t lsm6dsr_gy_set_hm_mode(lsm6dsr_io_t *io, uint8_t enable);
/**@}*/

#ifdef __cplusplus
}
#endif

#endif
