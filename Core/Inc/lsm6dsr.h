#ifndef LSM6DSR_H
#define LSM6DSR_H

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LSM6DSR_I2C_ADDR         (0x6A << 1)
#define LSM6DSR_WHO_AM_I_VAL     0x6B

#define LSM6DSR_REG_FIFO_CTRL1      0x07
#define LSM6DSR_REG_FIFO_CTRL2      0x08
#define LSM6DSR_REG_FIFO_CTRL3      0x09
#define LSM6DSR_REG_FIFO_CTRL4      0x0A
#define LSM6DSR_REG_WHO_AM_I        0x0F
#define LSM6DSR_REG_CTRL1_XL        0x10
#define LSM6DSR_REG_CTRL2_G         0x11
#define LSM6DSR_REG_CTRL3_C         0x12
#define LSM6DSR_REG_CTRL4_C         0x13
#define LSM6DSR_REG_CTRL6_C         0x15
#define LSM6DSR_REG_CTRL7_G         0x16
#define LSM6DSR_REG_CTRL8_XL        0x17
#define LSM6DSR_REG_CTRL9_XL        0x18
#define LSM6DSR_REG_STATUS_REG      0x1E
#define LSM6DSR_REG_OUT_TEMP_L      0x20
#define LSM6DSR_REG_OUT_TEMP_H      0x21
#define LSM6DSR_REG_OUTX_L_G        0x22
#define LSM6DSR_REG_OUTX_H_G        0x23
#define LSM6DSR_REG_OUTY_L_G        0x24
#define LSM6DSR_REG_OUTY_H_G        0x25
#define LSM6DSR_REG_OUTZ_L_G        0x26
#define LSM6DSR_REG_OUTZ_H_G        0x27
#define LSM6DSR_REG_OUTX_L_XL       0x28
#define LSM6DSR_REG_OUTX_H_XL       0x29
#define LSM6DSR_REG_OUTY_L_XL       0x2A
#define LSM6DSR_REG_OUTY_H_XL       0x2B
#define LSM6DSR_REG_OUTZ_L_XL       0x2C
#define LSM6DSR_REG_OUTZ_H_XL       0x2D
#define LSM6DSR_REG_FIFO_STATUS1     0x3A
#define LSM6DSR_REG_FIFO_STATUS2     0x3B
#define LSM6DSR_REG_FIFO_DATA_OUT_TAG 0x78
#define LSM6DSR_REG_FIFO_DATA_OUT_XL  0x79
#define LSM6DSR_REG_INT1_CTRL       0x0D
#define LSM6DSR_REG_INT2_CTRL       0x0E

#define CTRL3_C_SW_RESET    (1<<0)
#define CTRL3_C_BDU         (1<<6)
#define CTRL3_C_IF_INC      (1<<2)
#define CTRL3_C_BOOT        (1<<7)

#define CTRL9_XL_I3C_DISABLE  (1<<1)

#define STATUS_REG_DRDY_XL  (1<<0)
#define STATUS_REG_DRDY_G   (1<<1)
#define STATUS_REG_DRDY_TEMP (1<<2)

#define FIFO_CTRL4_FIFO_MODE_MASK   0x07
#define FIFO_MODE_BYPASS            0x00
#define FIFO_MODE_FIFO              0x01
#define FIFO_MODE_CONT_TO_FIFO      0x03
#define FIFO_MODE_CONT              0x06

#define LSM6DSR_BDR_NOT_BATCHED  0x00
#define LSM6DSR_BDR_12Hz5        0x01
#define LSM6DSR_BDR_26Hz         0x02
#define LSM6DSR_BDR_52Hz         0x03
#define LSM6DSR_BDR_104Hz        0x04
#define LSM6DSR_BDR_208Hz        0x05
#define LSM6DSR_BDR_416Hz        0x06
#define LSM6DSR_BDR_833Hz        0x07

typedef enum {
    LSM6DSR_OK             = 0x00,
    LSM6DSR_ERROR          = 0x01,
    LSM6DSR_TIMEOUT        = 0x02,
    LSM6DSR_NOT_FOUND      = 0x03,
    LSM6DSR_INVALID_PARAM  = 0x04,
    LSM6DSR_NULL_PTR       = 0x05
} lsm6dsr_status_t;

typedef enum {
    LSM6DSR_ACCEL_ODR_OFF     = 0x00,
    LSM6DSR_ACCEL_ODR_12_5HZ  = 0x01,
    LSM6DSR_ACCEL_ODR_26HZ    = 0x02,
    LSM6DSR_ACCEL_ODR_52HZ    = 0x03,
    LSM6DSR_ACCEL_ODR_104HZ   = 0x04,
    LSM6DSR_ACCEL_ODR_208HZ   = 0x05,
    LSM6DSR_ACCEL_ODR_416HZ   = 0x06,
    LSM6DSR_ACCEL_ODR_833HZ   = 0x07,
    LSM6DSR_ACCEL_ODR_1_66KHZ = 0x08,
    LSM6DSR_ACCEL_ODR_3_33KHZ = 0x09,
    LSM6DSR_ACCEL_ODR_6_66KHZ = 0x0A
} lsm6dsr_accel_odr_t;

typedef enum {
    LSM6DSR_ACCEL_FS_2G  = 0x00,
    LSM6DSR_ACCEL_FS_4G  = 0x02,
    LSM6DSR_ACCEL_FS_8G  = 0x03,
    LSM6DSR_ACCEL_FS_16G = 0x01
} lsm6dsr_accel_fs_t;

typedef lsm6dsr_accel_odr_t lsm6dsr_gyro_odr_t;
#define LSM6DSR_GYRO_ODR_OFF     LSM6DSR_ACCEL_ODR_OFF
#define LSM6DSR_GYRO_ODR_12_5HZ LSM6DSR_ACCEL_ODR_12_5HZ
#define LSM6DSR_GYRO_ODR_26HZ   LSM6DSR_ACCEL_ODR_26HZ
#define LSM6DSR_GYRO_ODR_52HZ   LSM6DSR_ACCEL_ODR_52HZ
#define LSM6DSR_GYRO_ODR_104HZ  LSM6DSR_ACCEL_ODR_104HZ
#define LSM6DSR_GYRO_ODR_208HZ  LSM6DSR_ACCEL_ODR_208HZ

typedef enum {
    LSM6DSR_GYRO_FS_250DPS  = 0,
    LSM6DSR_GYRO_FS_500DPS  = 4,
    LSM6DSR_GYRO_FS_1000DPS = 8,
    LSM6DSR_GYRO_FS_2000DPS = 12
} lsm6dsr_gyro_fs_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lsm6dsr_axis_t;

#define LSM6DSR_ACCEL_SENS_2G   0.061f
#define LSM6DSR_ACCEL_SENS_4G   0.122f
#define LSM6DSR_ACCEL_SENS_8G   0.244f
#define LSM6DSR_ACCEL_SENS_16G  0.488f

#define LSM6DSR_GYRO_SENS_250DPS   0.00875f
#define LSM6DSR_GYRO_SENS_500DPS  0.01750f
#define LSM6DSR_GYRO_SENS_1000DPS 0.03500f
#define LSM6DSR_GYRO_SENS_2000DPS 0.07000f

#define LSM6DSR_TEMP_SENSITIVITY   256.0f
#define LSM6DSR_TEMP_OFFSET        25.0f

#define FIFO_TAG_SENSOR(tag)     ((tag) & 0x1F)
#define FIFO_TAG_CNT(tag)        (((tag) >> 5) & 0x03)
#define FIFO_TAG_PARITY(tag)     (((tag) >> 7) & 0x01)

/* Note: LSM6DSR FIFO tag byte encoding differs from ST documentation's
 * tag_sensor[4:0] format. Empirical testing shows bit4=0 means GYRO,
 * bit4=1 means ACC. Bits[3:0] encode compression/batch metadata. */
#define FIFO_TAG_IS_GYRO(tag)   (!((tag) & 0x10))
#define FIFO_TAG_IS_ACC(tag)    (((tag) & 0x10) != 0)

#define FIFO_TAG_GYRO  0
#define FIFO_TAG_ACC   1

lsm6dsr_status_t lsm6dsr_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);
lsm6dsr_status_t lsm6dsr_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val);
lsm6dsr_status_t lsm6dsr_read_multi(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len);
lsm6dsr_status_t lsm6dsr_read_multi_bytewise(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len);

lsm6dsr_status_t lsm6dsr_verify_id(I2C_HandleTypeDef *hi2c);
lsm6dsr_status_t lsm6dsr_reset(I2C_HandleTypeDef *hi2c);
lsm6dsr_status_t lsm6dsr_boot(I2C_HandleTypeDef *hi2c);
lsm6dsr_status_t lsm6dsr_i3c_disable(I2C_HandleTypeDef *hi2c);

lsm6dsr_status_t lsm6dsr_accel_config(I2C_HandleTypeDef *hi2c, lsm6dsr_accel_odr_t odr, lsm6dsr_accel_fs_t fs);
lsm6dsr_status_t lsm6dsr_read_accel_raw(I2C_HandleTypeDef *hi2c, lsm6dsr_axis_t *accel);
lsm6dsr_status_t lsm6dsr_read_accel_float(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az, lsm6dsr_accel_fs_t fs);

lsm6dsr_status_t lsm6dsr_gyro_config(I2C_HandleTypeDef *hi2c, lsm6dsr_gyro_odr_t odr, lsm6dsr_gyro_fs_t fs);
lsm6dsr_status_t lsm6dsr_read_gyro_raw(I2C_HandleTypeDef *hi2c, lsm6dsr_axis_t *gyro);
lsm6dsr_status_t lsm6dsr_read_gyro_float(I2C_HandleTypeDef *hi2c, float *wx, float *wy, float *wz, lsm6dsr_gyro_fs_t fs);

lsm6dsr_status_t lsm6dsr_read_temp(I2C_HandleTypeDef *hi2c, float *temp_celsius);

lsm6dsr_status_t lsm6dsr_fifo_init(I2C_HandleTypeDef *hi2c, uint16_t threshold, uint8_t bdr_xl, uint8_t bdr_gy);
lsm6dsr_status_t lsm6dsr_fifo_set_mode(I2C_HandleTypeDef *hi2c, uint8_t mode);
lsm6dsr_status_t lsm6dsr_fifo_read_tag_data(I2C_HandleTypeDef *hi2c, uint8_t *tag, uint8_t *data);
uint16_t lsm6dsr_fifo_get_level(I2C_HandleTypeDef *hi2c);

lsm6dsr_status_t lsm6dsr_get_drdy(I2C_HandleTypeDef *hi2c, uint8_t *accel_drdy, uint8_t *gyro_drdy);

/* FIFO status flags */
uint8_t lsm6dsr_fifo_wtm_flag(I2C_HandleTypeDef *hi2c);
uint8_t lsm6dsr_fifo_ovr_flag(I2C_HandleTypeDef *hi2c);
uint8_t lsm6dsr_fifo_full_flag(I2C_HandleTypeDef *hi2c);
lsm6dsr_status_t lsm6dsr_fifo_flush(I2C_HandleTypeDef *hi2c);
lsm6dsr_status_t lsm6dsr_fifo_set_wtm(I2C_HandleTypeDef *hi2c, uint16_t threshold);

/* Independent BDU / IF_INC control */
lsm6dsr_status_t lsm6dsr_set_bdu(I2C_HandleTypeDef *hi2c, uint8_t enable);
lsm6dsr_status_t lsm6dsr_set_if_inc(I2C_HandleTypeDef *hi2c, uint8_t enable);

/* FIFO entry read with sensor type */
typedef enum {
    LSM6DSR_FIFO_SENSOR_GYRO = 1,
    LSM6DSR_FIFO_SENSOR_ACC  = 2
} lsm6dsr_fifo_sensor_t;
lsm6dsr_status_t lsm6dsr_read_fifo_entry(I2C_HandleTypeDef *hi2c,
                                          lsm6dsr_fifo_sensor_t *sensor,
                                          lsm6dsr_axis_t *data);

#ifdef __cplusplus
}
#endif

#endif
