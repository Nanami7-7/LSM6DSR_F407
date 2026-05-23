#include "lsm6dsr.h"

static uint32_t i2c_timeout = 100;

lsm6dsr_status_t lsm6dsr_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    if (hi2c == NULL) return LSM6DSR_NULL_PTR;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(hi2c, LSM6DSR_I2C_ADDR, reg,
                                              I2C_MEMADD_SIZE_8BIT, &val, 1, i2c_timeout);
    return (ret == HAL_OK) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

lsm6dsr_status_t lsm6dsr_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
    if (hi2c == NULL || val == NULL) return LSM6DSR_NULL_PTR;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, LSM6DSR_I2C_ADDR, reg,
                                             I2C_MEMADD_SIZE_8BIT, val, 1, i2c_timeout);
    return (ret == HAL_OK) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

lsm6dsr_status_t lsm6dsr_read_multi(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (hi2c == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, LSM6DSR_I2C_ADDR, reg,
                                             I2C_MEMADD_SIZE_8BIT, data, len, i2c_timeout);
    return (ret == HAL_OK) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

lsm6dsr_status_t lsm6dsr_read_multi_bytewise(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (hi2c == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    for (uint16_t i = 0; i < len; i++) {
        HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hi2c, LSM6DSR_I2C_ADDR,
                                                 reg + i, I2C_MEMADD_SIZE_8BIT,
                                                 &data[i], 1, i2c_timeout);
        if (ret != HAL_OK) return LSM6DSR_ERROR;
    }
    return LSM6DSR_OK;
}

lsm6dsr_status_t lsm6dsr_verify_id(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_WHO_AM_I, &id);
    if (st != LSM6DSR_OK) return st;
    return (id == LSM6DSR_WHO_AM_I_VAL) ? LSM6DSR_OK : LSM6DSR_NOT_FOUND;
}

lsm6dsr_status_t lsm6dsr_reset(I2C_HandleTypeDef *hi2c)
{
    lsm6dsr_status_t st = lsm6dsr_write_reg(hi2c, LSM6DSR_REG_CTRL3_C, CTRL3_C_SW_RESET);
    if (st != LSM6DSR_OK) return st;
    uint8_t rst = 1;
    uint32_t timeout = 1000;
    while (rst != 0 && timeout-- > 0) {
        HAL_Delay(1);
        lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL3_C, &rst);
        rst &= CTRL3_C_SW_RESET;
    }
    return (rst == 0) ? LSM6DSR_OK : LSM6DSR_TIMEOUT;
}

lsm6dsr_status_t lsm6dsr_boot(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL3_C, &val);
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_write_reg(hi2c, LSM6DSR_REG_CTRL3_C, val | CTRL3_C_BOOT);
    if (st != LSM6DSR_OK) return st;
    uint8_t boot = 1;
    uint32_t timeout = 500;
    while (boot != 0 && timeout-- > 0) {
        HAL_Delay(1);
        lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL3_C, &boot);
        boot &= CTRL3_C_BOOT;
    }
    return (boot == 0) ? LSM6DSR_OK : LSM6DSR_TIMEOUT;
}

lsm6dsr_status_t lsm6dsr_i3c_disable(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL9_XL, &val);
    if (st != LSM6DSR_OK) return st;
    val |= CTRL9_XL_I3C_DISABLE;
    return lsm6dsr_write_reg(hi2c, LSM6DSR_REG_CTRL9_XL, val);
}

static float accel_sensitivity(lsm6dsr_accel_fs_t fs)
{
    switch (fs) {
        case LSM6DSR_ACCEL_FS_2G:  return LSM6DSR_ACCEL_SENS_2G;
        case LSM6DSR_ACCEL_FS_4G:  return LSM6DSR_ACCEL_SENS_4G;
        case LSM6DSR_ACCEL_FS_8G:  return LSM6DSR_ACCEL_SENS_8G;
        case LSM6DSR_ACCEL_FS_16G: return LSM6DSR_ACCEL_SENS_16G;
        default:                   return LSM6DSR_ACCEL_SENS_2G;
    }
}

static float gyro_sensitivity(lsm6dsr_gyro_fs_t fs)
{
    switch (fs) {
        case LSM6DSR_GYRO_FS_250DPS:  return LSM6DSR_GYRO_SENS_250DPS;
        case LSM6DSR_GYRO_FS_500DPS:  return LSM6DSR_GYRO_SENS_500DPS;
        case LSM6DSR_GYRO_FS_1000DPS: return LSM6DSR_GYRO_SENS_1000DPS;
        case LSM6DSR_GYRO_FS_2000DPS: return LSM6DSR_GYRO_SENS_2000DPS;
        default:                      return LSM6DSR_GYRO_SENS_250DPS;
    }
}

lsm6dsr_status_t lsm6dsr_accel_config(I2C_HandleTypeDef *hi2c, lsm6dsr_accel_odr_t odr, lsm6dsr_accel_fs_t fs)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL1_XL, &val);
    if (st != LSM6DSR_OK) return st;
    val = (val & 0x03) | ((uint8_t)fs << 2) | ((uint8_t)odr << 4);
    return lsm6dsr_write_reg(hi2c, LSM6DSR_REG_CTRL1_XL, val);
}

lsm6dsr_status_t lsm6dsr_read_accel_raw(I2C_HandleTypeDef *hi2c, lsm6dsr_axis_t *accel)
{
    if (accel == NULL) return LSM6DSR_NULL_PTR;
    uint8_t buf[6];
    lsm6dsr_status_t st = lsm6dsr_read_multi(hi2c, LSM6DSR_REG_OUTX_L_XL, buf, 6);
    if (st != LSM6DSR_OK) return st;
    accel->x = (int16_t)(buf[1] << 8 | buf[0]);
    accel->y = (int16_t)(buf[3] << 8 | buf[2]);
    accel->z = (int16_t)(buf[5] << 8 | buf[4]);
    return LSM6DSR_OK;
}

lsm6dsr_status_t lsm6dsr_read_accel_float(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az, lsm6dsr_accel_fs_t fs)
{
    if (ax == NULL || ay == NULL || az == NULL) return LSM6DSR_NULL_PTR;
    lsm6dsr_axis_t raw;
    lsm6dsr_status_t st = lsm6dsr_read_accel_raw(hi2c, &raw);
    if (st != LSM6DSR_OK) return st;
    float sens = accel_sensitivity(fs);
    *ax = raw.x * sens;
    *ay = raw.y * sens;
    *az = raw.z * sens;
    return LSM6DSR_OK;
}

lsm6dsr_status_t lsm6dsr_gyro_config(I2C_HandleTypeDef *hi2c, lsm6dsr_gyro_odr_t odr, lsm6dsr_gyro_fs_t fs)
{
    uint8_t val = ((uint8_t)fs & 0x0F) | ((uint8_t)odr << 4);
    return lsm6dsr_write_reg(hi2c, LSM6DSR_REG_CTRL2_G, val);
}

lsm6dsr_status_t lsm6dsr_read_gyro_raw(I2C_HandleTypeDef *hi2c, lsm6dsr_axis_t *gyro)
{
    if (gyro == NULL) return LSM6DSR_NULL_PTR;
    uint8_t buf[6];
    lsm6dsr_status_t st = lsm6dsr_read_multi(hi2c, LSM6DSR_REG_OUTX_L_G , buf, 6);
    if (st != LSM6DSR_OK) return st;
    gyro->x = (int16_t)(buf[1] << 8 | buf[0]);
    gyro->y = (int16_t)(buf[3] << 8 | buf[2]);
    gyro->z = (int16_t)(buf[5] << 8 | buf[4]);
    return LSM6DSR_OK;
}

lsm6dsr_status_t lsm6dsr_read_gyro_float(I2C_HandleTypeDef *hi2c, float *wx, float *wy, float *wz, lsm6dsr_gyro_fs_t fs)
{
    if (wx == NULL || wy == NULL || wz == NULL) return LSM6DSR_NULL_PTR;
    lsm6dsr_axis_t raw;
    lsm6dsr_status_t st = lsm6dsr_read_gyro_raw(hi2c, &raw);
    if (st != LSM6DSR_OK) return st;
    float sens = gyro_sensitivity(fs);
    *wx = raw.x * sens;
    *wy = raw.y * sens;
    *wz = raw.z * sens;
    return LSM6DSR_OK;
}

lsm6dsr_status_t lsm6dsr_read_temp(I2C_HandleTypeDef *hi2c, float *temp_celsius)
{
    if (temp_celsius == NULL) return LSM6DSR_NULL_PTR;
    uint8_t buf[2];
    lsm6dsr_status_t st = lsm6dsr_read_multi(hi2c, LSM6DSR_REG_OUT_TEMP_L , buf, 2);
    if (st != LSM6DSR_OK) return st;
    int16_t raw = (int16_t)(buf[1] << 8 | buf[0]);
    *temp_celsius = (raw / LSM6DSR_TEMP_SENSITIVITY) + LSM6DSR_TEMP_OFFSET;
    return LSM6DSR_OK;
}

lsm6dsr_status_t lsm6dsr_fifo_init(I2C_HandleTypeDef *hi2c, uint16_t threshold, uint8_t bdr_xl, uint8_t bdr_gy)
{
    lsm6dsr_status_t st;
    st = lsm6dsr_write_reg(hi2c, LSM6DSR_REG_FIFO_CTRL1, (uint8_t)(threshold & 0xFF));
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_write_reg(hi2c, LSM6DSR_REG_FIFO_CTRL2,
                            (uint8_t)(((threshold >> 8) & 0x01) | (0x03 << 1)));
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_write_reg(hi2c, LSM6DSR_REG_FIFO_CTRL3, (bdr_gy << 4) | bdr_xl);
    if (st != LSM6DSR_OK) return st;
    return LSM6DSR_OK;
}

lsm6dsr_status_t lsm6dsr_fifo_set_mode(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
    uint8_t val;
    lsm6dsr_status_t st;

    st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_CTRL4, &val);
    if (st != LSM6DSR_OK) return st;
    val = (val & ~FIFO_CTRL4_FIFO_MODE_MASK) | (mode & FIFO_CTRL4_FIFO_MODE_MASK);
    st = lsm6dsr_write_reg(hi2c, LSM6DSR_REG_FIFO_CTRL4, val);
    if (st != LSM6DSR_OK) return st;

    st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_CTRL2, &val);
    if (st != LSM6DSR_OK) return st;
    val &= ~((1 << 7) | (1 << 6));
    return lsm6dsr_write_reg(hi2c, LSM6DSR_REG_FIFO_CTRL2, val);
}

lsm6dsr_status_t lsm6dsr_fifo_read_tag_data(I2C_HandleTypeDef *hi2c, uint8_t *tag, uint8_t *data)
{
    if (tag == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_DATA_OUT_TAG, tag);
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_read_multi(hi2c, LSM6DSR_REG_FIFO_DATA_OUT_XL, data, 6);
    return st;
}

uint16_t lsm6dsr_fifo_get_level(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2];
    if (lsm6dsr_read_multi(hi2c, LSM6DSR_REG_FIFO_STATUS1 , buf, 2) != LSM6DSR_OK)
        return 0;
    return (uint16_t)((buf[1] & 0x03) << 8 | buf[0]) & 0x03FF;
}

lsm6dsr_status_t lsm6dsr_get_drdy(I2C_HandleTypeDef *hi2c, uint8_t *accel_drdy, uint8_t *gyro_drdy)
{
    if (accel_drdy == NULL || gyro_drdy == NULL) return LSM6DSR_NULL_PTR;
    uint8_t status;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_STATUS_REG, &status);
    if (st != LSM6DSR_OK) return st;
    *accel_drdy = (status & STATUS_REG_DRDY_XL) ? 1 : 0;
    *gyro_drdy  = (status & STATUS_REG_DRDY_G)  ? 1 : 0;
    return LSM6DSR_OK;
}

/*============================================================================*
 * FIFO Status Flags
 *============================================================================*/
uint8_t lsm6dsr_fifo_wtm_flag(I2C_HandleTypeDef *hi2c)
{
    uint8_t status = 0;
    lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_STATUS2, &status);
    return (status >> 7) & 1;
}

uint8_t lsm6dsr_fifo_ovr_flag(I2C_HandleTypeDef *hi2c)
{
    uint8_t status = 0;
    lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_STATUS2, &status);
    return (status >> 6) & 1;
}

uint8_t lsm6dsr_fifo_full_flag(I2C_HandleTypeDef *hi2c)
{
    uint8_t status = 0;
    lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_STATUS2, &status);
    return (status >> 5) & 1;
}

lsm6dsr_status_t lsm6dsr_fifo_flush(I2C_HandleTypeDef *hi2c)
{
    return lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_BYPASS);
}

lsm6dsr_status_t lsm6dsr_fifo_set_wtm(I2C_HandleTypeDef *hi2c, uint16_t threshold)
{
    lsm6dsr_status_t st;
    st = lsm6dsr_write_reg(hi2c, LSM6DSR_REG_FIFO_CTRL1, (uint8_t)(threshold & 0xFF));
    if (st != LSM6DSR_OK) return st;
    uint8_t ctrl2;
    st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_CTRL2, &ctrl2);
    if (st != LSM6DSR_OK) return st;
    ctrl2 = (ctrl2 & 0xFE) | ((threshold >> 8) & 0x01);
    return lsm6dsr_write_reg(hi2c, LSM6DSR_REG_FIFO_CTRL2, ctrl2);
}

/*============================================================================*
 * BDU / IF_INC Control
 *============================================================================*/
lsm6dsr_status_t lsm6dsr_set_bdu(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL3_C, &val);
    if (st != LSM6DSR_OK) return st;
    if (enable) val |= CTRL3_C_BDU;
    else        val &= ~CTRL3_C_BDU;
    return lsm6dsr_write_reg(hi2c, LSM6DSR_REG_CTRL3_C, val);
}

lsm6dsr_status_t lsm6dsr_set_if_inc(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL3_C, &val);
    if (st != LSM6DSR_OK) return st;
    if (enable) val |= CTRL3_C_IF_INC;
    else        val &= ~CTRL3_C_IF_INC;
    return lsm6dsr_write_reg(hi2c, LSM6DSR_REG_CTRL3_C, val);
}

/*============================================================================*
 * FIFO Entry Read with Sensor Type
 *============================================================================*/
lsm6dsr_status_t lsm6dsr_read_fifo_entry(I2C_HandleTypeDef *hi2c,
                                          lsm6dsr_fifo_sensor_t *sensor,
                                          lsm6dsr_axis_t *data)
{
    if (sensor == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    uint8_t tag, buf[6];
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_FIFO_DATA_OUT_TAG, &tag);
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_read_multi(hi2c, LSM6DSR_REG_FIFO_DATA_OUT_XL, buf, 6);
    if (st != LSM6DSR_OK) return st;
    data->x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    data->y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    data->z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    *sensor = FIFO_TAG_IS_ACC(tag) ? LSM6DSR_FIFO_SENSOR_ACC : LSM6DSR_FIFO_SENSOR_GYRO;
    return LSM6DSR_OK;
}
