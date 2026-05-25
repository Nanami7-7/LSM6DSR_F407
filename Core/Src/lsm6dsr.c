/**
 * @file    lsm6dsr.c
 * @brief   LSM6DSR 驱动层实现
 *
 * 提供 lsm6dsr.h 中声明的所有函数的具体实现。
 * 所有 I/O 通过 lsm6dsr_io_t 回调函数完成，平台无关。
 *
 * 功能覆盖：
 *   - 寄存器单字节/多字节读写
 *   - ACC/GYRO/TEMP 数据读取 (raw + float)
 *   - FIFO 初始化、模式设置、状态查询、数据读取
 *   - 自检、功耗模式、BDU/IF_INC 控制
 */
#include "lsm6dsr.h"

/**
 * @brief  写单字节寄存器
 * @param  io  I/O 抽象层指针
 * @param  reg 寄存器地址
 * @param  val 写入值
 * @return LSM6DSR_OK 成功 / LSM6DSR_NULL_PTR 空指针 / LSM6DSR_ERROR 通信失败
 */
lsm6dsr_status_t lsm6dsr_write_reg(lsm6dsr_io_t *io, uint8_t reg, uint8_t val)
{
    if (io == NULL || io->write == NULL) return LSM6DSR_NULL_PTR;
    return (io->write(io->ctx, reg, &val, 1) == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

/**
 * @brief  读单字节寄存器
 * @param  io  I/O 抽象层指针
 * @param  reg 寄存器地址
 * @param  val 输出缓冲区
 * @return LSM6DSR_OK 成功 / LSM6DSR_NULL_PTR 空指针 / LSM6DSR_ERROR 通信失败
 */
lsm6dsr_status_t lsm6dsr_read_reg(lsm6dsr_io_t *io, uint8_t reg, uint8_t *val)
{
    if (io == NULL || io->read == NULL || val == NULL) return LSM6DSR_NULL_PTR;
    return (io->read(io->ctx, reg, val, 1) == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

/**
 * @brief  多字节读 (需 IF_INC 自动递增地址)
 * @param  io   I/O 抽象层指针
 * @param  reg  起始寄存器地址
 * @param  data 输出缓冲区
 * @param  len  读取字节数
 * @return LSM6DSR_OK 成功 / LSM6DSR_NULL_PTR 空指针 / LSM6DSR_ERROR 通信失败
 */
lsm6dsr_status_t lsm6dsr_read_multi(lsm6dsr_io_t *io, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (io == NULL || io->read == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    return (io->read(io->ctx, reg, data, len) == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

/**
 * @brief  逐字节多读 (无需 IF_INC，但效率低)
 * @details 某些平台不支持多字节连续读取时使用此函数。
 *          每字节地址递增 reg+i，适用于寄存器不自增的情况。
 * @param  io   I/O 抽象层指针
 * @param  reg  起始寄存器地址
 * @param  data 输出缓冲区
 * @param  len  读取字节数
 * @return LSM6DSR_OK 成功 / 其他 失败
 */
lsm6dsr_status_t lsm6dsr_read_multi_bytewise(lsm6dsr_io_t *io, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (io == NULL || io->read == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    for (uint16_t i = 0; i < len; i++) {
        if (io->read(io->ctx, reg + i, &data[i], 1) != 0)
            return LSM6DSR_ERROR;
    }
    return LSM6DSR_OK;
}

/**
 * @brief  验证 WHO_AM_I
 * @param  io I/O 抽象层指针
 * @return LSM6DSR_OK 匹配 / LSM6DSR_NOT_FOUND 不匹配 / 其他 通信失败
 */
lsm6dsr_status_t lsm6dsr_verify_id(lsm6dsr_io_t *io)
{
    uint8_t id = 0;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_WHO_AM_I, &id);
    if (st != LSM6DSR_OK) return st;
    return (id == LSM6DSR_WHO_AM_I_VAL) ? LSM6DSR_OK : LSM6DSR_NOT_FOUND;
}

/**
 * @brief  软件复位 (SW_RESET)
 * @details 写 CTRL3_C bit0=1，轮询等待硬件自动清零
 * @param  io I/O 抽象层指针
 * @return LSM6DSR_OK 成功 / LSM6DSR_TIMEOUT 超时 / 其他 通信失败
 */
lsm6dsr_status_t lsm6dsr_reset(lsm6dsr_io_t *io)
{
    lsm6dsr_status_t st = lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL3_C, CTRL3_C_SW_RESET);
    if (st != LSM6DSR_OK) return st;
    uint8_t rst = 1;
    uint32_t timeout = 1000;
    while (rst != 0 && timeout-- > 0) {
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL3_C, &rst);
        rst &= CTRL3_C_SW_RESET;
    }
    return (rst == 0) ? LSM6DSR_OK : LSM6DSR_TIMEOUT;
}

/**
 * @brief  重新加载 BOOT (校准参数)
 * @param  io I/O 抽象层指针
 * @return LSM6DSR_OK 成功 / LSM6DSR_TIMEOUT 超时 / 其他 通信失败
 */
lsm6dsr_status_t lsm6dsr_boot(lsm6dsr_io_t *io)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL3_C, &val);
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL3_C, val | CTRL3_C_BOOT);
    if (st != LSM6DSR_OK) return st;
    uint8_t boot = 1;
    uint32_t timeout = 500;
    while (boot != 0 && timeout-- > 0) {
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL3_C, &boot);
        boot &= CTRL3_C_BOOT;
    }
    return (boot == 0) ? LSM6DSR_OK : LSM6DSR_TIMEOUT;
}

/**
 * @brief  禁用 I3C 接口 (强制使用 I2C/SPI)
 * @param  io I/O 抽象层指针
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_i3c_disable(lsm6dsr_io_t *io)
{
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL9_XL, 0xE2);
}

/**
 * @brief  获取 ACC 灵敏度系数 (mg/LSB)
 * @param  fs 满量程选择
 * @return 灵敏度值 (mg/LSB)
 */
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

/**
 * @brief  获取 GYRO 灵敏度系数 (dps/LSB)
 * @param  fs 满量程选择
 * @return 灵敏度值 (dps/LSB)
 */
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

/**
 * @brief  配置 ACC (ODR + 满量程)
 * @param  io  I/O 抽象层指针
 * @param  odr 输出数据速率
 * @param  fs  满量程
 * @return lsm6dsr_status_t
 * @note  保留 CTRL1_XL 的低 2 位 (滤波配置不变)
 */
lsm6dsr_status_t lsm6dsr_accel_config(lsm6dsr_io_t *io, lsm6dsr_accel_odr_t odr, lsm6dsr_accel_fs_t fs)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL1_XL, &val);
    if (st != LSM6DSR_OK) return st;
    val = (val & 0x03) | ((uint8_t)fs << 2) | ((uint8_t)odr << 4);
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL1_XL, val);
}

/**
 * @brief  读取 ACC 原始数据 (int16)
 * @details 从 0x28 连续读取 6 字节 X/Y/Z (LSB 在先)
 * @param  io    I/O 抽象层指针
 * @param  accel 输出三轴 raw 数据
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_read_accel_raw(lsm6dsr_io_t *io, lsm6dsr_axis_t *accel)
{
    if (accel == NULL) return LSM6DSR_NULL_PTR;
    uint8_t buf[6];
    lsm6dsr_status_t st = lsm6dsr_read_multi(io, LSM6DSR_REG_OUTX_L_XL, buf, 6);
    if (st != LSM6DSR_OK) return st;
    accel->x = (int16_t)(buf[1] << 8 | buf[0]);
    accel->y = (int16_t)(buf[3] << 8 | buf[2]);
    accel->z = (int16_t)(buf[5] << 8 | buf[4]);
    return LSM6DSR_OK;
}

/**
 * @brief  读取 ACC 数据并转换为重力加速度 (g)
 * @param  io I/O 抽象层指针
 * @param  ax X 轴输出 (g)
 * @param  ay Y 轴输出 (g)
 * @param  az Z 轴输出 (g)
 * @param  fs 满量程
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_read_accel_float(lsm6dsr_io_t *io, float *ax, float *ay, float *az, lsm6dsr_accel_fs_t fs)
{
    if (ax == NULL || ay == NULL || az == NULL) return LSM6DSR_NULL_PTR;
    lsm6dsr_axis_t raw;
    lsm6dsr_status_t st = lsm6dsr_read_accel_raw(io, &raw);
    if (st != LSM6DSR_OK) return st;
    float sens = accel_sensitivity(fs);
    *ax = raw.x * sens;
    *ay = raw.y * sens;
    *az = raw.z * sens;
    return LSM6DSR_OK;
}

/**
 * @brief  配置 GYRO (ODR + 满量程)
 * @param  io  I/O 抽象层指针
 * @param  odr 输出数据速率
 * @param  fs  满量程
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_gyro_config(lsm6dsr_io_t *io, lsm6dsr_gyro_odr_t odr, lsm6dsr_gyro_fs_t fs)
{
    uint8_t val = ((uint8_t)fs & 0x0F) | ((uint8_t)odr << 4);
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL2_G, val);
}

/**
 * @brief  读取 GYRO 原始数据 (int16)
 * @details 从 0x22 连续读取 6 字节 X/Y/Z (LSB 在先)
 * @param  io   I/O 抽象层指针
 * @param  gyro 输出三轴 raw 数据
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_read_gyro_raw(lsm6dsr_io_t *io, lsm6dsr_axis_t *gyro)
{
    if (gyro == NULL) return LSM6DSR_NULL_PTR;
    uint8_t buf[6];
    lsm6dsr_status_t st = lsm6dsr_read_multi(io, LSM6DSR_REG_OUTX_L_G , buf, 6);
    if (st != LSM6DSR_OK) return st;
    gyro->x = (int16_t)(buf[1] << 8 | buf[0]);
    gyro->y = (int16_t)(buf[3] << 8 | buf[2]);
    gyro->z = (int16_t)(buf[5] << 8 | buf[4]);
    return LSM6DSR_OK;
}

/**
 * @brief  读取 GYRO 数据并转换为角速度 (dps)
 * @param  io I/O 抽象层指针
 * @param  wx X 轴输出 (dps)
 * @param  wy Y 轴输出 (dps)
 * @param  wz Z 轴输出 (dps)
 * @param  fs 满量程
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_read_gyro_float(lsm6dsr_io_t *io, float *wx, float *wy, float *wz, lsm6dsr_gyro_fs_t fs)
{
    if (wx == NULL || wy == NULL || wz == NULL) return LSM6DSR_NULL_PTR;
    lsm6dsr_axis_t raw;
    lsm6dsr_status_t st = lsm6dsr_read_gyro_raw(io, &raw);
    if (st != LSM6DSR_OK) return st;
    float sens = gyro_sensitivity(fs);
    *wx = raw.x * sens;
    *wy = raw.y * sens;
    *wz = raw.z * sens;
    return LSM6DSR_OK;
}

/**
 * @brief  读取温度传感器
 * @details 温度公式: T = raw/256 + 25 (°C)，25°C 时输出 0
 * @param  io           I/O 抽象层指针
 * @param  temp_celsius 输出温度 (°C)
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_read_temp(lsm6dsr_io_t *io, float *temp_celsius)
{
    if (temp_celsius == NULL) return LSM6DSR_NULL_PTR;
    uint8_t buf[2];
    lsm6dsr_status_t st = lsm6dsr_read_multi(io, LSM6DSR_REG_OUT_TEMP_L , buf, 2);
    if (st != LSM6DSR_OK) return st;
    int16_t raw = (int16_t)(buf[1] << 8 | buf[0]);
    *temp_celsius = (raw / LSM6DSR_TEMP_SENSITIVITY) + LSM6DSR_TEMP_OFFSET;
    return LSM6DSR_OK;
}

/**
 * @brief  初始化 FIFO (设置水印阈值 + 批速率)
 * @param  io        I/O 抽象层指针
 * @param  threshold 水印阈值 (9-bit)
 * @param  bdr_xl    ACC 批速率
 * @param  bdr_gy    GYRO 批速率
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_fifo_init(lsm6dsr_io_t *io, uint16_t threshold, uint8_t bdr_xl, uint8_t bdr_gy)
{
    lsm6dsr_status_t st;
    st = lsm6dsr_write_reg(io, LSM6DSR_REG_FIFO_CTRL1, (uint8_t)(threshold & 0xFF));
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_write_reg(io, LSM6DSR_REG_FIFO_CTRL2,
                            (uint8_t)(((threshold >> 8) & 0x01) | (0x03 << 1)));
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_write_reg(io, LSM6DSR_REG_FIFO_CTRL3, (bdr_gy << 4) | bdr_xl);
    if (st != LSM6DSR_OK) return st;
    return LSM6DSR_OK;
}

/**
 * @brief  设置 FIFO 模式
 * @param  io   I/O 抽象层指针
 * @param  mode FIFO 模式 (FIFO_MODE_*)
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_fifo_set_mode(lsm6dsr_io_t *io, uint8_t mode)
{
    uint8_t val;
    lsm6dsr_status_t st;

    st = lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_CTRL4, &val);
    if (st != LSM6DSR_OK) return st;
    val = (val & ~FIFO_CTRL4_FIFO_MODE_MASK) | (mode & FIFO_CTRL4_FIFO_MODE_MASK);
    st = lsm6dsr_write_reg(io, LSM6DSR_REG_FIFO_CTRL4, val);
    if (st != LSM6DSR_OK) return st;

    st = lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_CTRL2, &val);
    if (st != LSM6DSR_OK) return st;
    val &= ~((1 << 7) | (1 << 6));
    return lsm6dsr_write_reg(io, LSM6DSR_REG_FIFO_CTRL2, val);
}

/**
 * @brief  读取 FIFO 标签 + 数据 (6 字节)
 * @param  io   I/O 抽象层指针
 * @param  tag  输出标签
 * @param  data 输出数据缓冲区 (6 字节 X/Y/Z)
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_fifo_read_tag_data(lsm6dsr_io_t *io, uint8_t *tag, uint8_t *data)
{
    if (tag == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_DATA_OUT_TAG, tag);
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_read_multi(io, LSM6DSR_REG_FIFO_DATA_OUT_XL, data, 6);
    return st;
}

/**
 * @brief  获取 FIFO 已用深度
 * @param  io I/O 抽象层指针
 * @return FIFO 条目数 (0-1023)，通信失败时返回 0
 */
uint16_t lsm6dsr_fifo_get_level(lsm6dsr_io_t *io)
{
    uint8_t buf[2];
    if (lsm6dsr_read_multi(io, LSM6DSR_REG_FIFO_STATUS1 , buf, 2) != LSM6DSR_OK)
        return 0;
    return (uint16_t)((buf[1] & 0x03) << 8 | buf[0]) & 0x03FF;
}

/**
 * @brief  查询数据就绪状态
 * @param  io         I/O 抽象层指针
 * @param  accel_drdy ACC 就绪标志 (1=就绪)
 * @param  gyro_drdy  GYRO 就绪标志 (1=就绪)
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_get_drdy(lsm6dsr_io_t *io, uint8_t *accel_drdy, uint8_t *gyro_drdy)
{
    if (accel_drdy == NULL || gyro_drdy == NULL) return LSM6DSR_NULL_PTR;
    uint8_t status;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_STATUS_REG, &status);
    if (st != LSM6DSR_OK) return st;
    *accel_drdy = (status & STATUS_REG_DRDY_XL) ? 1 : 0;
    *gyro_drdy  = (status & STATUS_REG_DRDY_G)  ? 1 : 0;
    return LSM6DSR_OK;
}

/*============================================================================*
 * FIFO Status Flags
 *============================================================================*/
/**
 * @brief  查询 FIFO 水印标志
 * @param  io I/O 抽象层指针
 * @return 1=达到水印阈值 / 0=未达到
 */
uint8_t lsm6dsr_fifo_wtm_flag(lsm6dsr_io_t *io)
{
    uint8_t status = 0;
    lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_STATUS2, &status);
    return (status >> 7) & 1;
}

/**
 * @brief  查询 FIFO 溢出标志
 * @param  io I/O 抽象层指针
 * @return 1=溢出 / 0=正常
 */
uint8_t lsm6dsr_fifo_ovr_flag(lsm6dsr_io_t *io)
{
    uint8_t status = 0;
    lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_STATUS2, &status);
    return (status >> 6) & 1;
}

/**
 * @brief  查询 FIFO 满标志
 * @param  io I/O 抽象层指针
 * @return 1=满 / 0=未满
 */
uint8_t lsm6dsr_fifo_full_flag(lsm6dsr_io_t *io)
{
    uint8_t status = 0;
    lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_STATUS2, &status);
    return (status >> 5) & 1;
}

/**
 * @brief  冲刷 FIFO (切换为 Bypass 模式)
 * @param  io I/O 抽象层指针
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_fifo_flush(lsm6dsr_io_t *io)
{
    return lsm6dsr_fifo_set_mode(io, FIFO_MODE_BYPASS);
}

/**
 * @brief  设置 FIFO 水印阈值
 * @param  io        I/O 抽象层指针
 * @param  threshold 阈值 (9-bit, 0-511)
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_fifo_set_wtm(lsm6dsr_io_t *io, uint16_t threshold)
{
    lsm6dsr_status_t st;
    st = lsm6dsr_write_reg(io, LSM6DSR_REG_FIFO_CTRL1, (uint8_t)(threshold & 0xFF));
    if (st != LSM6DSR_OK) return st;
    uint8_t ctrl2;
    st = lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_CTRL2, &ctrl2);
    if (st != LSM6DSR_OK) return st;
    ctrl2 = (ctrl2 & 0xFE) | ((threshold >> 8) & 0x01);
    return lsm6dsr_write_reg(io, LSM6DSR_REG_FIFO_CTRL2, ctrl2);
}

/*============================================================================*
 * BDU / IF_INC Control
 *============================================================================*/
/**
 * @brief  使能/禁用块数据更新 (BDU)
 * @details BDU 使能时，输出寄存器在读取完成前不更新，避免读取过程中数据改变。
 * @param  io     I/O 抽象层指针
 * @param  enable 1=使能 / 0=禁用
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_set_bdu(lsm6dsr_io_t *io, uint8_t enable)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL3_C, &val);
    if (st != LSM6DSR_OK) return st;
    if (enable) val |= CTRL3_C_BDU;
    else        val &= ~CTRL3_C_BDU;
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL3_C, val);
}

/**
 * @brief  使能/禁用寄存器地址自动递增 (IF_INC)
 * @details IF_INC 使能时，多字节读取会自动递增寄存器地址。
 *          必须使能以使用 lsm6dsr_read_multi 一次读取 6 字节三轴数据。
 * @param  io     I/O 抽象层指针
 * @param  enable 1=使能 / 0=禁用
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_set_if_inc(lsm6dsr_io_t *io, uint8_t enable)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL3_C, &val);
    if (st != LSM6DSR_OK) return st;
    if (enable) val |= CTRL3_C_IF_INC;
    else        val &= ~CTRL3_C_IF_INC;
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL3_C, val);
}

/*============================================================================*
 * FIFO Entry Read with Sensor Type
 *============================================================================*/
/**
 * @brief  读取 FIFO 条目并判断传感器类型
 * @param  io     I/O 抽象层指针
 * @param  sensor 输出传感器类型 (GYRO/ACC)
 * @param  data   输出三轴数据
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_read_fifo_entry(lsm6dsr_io_t *io,
                                          lsm6dsr_fifo_sensor_t *sensor,
                                          lsm6dsr_axis_t *data)
{
    if (sensor == NULL || data == NULL) return LSM6DSR_NULL_PTR;
    uint8_t tag, buf[6];
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_FIFO_DATA_OUT_TAG, &tag);
    if (st != LSM6DSR_OK) return st;
    st = lsm6dsr_read_multi(io, LSM6DSR_REG_FIFO_DATA_OUT_XL, buf, 6);
    if (st != LSM6DSR_OK) return st;
    data->x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    data->y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    data->z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    *sensor = FIFO_TAG_IS_ACC(tag) ? LSM6DSR_FIFO_SENSOR_ACC : LSM6DSR_FIFO_SENSOR_GYRO;
    return LSM6DSR_OK;
}

/*============================================================================*
 * Self-Test
 *============================================================================*/
/**
 * @brief  设置 ACC 自检模式
 * @param  io   I/O 抽象层指针
 * @param  mode 自检模式 (LSM6DSR_XL_ST_DISABLE/POSITIVE/NEGATIVE)
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_xl_self_test(lsm6dsr_io_t *io, uint8_t mode)
{
    if (io == NULL) return LSM6DSR_NULL_PTR;
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL5_C, &val);
    if (st != LSM6DSR_OK) return st;
    val = (val & 0xFC) | (mode & 0x03);
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL5_C, val);
}

/**
 * @brief  设置 GYRO 自检模式
 * @param  io   I/O 抽象层指针
 * @param  mode 自检模式 (LSM6DSR_GY_ST_DISABLE/POSITIVE/NEGATIVE)
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_gy_self_test(lsm6dsr_io_t *io, uint8_t mode)
{
    if (io == NULL) return LSM6DSR_NULL_PTR;
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL5_C, &val);
    if (st != LSM6DSR_OK) return st;
    val = (val & 0xF3) | ((mode & 0x03) << 2);
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL5_C, val);
}

/*============================================================================*
 * Power Mode Control
 *============================================================================*/
/**
 * @brief  设置 ACC 高性能模式
 * @param  io     I/O 抽象层指针
 * @param  enable 1=高性能 / 0=低功耗
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_xl_set_hm_mode(lsm6dsr_io_t *io, uint8_t enable)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL6_C, &val);
    if (st != LSM6DSR_OK) return st;
    if (enable) val |= CTRL6_C_XL_HM_MODE;
    else        val &= ~CTRL6_C_XL_HM_MODE;
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL6_C, val);
}

/**
 * @brief  设置 GYRO 高性能模式
 * @param  io     I/O 抽象层指针
 * @param  enable 1=高性能 / 0=低功耗
 * @return lsm6dsr_status_t
 */
lsm6dsr_status_t lsm6dsr_gy_set_hm_mode(lsm6dsr_io_t *io, uint8_t enable)
{
    uint8_t val;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL7_G, &val);
    if (st != LSM6DSR_OK) return st;
    if (enable) val |= CTRL7_G_GY_HM_MODE;
    else        val &= ~CTRL7_G_GY_HM_MODE;
    return lsm6dsr_write_reg(io, LSM6DSR_REG_CTRL7_G, val);
}
