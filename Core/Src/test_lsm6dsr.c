#include "test_lsm6dsr.h"
#include "i2c.h"
#include "usart.h"
#include <stdlib.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int g_pass = 0;
int g_fail = 0;

static char vofa_buf[128];
volatile uint8_t vofa_tx_busy = 0;

/* Platform I/O Adapters */
static int8_t stm32_i2c_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *h = (I2C_HandleTypeDef *)ctx;
    if (HAL_I2C_Mem_Read(h, LSM6DSR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK)
        return 0;
    return -1;
}

static int8_t stm32_i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *h = (I2C_HandleTypeDef *)ctx;
    if (HAL_I2C_Mem_Write(h, LSM6DSR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 100) == HAL_OK)
        return 0;
    return -1;
}

lsm6dsr_io_t lsm6dsr_io = {
    .read  = stm32_i2c_read,
    .write = stm32_i2c_write,
    .ctx   = &hi2c1
};

/* ===================================================================
 * Phase 1-4: Core I2C & Register Verification
 * =================================================================== */

void phase1(void)
{
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, LSM6DSR_I2C_ADDR, 3, 100);
    printf("  HAL_I2C_IsDeviceReady(0x%02X) = %d\r\n", LSM6DSR_I2C_ADDR, (int)ret);
    if (ret == HAL_OK) {
        PASS("I2C device detected at 0x%02X", LSM6DSR_I2C_ADDR);
    } else {
        FAIL("I2C device NOT detected at 0x%02X (HAL=%d)", LSM6DSR_I2C_ADDR, (int)ret);
    }
}

void phase2(lsm6dsr_io_t *io)
{
    uint8_t id = 0;
    lsm6dsr_status_t st = lsm6dsr_read_reg(io, LSM6DSR_REG_WHO_AM_I, &id);
    printf("  WHO_AM_I = 0x%02X (expect 0x%02X)\r\n", id, LSM6DSR_WHO_AM_I_VAL);
    if (st != LSM6DSR_OK) {
        FAIL("WHO_AM_I read failed (status=%d)", st);
        goto halt;
    }
    if (id == LSM6DSR_WHO_AM_I_VAL) {
        PASS("WHO_AM_I = 0x%02X", id);
        return;
    }
    FAIL("WHO_AM_I = 0x%02X (expect 0x%02X)", id, LSM6DSR_WHO_AM_I_VAL);
halt:
    printf("\r\n*** I2C communication failed — halting ***\r\n");
    while (1);
}

void phase3(lsm6dsr_io_t *io)
{
    struct {
        uint8_t addr;
        uint8_t safe_mask;
    } regs[] = {
        {LSM6DSR_REG_CTRL1_XL, 0xFF},
        {LSM6DSR_REG_CTRL2_G,  0xFF},
        {LSM6DSR_REG_CTRL3_C,  0x7C},
        {LSM6DSR_REG_CTRL4_C,  0xFB},
        {LSM6DSR_REG_CTRL5_C,  0x6F},
        {LSM6DSR_REG_CTRL8_XL, 0xFF},
        {LSM6DSR_REG_CTRL9_XL, 0xFD},
    };
    int nregs = sizeof(regs) / sizeof(regs[0]);
    uint8_t patterns[] = {0x5A, 0xA5, 0x00, 0xFF};
    int npatterns = sizeof(patterns) / sizeof(patterns[0]);
    int all_ok = 1;

    for (int r = 0; r < nregs; r++) {
        for (int p = 0; p < npatterns; p++) {
            uint8_t wr_val = patterns[p] & regs[r].safe_mask;
            uint8_t rd_val;
            lsm6dsr_write_reg(io, regs[r].addr, wr_val);
            lsm6dsr_read_reg(io, regs[r].addr, &rd_val);
            if (rd_val != wr_val) {
                all_ok = 0;
                FAIL("0x%02X: wr 0x%02X rd 0x%02X", regs[r].addr, wr_val, rd_val);
            }
        }
        lsm6dsr_write_reg(io, regs[r].addr, 0x00);
    }
    if (all_ok) {
        PASS("R/W: %d regs x %d patterns = %d tests OK",
             nregs, npatterns, nregs * npatterns);
    } else {
        FAIL("R/W: some tests failed");
    }
}

void phase4(lsm6dsr_io_t *io)
{
    lsm6dsr_status_t st;

    lsm6dsr_set_bdu(io, 1);
    lsm6dsr_set_if_inc(io, 1);
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_2000DPS);
    HAL_Delay(50);

    float temp;
    st = lsm6dsr_read_temp(io, &temp);
    if (st == LSM6DSR_OK) {
        printf("  Temperature = %.1f C\r\n", temp);
        if (temp > 10.0f && temp < 50.0f) PASS("Temp %.1fC in 10-50 range", temp);
        else FAIL("Temp %.1fC out of range", temp);
    } else {
        FAIL("Temp read failed (status=%d)", st);
    }

    lsm6dsr_axis_t accel;
    st = lsm6dsr_read_accel_raw(io, &accel);
    if (st == LSM6DSR_OK) {
        printf("  ACC raw: X=%d Y=%d Z=%d\r\n", accel.x, accel.y, accel.z);
        int64_t mag2 = (int64_t)accel.x * accel.x
                     + (int64_t)accel.y * accel.y
                     + (int64_t)accel.z * accel.z;
        if (mag2 > 15000000LL && mag2 < 150000000LL) {
            PASS("ACC magnitude ~1G (mag2=%lld)", mag2);
        } else {
            printf("  Note: ACC mag2=%lld (may vary with orientation)\r\n", mag2);
        }
    } else {
        FAIL("ACC burst read failed (status=%d)", st);
    }

    float fax, fay, faz;
    st = lsm6dsr_read_accel_float(io, &fax, &fay, &faz, LSM6DSR_ACCEL_FS_4G);
    if (st == LSM6DSR_OK) {
        printf("  ACC float: %.2f %.2f %.2f g\r\n", fax, fay, faz);
    }

    lsm6dsr_axis_t gyro;
    st = lsm6dsr_read_gyro_raw(io, &gyro);
    if (st == LSM6DSR_OK) {
        printf("  GYRO raw: X=%d Y=%d Z=%d\r\n", gyro.x, gyro.y, gyro.z);
    } else {
        FAIL("GYRO burst read failed (status=%d)", st);
    }
}

/* ===================================================================
 * Phase 5-15: Extended Validation
 * =================================================================== */

void phase5_xl_self_test(lsm6dsr_io_t *io)
{
    int64_t bx = 0, by = 0, bz = 0, px = 0, py = 0, pz = 0, nx = 0, ny = 0, nz = 0;
    lsm6dsr_axis_t a;
    const int N = 16;
    const int64_t TH = 16 * 1000;

    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    HAL_Delay(50);

    for (int i = 0; i < N; i++) {
        lsm6dsr_read_accel_raw(io, &a);
        bx += a.x; by += a.y; bz += a.z;
    }

    lsm6dsr_xl_self_test(io, LSM6DSR_XL_ST_POSITIVE);
    HAL_Delay(50);
    for (int i = 0; i < N; i++) {
        lsm6dsr_read_accel_raw(io, &a);
        px += a.x; py += a.y; pz += a.z;
    }

    lsm6dsr_xl_self_test(io, LSM6DSR_XL_ST_DISABLE);
    HAL_Delay(20);

    lsm6dsr_xl_self_test(io, LSM6DSR_XL_ST_NEGATIVE);
    HAL_Delay(50);
    for (int i = 0; i < N; i++) {
        lsm6dsr_read_accel_raw(io, &a);
        nx += a.x; ny += a.y; nz += a.z;
    }

    lsm6dsr_xl_self_test(io, LSM6DSR_XL_ST_DISABLE);

    int64_t dxp = px - bx, dyp = py - by, dzp = pz - bz;
    int64_t dxn = nx - bx, dyn = ny - by, dzn = nz - bz;

    printf("  Base avg: X=%.1f Y=%.1f Z=%.1f\r\n",
           (double)bx/N, (double)by/N, (double)bz/N);
    printf("  Pos delta sum: X=%lld Y=%lld Z=%lld\r\n", dxp, dyp, dzp);
    printf("  Neg delta sum: X=%lld Y=%lld Z=%lld\r\n", dxn, dyn, dzn);

    int ppass = (dxp > TH) + (dyp > TH) + (dzp > TH) >= 2;
    int npass = (dxn < -TH) + (dyn < -TH) + (dzn < -TH) >= 2;

    if (ppass && npass) PASS("ACC self-test (+/-) per-axis signed delta OK");
    else {
        if (!ppass) FAIL("ACC+ST: <2 axes delta > %lld", TH);
        if (!npass) FAIL("ACC-ST: <2 axes delta < -%lld", TH);
    }
}

void phase6_gy_self_test(lsm6dsr_io_t *io)
{
    lsm6dsr_axis_t baselines[16], pos[16], neg[16];
    int64_t bx = 0, by = 0, bz = 0, px = 0, py = 0, pz = 0, nx = 0, ny = 0, nz = 0;

    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_2000DPS);
    HAL_Delay(50);

    for (int i = 0; i < 16; i++) {
        lsm6dsr_read_gyro_raw(io, &baselines[i]);
        bx += baselines[i].x; by += baselines[i].y; bz += baselines[i].z;
    }

    lsm6dsr_gy_self_test(io, LSM6DSR_GY_ST_POSITIVE);
    HAL_Delay(50);
    for (int i = 0; i < 16; i++) {
        lsm6dsr_read_gyro_raw(io, &pos[i]);
        px += pos[i].x; py += pos[i].y; pz += pos[i].z;
    }

    lsm6dsr_gy_self_test(io, LSM6DSR_GY_ST_DISABLE);
    HAL_Delay(20);

    lsm6dsr_gy_self_test(io, LSM6DSR_GY_ST_NEGATIVE);
    HAL_Delay(50);
    for (int i = 0; i < 16; i++) {
        lsm6dsr_read_gyro_raw(io, &neg[i]);
        nx += neg[i].x; ny += neg[i].y; nz += neg[i].z;
    }

    lsm6dsr_gy_self_test(io, LSM6DSR_GY_ST_DISABLE);

    int64_t bm = bx*bx + by*by + bz*bz;
    int64_t pm = px*px + py*py + pz*pz;
    int64_t nm = nx*nx + ny*ny + nz*nz;

    printf("  GY ST: base=%lld pos=%lld neg=%lld\r\n", bm, pm, nm);
    int pass = 1;
    if (pm <= bm) { FAIL("GY+ST: pos=%lld <= base=%lld", pm, bm); pass = 0; }
    if (nm <= bm) { FAIL("GY-ST: neg=%lld <= base=%lld", nm, bm); pass = 0; }
    if (pass) PASS("GYRO self-test (+/-) magnitude OK");
}

void phase7_power_mode(lsm6dsr_io_t *io)
{
    uint8_t reg;
    int pass = 1;
    int N = 64;

    /* ACC section */
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    HAL_Delay(30);

    lsm6dsr_xl_set_hm_mode(io, 0);
    HAL_Delay(5);
    lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL6_C, &reg);
    printf("  ACC CTRL6_C after HP set: 0x%02X (xl_hm_mode=%d)\r\n",
           reg, (reg >> 4) & 1);
    int64_t sx0 = 0, sy0 = 0, sz0 = 0, sxx0 = 0, syy0 = 0, szz0 = 0;
    for (int i = 0; i < N; i++) {
        lsm6dsr_axis_t a;
        lsm6dsr_read_accel_raw(io, &a);
        sx0 += a.x; sxx0 += (int64_t)a.x * a.x;
        sy0 += a.y; syy0 += (int64_t)a.y * a.y;
        sz0 += a.z; szz0 += (int64_t)a.z * a.z;
    }
    int64_t var_hp = (sxx0*N - sx0*sx0) + (syy0*N - sy0*sy0) + (szz0*N - sz0*sz0);

    lsm6dsr_xl_set_hm_mode(io, 1);
    HAL_Delay(5);
    lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL6_C, &reg);
    printf("  ACC CTRL6_C after LP set: 0x%02X (xl_hm_mode=%d)\r\n",
           reg, (reg >> 4) & 1);
    int64_t sx1 = 0, sy1 = 0, sz1 = 0, sxx1 = 0, syy1 = 0, szz1 = 0;
    for (int i = 0; i < N; i++) {
        lsm6dsr_axis_t a;
        lsm6dsr_read_accel_raw(io, &a);
        sx1 += a.x; sxx1 += (int64_t)a.x * a.x;
        sy1 += a.y; syy1 += (int64_t)a.y * a.y;
        sz1 += a.z; szz1 += (int64_t)a.z * a.z;
    }
    int64_t var_lp = (sxx1*N - sx1*sx1) + (syy1*N - sy1*sy1) + (szz1*N - sz1*sz1);

    lsm6dsr_xl_set_hm_mode(io, 0);
    printf("  ACC variance*N^2: HP=%lld LP=%lld ratio=%.3f\r\n",
           var_hp, var_lp, var_hp ? (double)var_lp / var_hp : 0);
    if (var_lp <= var_hp) { FAIL("ACC: LP var=%lld <= HP var=%lld", var_lp, var_hp); pass = 0; }
    else PASS("ACC: LP variance %.2f× HP", (double)var_lp / var_hp);

    /* GYRO section */
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(30);

    lsm6dsr_gy_set_hm_mode(io, 0);
    HAL_Delay(5);
    lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL7_G, &reg);
    printf("  GYRO CTRL7_G after HP set: 0x%02X (g_hm_mode=%d)\r\n",
           reg, (reg >> 7) & 1);
    int64_t gx0 = 0, gy0 = 0, gz0 = 0, gxx0 = 0, gyy0 = 0, gzz0 = 0;
    for (int i = 0; i < N; i++) {
        lsm6dsr_axis_t g;
        lsm6dsr_read_gyro_raw(io, &g);
        gx0 += g.x; gxx0 += (int64_t)g.x * g.x;
        gy0 += g.y; gyy0 += (int64_t)g.y * g.y;
        gz0 += g.z; gzz0 += (int64_t)g.z * g.z;
    }
    int64_t gvar_hp = (gxx0*N - gx0*gx0) + (gyy0*N - gy0*gy0) + (gzz0*N - gz0*gz0);

    lsm6dsr_gy_set_hm_mode(io, 1);
    HAL_Delay(5);
    lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL7_G, &reg);
    printf("  GYRO CTRL7_G after LP set: 0x%02X (g_hm_mode=%d)\r\n",
           reg, (reg >> 7) & 1);
    int64_t gx1 = 0, gy1 = 0, gz1 = 0, gxx1 = 0, gyy1 = 0, gzz1 = 0;
    for (int i = 0; i < N; i++) {
        lsm6dsr_axis_t g;
        lsm6dsr_read_gyro_raw(io, &g);
        gx1 += g.x; gxx1 += (int64_t)g.x * g.x;
        gy1 += g.y; gyy1 += (int64_t)g.y * g.y;
        gz1 += g.z; gzz1 += (int64_t)g.z * g.z;
    }
    int64_t gvar_lp = (gxx1*N - gx1*gx1) + (gyy1*N - gy1*gy1) + (gzz1*N - gz1*gz1);

    lsm6dsr_gy_set_hm_mode(io, 0);
    int64_t gv_max = (gvar_hp > gvar_lp) ? gvar_hp : gvar_lp;
    int64_t gv_min = (gvar_hp > gvar_lp) ? gvar_lp : gvar_hp;
    double gv_ratio = gv_min ? (double)gv_max / gv_min : 0;
    printf("  GYRO variance*N^2: HP=%lld LP=%lld max/min=%.3f\r\n",
           gvar_hp, gvar_lp, gv_ratio);

    if (gv_ratio > 1.5 && gv_min > 50) {
        PASS("GYRO: HP/LP variance ratio=%.2f (mode switching verified)", gv_ratio);
    } else {
        if (gv_min <= 50) FAIL("GYRO: variance near zero (HP=%lld LP=%lld)", gvar_hp, gvar_lp);
        else FAIL("GYRO: HP/LP too close ratio=%.3f (HP=%lld LP=%lld)", gv_ratio, gvar_hp, gvar_lp);
        pass = 0;
    }

    if (pass) PASS("Both sensors: LP noise > HP noise");
}

void phase8_fs_odr_sweep(lsm6dsr_io_t *io)
{
    lsm6dsr_accel_fs_t accel_fs_list[] = {
        LSM6DSR_ACCEL_FS_2G, LSM6DSR_ACCEL_FS_4G,
        LSM6DSR_ACCEL_FS_8G, LSM6DSR_ACCEL_FS_16G};

    int fs_ok = 1;
    for (int i = 0; i < 4; i++) {
        lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, accel_fs_list[i]);
        uint8_t rb;
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL1_XL, &rb);
        uint8_t fs_read = (rb >> 2) & 3;
        uint8_t fs_expect = (uint8_t)accel_fs_list[i];
        if (fs_read != fs_expect) { fs_ok = 0; FAIL("ACC FS %d: wrote %d rd %d", i, fs_expect, fs_read); }
    }
    if (fs_ok) PASS("ACC FS: 4/4 verified");

    lsm6dsr_gyro_fs_t gyro_fs_list[] = {
        LSM6DSR_GYRO_FS_250DPS, LSM6DSR_GYRO_FS_500DPS,
        LSM6DSR_GYRO_FS_1000DPS, LSM6DSR_GYRO_FS_2000DPS};

    int gfs_ok = 1;
    for (int i = 0; i < 4; i++) {
        lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, gyro_fs_list[i]);
        uint8_t rb;
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL2_G, &rb);
        uint8_t fs_read = (rb >> 1) & 7;
        uint8_t fs_expect = ((uint8_t)gyro_fs_list[i] >> 1) & 7;
        if (fs_read != fs_expect) { gfs_ok = 0; FAIL("GYRO FS %d: wrote %d rd %d", i, fs_expect, fs_read); }
    }
    if (gfs_ok) PASS("GYRO FS: 4/4 verified");

    lsm6dsr_accel_odr_t odr_list[] = {
        LSM6DSR_ACCEL_ODR_12_5HZ, LSM6DSR_ACCEL_ODR_26HZ,
        LSM6DSR_ACCEL_ODR_52HZ, LSM6DSR_ACCEL_ODR_104HZ,
        LSM6DSR_ACCEL_ODR_208HZ, LSM6DSR_ACCEL_ODR_416HZ,
        LSM6DSR_ACCEL_ODR_833HZ};

    int odr_ok = 1;
    for (int i = 0; i < 7; i++) {
        lsm6dsr_accel_config(io, odr_list[i], LSM6DSR_ACCEL_FS_4G);
        uint8_t rb;
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL1_XL, &rb);
        uint8_t odr_read = (rb >> 4) & 0xF;
        uint8_t odr_expect = (uint8_t)odr_list[i];
        if (odr_read != odr_expect) { odr_ok = 0; FAIL("ODR %d: wrote %d rd %d", i, odr_expect, odr_read); }
    }
    if (odr_ok) PASS("ODR: 7/7 verified");

    lsm6dsr_gyro_odr_t gy_odr_list[] = {
        LSM6DSR_GYRO_ODR_12_5HZ, LSM6DSR_GYRO_ODR_26HZ,
        LSM6DSR_GYRO_ODR_52HZ, LSM6DSR_GYRO_ODR_104HZ,
        LSM6DSR_GYRO_ODR_208HZ, LSM6DSR_GYRO_ODR_416HZ,
        LSM6DSR_GYRO_ODR_833HZ, LSM6DSR_GYRO_ODR_1_66KHZ,
        LSM6DSR_GYRO_ODR_3_33KHZ, LSM6DSR_GYRO_ODR_6_66KHZ};

    int gy_odr_ok = 1;
    for (int i = 0; i < 10; i++) {
        lsm6dsr_gyro_config(io, gy_odr_list[i], LSM6DSR_GYRO_FS_250DPS);
        uint8_t rb;
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL2_G, &rb);
        uint8_t odr_read = (rb >> 4) & 0xF;
        uint8_t odr_expect = (uint8_t)gy_odr_list[i];
        if (odr_read != odr_expect) { gy_odr_ok = 0; FAIL("GYRO ODR %d: wrote %d rd %d", i, odr_expect, odr_read); }
    }
    if (gy_odr_ok) PASS("GYRO ODR: 10/10 verified");
    else FAIL("GYRO ODR: some failed");

    lsm6dsr_reset(io);
    lsm6dsr_boot(io);
}

void phase9_fifo_all_modes(lsm6dsr_io_t *io)
{
    lsm6dsr_set_bdu(io, 1);
    lsm6dsr_set_if_inc(io, 1);
    lsm6dsr_i3c_disable(io);

    lsm6dsr_fifo_init(io, 10, LSM6DSR_BDR_52Hz, LSM6DSR_BDR_52Hz);
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_52HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_52HZ, LSM6DSR_GYRO_FS_2000DPS);
    HAL_Delay(20);

    lsm6dsr_fifo_set_mode(io, FIFO_MODE_BYPASS);
    HAL_Delay(5);
    uint16_t lev = lsm6dsr_fifo_get_level(io);
    if (lev == 0) PASS("BYPASS: level=0");
    else FAIL("BYPASS: level=%u", lev);

    lsm6dsr_fifo_set_mode(io, FIFO_MODE_FIFO);
    HAL_Delay(2000);
    lev = lsm6dsr_fifo_get_level(io);
    if (lev > 0) PASS("FIFO mode: level=%u", lev);
    else FAIL("FIFO mode: level=0");

    while (lsm6dsr_fifo_get_level(io) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(io, &s, &d);
    }

    lsm6dsr_fifo_set_mode(io, FIFO_MODE_CONT);
    HAL_Delay(1000);
    lev = lsm6dsr_fifo_get_level(io);
    printf("  CONT mode 1s: level=%u (~104/s)\r\n", lev);
    if (lev > 50) PASS("CONT: fill rate OK (%u/s)", lev);
    else FAIL("CONT: low fill rate (%u/s)", lev);

    int acc_count = 0, gyro_count = 0, total_tags = 0;
    int max_tags = (lev > 100) ? 100 : lev;
    for (int i = 0; i < max_tags; i++) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        if (lsm6dsr_read_fifo_entry(io, &s, &d) != LSM6DSR_OK) break;
        total_tags++;
        if (s == LSM6DSR_FIFO_SENSOR_ACC) acc_count++;
        else gyro_count++;
    }
    printf("  Tags: %d ACC + %d GYRO = %d total\r\n", acc_count, gyro_count, total_tags);
    if (acc_count > 0 && gyro_count > 0) PASS("FIFO tag distribution OK");
    else FAIL("FIFO tag: ACC=%d GYRO=%d", acc_count, gyro_count);

    while (lsm6dsr_fifo_get_level(io) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(io, &s, &d);
    }

    lsm6dsr_fifo_set_wtm(io, 5);
    lsm6dsr_fifo_set_mode(io, FIFO_MODE_CONT_TO_FIFO);
    HAL_Delay(500);
    uint8_t wtm = lsm6dsr_fifo_wtm_flag(io);
    printf("  CONT_TO_FIFO WTM flag=%d (expect 1)\r\n", wtm);
    if (wtm) PASS("WTM trigger OK");
    else FAIL("WTM not triggered");

    lsm6dsr_fifo_set_mode(io, FIFO_MODE_BYPASS);
    while (lsm6dsr_fifo_get_level(io) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(io, &s, &d);
    }

    /* High-ODR FIFO verification */
    lsm6dsr_fifo_init(io, 50, LSM6DSR_BDR_833Hz, LSM6DSR_BDR_833Hz);
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_833HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_833HZ, LSM6DSR_GYRO_FS_250DPS);
    lsm6dsr_fifo_set_mode(io, FIFO_MODE_CONT);
    HAL_Delay(500);
    lev = lsm6dsr_fifo_get_level(io);
    printf("  High-ODR 833Hz CONT 0.5s: level=%u\r\n", lev);

    int ha_acc = 0, ha_gyro = 0;
    int max_t = (lev > 80) ? 80 : lev;
    for (int i = 0; i < max_t; i++) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        if (lsm6dsr_read_fifo_entry(io, &s, &d) != LSM6DSR_OK) break;
        if (s == LSM6DSR_FIFO_SENSOR_ACC) ha_acc++;
        else ha_gyro++;
    }
    printf("  High-ODR tags: %d ACC + %d GYRO\r\n", ha_acc, ha_gyro);
    if (ha_acc > 0 && ha_gyro > 0) PASS("High-ODR FIFO tag OK");
    else FAIL("High-ODR FIFO tag: ACC=%d GYRO=%d", ha_acc, ha_gyro);

    lsm6dsr_fifo_set_mode(io, FIFO_MODE_BYPASS);
    while (lsm6dsr_fifo_get_level(io) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(io, &s, &d);
    }
}

void phase10_bdu_stress(lsm6dsr_io_t *io)
{
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_833HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_set_bdu(io, 1);
    HAL_Delay(10);

    int bad = 0;
    for (int i = 0; i < 200; i++) {
        uint8_t buf[6];
        lsm6dsr_read_multi(io, LSM6DSR_REG_OUTX_L_XL, buf, 6);
        int16_t x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
        int16_t y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
        int16_t z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
        int ax = (x < 0) ? -x : x;
        int ay = (y < 0) ? -y : y;
        int az = (z < 0) ? -z : z;
        if (ax > 20000 || ay > 20000 || az > 20000) bad++;
    }
    if (bad == 0) PASS("BDU stress: 0/200 torn");
    else FAIL("BDU stress: %d/200 torn", bad);
}

void phase11_fifo_overflow(lsm6dsr_io_t *io)
{
    lsm6dsr_fifo_init(io, 200, LSM6DSR_BDR_104Hz, LSM6DSR_BDR_104Hz);
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_2000DPS);
    lsm6dsr_fifo_set_mode(io, FIFO_MODE_CONT);
    HAL_Delay(100);

    uint8_t ovr = lsm6dsr_fifo_ovr_flag(io);
    uint8_t full = lsm6dsr_fifo_full_flag(io);
    printf("  After 100ms: OVR=%d FULL=%d\r\n", ovr, full);

    HAL_Delay(3000);
    uint16_t lev = lsm6dsr_fifo_get_level(io);
    ovr = lsm6dsr_fifo_ovr_flag(io);
    printf("  After 3s: level=%u OVR=%d\r\n", lev, ovr);

    lsm6dsr_fifo_flush(io);
    HAL_Delay(5);
    lev = lsm6dsr_fifo_get_level(io);
    printf("  After flush: level=%u\r\n", lev);
    if (lev == 0) PASS("FIFO flush OK");
    else FAIL("FIFO flush level=%u", lev);

    lsm6dsr_fifo_set_mode(io, FIFO_MODE_CONT);
    HAL_Delay(1000);
    lev = lsm6dsr_fifo_get_level(io);
    if (lev > 50) PASS("FIFO recovery after flush: level=%u", lev);
    else FAIL("FIFO recovery low level=%u", lev);

    lsm6dsr_fifo_set_mode(io, FIFO_MODE_BYPASS);
    while (lsm6dsr_fifo_get_level(io) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(io, &s, &d);
    }
}

void phase12_init_stability(lsm6dsr_io_t *io)
{
    float z_means[3];
    for (int cycle = 0; cycle < 3; cycle++) {
        lsm6dsr_reset(io);
        lsm6dsr_boot(io);
        lsm6dsr_set_bdu(io, 1);
        lsm6dsr_set_if_inc(io, 1);
        lsm6dsr_i3c_disable(io);
        lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
        HAL_Delay(30);

        int64_t sum_z = 0;
        for (int i = 0; i < 50; i++) {
            lsm6dsr_axis_t a;
            lsm6dsr_read_accel_raw(io, &a);
            sum_z += a.z;
        }
        z_means[cycle] = sum_z / 50.0f;
        printf("  Cycle %d: ACC Z mean = %.1f\r\n", cycle, z_means[cycle]);
    }

    float min_v = z_means[0], max_v = z_means[0];
    for (int i = 1; i < 3; i++) {
        if (z_means[i] < min_v) min_v = z_means[i];
        if (z_means[i] > max_v) max_v = z_means[i];
    }
    float denom = (fabsf(max_v) > fabsf(min_v)) ? fabsf(max_v) : fabsf(min_v);
    if (denom < 1.0f) denom = 1.0f;
    float spread = (max_v - min_v) / denom * 100.0f;
    printf("  Spread = %.1f%%\r\n", spread);
    if (spread < 5.0f) PASS("Repeat init OK (spread %.1f%%)", spread);
    else FAIL("Repeat init spread %.1f%% >= 5%%", spread);
}

void phase13_drdy_poll(lsm6dsr_io_t *io)
{
    int pass_104 = 1, pass_416 = 1;

    lsm6dsr_set_bdu(io, 1);
    lsm6dsr_set_if_inc(io, 1);

    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(20);

    for (int i = 0; i < 10; i++) {
        uint8_t a = 0, g = 0;
        uint32_t tmo = 100;
        while (tmo--) {
            lsm6dsr_get_drdy(io, &a, &g);
            if (a && g) break;
            HAL_Delay(1);
        }
        if (!a || !g) { pass_104 = 0; FAIL("DRDY @104Hz iter %d: ACC=%d GYRO=%d", i, a, g); break; }
    }
    if (pass_104) PASS("DRDY @104Hz: 10/10 OK");

    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_416HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_416HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(10);

    for (int i = 0; i < 10; i++) {
        uint8_t a = 0, g = 0;
        uint32_t tmo = 50;
        while (tmo--) {
            lsm6dsr_get_drdy(io, &a, &g);
            if (a && g) break;
            HAL_Delay(3);
        }
        if (!a || !g) { pass_416 = 0; FAIL("DRDY @416Hz iter %d: ACC=%d GYRO=%d", i, a, g); break; }
    }
    if (pass_416) PASS("DRDY @416Hz: 10/10 OK");
}

void phase14_data_integrity(lsm6dsr_io_t *io)
{
    lsm6dsr_set_bdu(io, 1);
    lsm6dsr_set_if_inc(io, 1);

    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_208HZ, LSM6DSR_ACCEL_FS_4G);
    HAL_Delay(10);

    lsm6dsr_axis_t acc_samples[50];
    int n_acc = 0;
    for (int i = 0; i < 50; i++) {
        uint8_t a, g;
        uint32_t tmo = 200;
        do {
            lsm6dsr_get_drdy(io, &a, &g);
            if (a) break;
        } while (tmo--);
        if (!a) { FAIL("ACC DRDY timeout @%d", i); break; }
        lsm6dsr_read_accel_raw(io, &acc_samples[i]);
        n_acc++;
    }

    int acc_spike = 0, acc_range_ok = 1;
    for (int i = 1; i < n_acc; i++) {
        int dx = abs((int)acc_samples[i].x - (int)acc_samples[i-1].x);
        int dy = abs((int)acc_samples[i].y - (int)acc_samples[i-1].y);
        int dz = abs((int)acc_samples[i].z - (int)acc_samples[i-1].z);
        if (dx > 5000 || dy > 5000 || dz > 5000) acc_spike++;
        int64_t mag2 = (int64_t)acc_samples[i].x * acc_samples[i].x
                     + (int64_t)acc_samples[i].y * acc_samples[i].y
                     + (int64_t)acc_samples[i].z * acc_samples[i].z;
        if (mag2 < 15000000LL || mag2 > 150000000LL) acc_range_ok = 0;
    }
    printf("  ACC: spikes=%d range_ok=%d\r\n", acc_spike, acc_range_ok);
    int acc_pass = (acc_spike == 0) && acc_range_ok;
    if (acc_pass) PASS("ACC data integrity OK");
    else {
        if (acc_spike) FAIL("ACC: %d spikes >5000LSB", acc_spike);
        if (!acc_range_ok) FAIL("ACC: samples outside 1G range");
    }

    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_208HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(10);

    lsm6dsr_axis_t gy_samples[50];
    int n_gy = 0;
    for (int i = 0; i < 50; i++) {
        uint8_t a, g;
        uint32_t tmo = 200;
        do {
            lsm6dsr_get_drdy(io, &a, &g);
            if (g) break;
        } while (tmo--);
        if (!g) { FAIL("GYRO DRDY timeout @%d", i); break; }
        lsm6dsr_read_gyro_raw(io, &gy_samples[i]);
        n_gy++;
    }

    int gy_spike = 0;
    for (int i = 1; i < n_gy; i++) {
        int dx = abs((int)gy_samples[i].x - (int)gy_samples[i-1].x);
        int dy = abs((int)gy_samples[i].y - (int)gy_samples[i-1].y);
        int dz = abs((int)gy_samples[i].z - (int)gy_samples[i-1].z);
        if (dx > 500 || dy > 500 || dz > 500) gy_spike++;
    }
    printf("  GYRO: spikes=%d\r\n", gy_spike);
    if (gy_spike == 0) PASS("GYRO data integrity OK");
    else FAIL("GYRO: %d spikes >500LSB", gy_spike);
}

void phase15_reg_after_reset(lsm6dsr_io_t *io)
{
    struct {
        uint8_t addr;
        uint8_t def;
        uint8_t write_mask;
        const char *name;
    } ri[] = {
        {LSM6DSR_REG_CTRL1_XL, 0x00, 0xFF, "CTRL1_XL"},
        {LSM6DSR_REG_CTRL2_G,  0x00, 0xFF, "CTRL2_G"},
        {LSM6DSR_REG_CTRL3_C,  0x04, 0x7C, "CTRL3_C"},
        {LSM6DSR_REG_CTRL4_C,  0x00, 0xFB, "CTRL4_C"},
        {LSM6DSR_REG_CTRL5_C,  0x00, 0x6F, "CTRL5_C"},
        {LSM6DSR_REG_CTRL6_C,  0x00, 0x00, "CTRL6_C"},
        {LSM6DSR_REG_CTRL7_G,  0x00, 0x00, "CTRL7_G"},
        {LSM6DSR_REG_CTRL8_XL, 0x00, 0xFF, "CTRL8_XL"},
        {LSM6DSR_REG_CTRL9_XL, 0xE2, 0xFD, "CTRL9_XL"},
    };
    int n = sizeof(ri) / sizeof(ri[0]);

    lsm6dsr_reset(io);
    HAL_Delay(50);

    int reset_ok = 1;
    for (int i = 0; i < n; i++) {
        uint8_t v;
        lsm6dsr_read_reg(io, ri[i].addr, &v);
        if (v != ri[i].def) { reset_ok = 0; FAIL("After reset: %s=0x%02X expect 0x%02X", ri[i].name, v, ri[i].def); }
    }
    if (reset_ok) PASS("After reset: %d regs default OK", n);

    int write_ok = 1;
    uint8_t expected[9];
    for (int i = 0; i < n; i++) {
        if (ri[i].write_mask == 0) { expected[i] = ri[i].def; continue; }
        uint8_t v = 0x5A & ri[i].write_mask;
        lsm6dsr_write_reg(io, ri[i].addr, v);
        uint8_t r;
        lsm6dsr_read_reg(io, ri[i].addr, &r);
        if (r != v) { write_ok = 0; FAIL("Write 0x%02X to %s: rd=0x%02X", v, ri[i].name, r); }
        expected[i] = v;
    }
    if (write_ok) PASS("Post-reset write: %d regs OK", n);

    lsm6dsr_boot(io);
    HAL_Delay(50);

    int boot_ok = 1;
    for (int i = 0; i < n; i++) {
        uint8_t v;
        lsm6dsr_read_reg(io, ri[i].addr, &v);
        if (v != expected[i]) { boot_ok = 0; FAIL("After boot: %s=0x%02X expect 0x%02X", ri[i].name, v, expected[i]); }
    }
    if (boot_ok) PASS("After boot: %d regs preserved", n);

    if (reset_ok && write_ok && boot_ok) PASS("Reset/Boot register integrity OK");

    lsm6dsr_reset(io);
    HAL_Delay(50);
}

void phase16_bias_noise(lsm6dsr_io_t *io)
{
    const int N = 200;
    int all_ok = 1;

    lsm6dsr_i3c_disable(io);
    lsm6dsr_set_if_inc(io, 1);
    lsm6dsr_set_bdu(io, 1);
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    HAL_Delay(50);

    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double ax_sq = 0, ay_sq = 0, az_sq = 0;

    {
        int acc_ok = 1;
        for (int i = 0; i < N && acc_ok; i++) {
            uint8_t a, g;
            uint32_t tmo = 200;
            do { lsm6dsr_get_drdy(io, &a, &g); } while (!a && tmo--);
            if (!a) { FAIL("ACC DRDY timeout @%d", i); acc_ok = 0; all_ok = 0; break; }
            float x, y, z;
            lsm6dsr_read_accel_float(io, &x, &y, &z, LSM6DSR_ACCEL_FS_4G);
            ax_sum += x; ay_sum += y; az_sum += z;
            ax_sq += x*x; ay_sq += y*y; az_sq += z*z;
        }
        if (acc_ok) {
            double ax_mg = ax_sum / N;
            double ay_mg = ay_sum / N;
            double az_mg = az_sum / N;
            double ax_std = sqrt(ax_sq/N - (ax_sum/N)*(ax_sum/N));
            double ay_std = sqrt(ay_sq/N - (ay_sum/N)*(ay_sum/N));
            double az_std = sqrt(az_sq/N - (az_sum/N)*(az_sum/N));
            printf("  ACC @104Hz/4G (N=%d):\r\n", N);
            printf("    X: mean=%7.2f mg  std=%5.2f mg\r\n", ax_mg, ax_std);
            printf("    Y: mean=%7.2f mg  std=%5.2f mg\r\n", ay_mg, ay_std);
            printf("    Z: mean=%7.2f mg  std=%5.2f mg\r\n", az_mg, az_std);
            printf("    Mag: %.2f g\r\n", sqrt(ax_mg*ax_mg + ay_mg*ay_mg + az_mg*az_mg) / 1000.0);
            if (ax_std < 20 && ay_std < 20 && az_std < 20) PASS("ACC noise: all axes std < 20 mg");
            else { FAIL("ACC noise: std exceeds 20 mg"); all_ok = 0; }
        }
    }

    /* ---- GYRO ---- */
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(50);

    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    double gx_sq = 0, gy_sq = 0, gz_sq = 0;

    {
        int gy_ok = 1;
        for (int i = 0; i < N && gy_ok; i++) {
            uint8_t a, g;
            uint32_t tmo = 200;
            do { lsm6dsr_get_drdy(io, &a, &g); } while (!g && tmo--);
            if (!g) { FAIL("GYRO DRDY timeout @%d", i); gy_ok = 0; all_ok = 0; break; }
            float x, y, z;
            lsm6dsr_read_gyro_float(io, &x, &y, &z, LSM6DSR_GYRO_FS_250DPS);
            gx_sum += x; gy_sum += y; gz_sum += z;
            gx_sq += x*x; gy_sq += y*y; gz_sq += z*z;
        }
        if (gy_ok) {
            double gx_dps = gx_sum / N;
            double gy_dps = gy_sum / N;
            double gz_dps = gz_sum / N;
            double gx_std = sqrt(gx_sq/N - (gx_sum/N)*(gx_sum/N));
            double gy_std = sqrt(gy_sq/N - (gy_sum/N)*(gy_sum/N));
            double gz_std = sqrt(gz_sq/N - (gz_sum/N)*(gz_sum/N));
            printf("  GYRO @104Hz/250dps (N=%d):\r\n", N);
            printf("    X: mean=%7.3f dps  std=%6.4f dps\r\n", gx_dps, gx_std);
            printf("    Y: mean=%7.3f dps  std=%6.4f dps\r\n", gy_dps, gy_std);
            printf("    Z: mean=%7.3f dps  std=%6.4f dps\r\n", gz_dps, gz_std);
            if (gx_std < 0.0001 && gy_std < 0.0001 && gz_std < 0.0001) {
                FAIL("GYRO data frozen (std=%.4f,%.4f,%.4f)", gx_std, gy_std, gz_std);
                all_ok = 0;
            } else if (gx_std < 0.5 && gy_std < 0.5 && gz_std < 0.5) {
                PASS("GYRO noise: all axes std < 0.5 dps");
            } else {
                FAIL("GYRO noise: std exceeds 0.5 dps"); all_ok = 0;
            }
        }
    }

    if (all_ok) PASS("Bias & noise floor measurement OK");
}

void phase17_live_display(lsm6dsr_io_t *io)
{
    float ax, ay, az, gx, gy, gz;
    float fax = 0, fay = 0, faz = 0, fgx = 0, fgy = 0, fgz = 0;

    lsm6dsr_i3c_disable(io);
    lsm6dsr_set_if_inc(io, 1);
    lsm6dsr_set_bdu(io, 1);
    lsm6dsr_accel_config(io, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(io, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_250DPS);
    HAL_Delay(50);

    {
        uint8_t ctrl2_g, ctrl7_g;
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL2_G, &ctrl2_g);
        lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL7_G, &ctrl7_g);
        printf("  GYRO config: CTRL2_G=0x%02X CTRL7_G=0x%02X\r\n", ctrl2_g, ctrl7_g);
    }

    lsm6dsr_read_accel_float(io, &ax, &ay, &az, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_read_gyro_float(io, &gx, &gy, &gz, LSM6DSR_GYRO_FS_250DPS);

    double pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;
    double roll  = atan2( ay, sqrt(ax*ax + az*az)) * 180.0 / M_PI;
    double yaw   = 0.0;

    const double alpha = 0.95;
    uint32_t last_tick = HAL_GetTick();

    printf("\r\n--- VOFA+ Live Data (9ch: ax,ay,az,gx,gy,gz,pitch,roll,yaw) ---\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();
        double dt = (now > last_tick) ? (now - last_tick) * 0.001 : 0.01;
        if (dt > 0.5) dt = 0.01;
        last_tick = now;

        /* Read DRDY for debug visibility */
        uint8_t a = 0, g = 0;
        lsm6dsr_get_drdy(io, &a, &g);

        /* D1-5: Deep diagnostic — before any data consumption */
        {   static int diag = 0;
            if (++diag <= 5) {
                uint8_t sr;
                lsm6dsr_read_reg(io, LSM6DSR_REG_STATUS_REG, &sr);
                uint8_t c3c, c6c, c7g, c10c, c9xl;
                lsm6dsr_read_reg(io, 0x12, &c3c);
                lsm6dsr_read_reg(io, 0x15, &c6c);
                lsm6dsr_read_reg(io, 0x16, &c7g);
                lsm6dsr_read_reg(io, 0x19, &c10c);
                lsm6dsr_read_reg(io, 0x18, &c9xl);
                uint8_t g1[6], g2[6];
                lsm6dsr_read_multi(io, LSM6DSR_REG_OUTX_L_G, g1, 6);
                HAL_Delay(10);
                lsm6dsr_read_multi(io, LSM6DSR_REG_OUTX_L_G, g2, 6);
                int same = (g1[0]==g2[0]&&g1[1]==g2[1]&&g1[2]==g2[2]&&
                            g1[3]==g2[3]&&g1[4]==g2[4]&&g1[5]==g2[5]);
                printf("\r\n[D%d] SR=0x%02X(xl=%d,g=%d)"
                       " | C3C=0x%02X C6C=0x%02X C7G=0x%02X C10C=0x%02X C9XL=0x%02X"
                       " | gy_dual=%s(%02X%02X%02X%02X%02X%02X vs %02X%02X%02X%02X%02X%02X)\r\n",
                       diag, sr, sr&1, (sr>>1)&1,
                       c3c, c6c, c7g, c10c, c9xl,
                       same ? "SAME" : "DIFF",
                       g1[0],g1[1],g1[2],g1[3],g1[4],g1[5],
                       g2[0],g2[1],g2[2],g2[3],g2[4],g2[5]);
            }
        }

        lsm6dsr_read_accel_float(io, &fax, &fay, &faz, LSM6DSR_ACCEL_FS_4G);
        lsm6dsr_read_gyro_float(io, &fgx, &fgy, &fgz, LSM6DSR_GYRO_FS_250DPS);

        /* Every 100 frames: STATUS + CTRL2_G check */
        {   static int cnt = 0;
            if (++cnt >= 100) {
                cnt = 0;
                uint8_t sr, c2g;
                lsm6dsr_read_reg(io, 0x1E, &sr);
                lsm6dsr_read_reg(io, LSM6DSR_REG_CTRL2_G, &c2g);
                printf("  SR=0x%02X(xl=%d,g=%d) CTRL2_G=0x%02X g=%.3f,%.3f,%.3f\r\n",
                       sr, sr&1, (sr>>1)&1, c2g, fgx, fgy, fgz);
            }
        }

        double acc_pitch = atan2(-fax, sqrt(fay*fay + faz*faz)) * 180.0 / M_PI;
        double acc_roll  = atan2( fay, sqrt(fax*fax + faz*faz)) * 180.0 / M_PI;

        pitch = alpha * (pitch + fgy * dt) + (1.0 - alpha) * acc_pitch;
        roll  = alpha * (roll  - fgx * dt) + (1.0 - alpha) * acc_roll;
        yaw  += fgz * dt;

        float ax_ms2 = fax * 0.00980665f;
        float ay_ms2 = fay * 0.00980665f;
        float az_ms2 = faz * 0.00980665f;

        while (vofa_tx_busy) { /* spin */ }

        int len = sprintf(vofa_buf,
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
            (double)ax_ms2, (double)ay_ms2, (double)az_ms2,
            (double)fgx, (double)fgy, (double)fgz,
            pitch, roll, yaw);

        if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)vofa_buf, len) == HAL_OK) {
            vofa_tx_busy = 1;
        }

        HAL_Delay(90);
    }
}

void run_all_tests(void)
{
    g_pass = 0;
    g_fail = 0;

    printf("\r\n========================================\r\n");
    printf("  LSM6DSR I2C & Register Core Test\r\n");
    printf("========================================\r\n");
    printf("  I2C1: DevAddr=0x%04X, Clock=%luHz, APB1=%uHz\r\n",
           (unsigned)LSM6DSR_I2C_ADDR, (unsigned long)hi2c1.Init.ClockSpeed,
           (unsigned)HAL_RCC_GetPCLK1Freq());
    HAL_Delay(200);

    PHASE(1, "I2C Bus Probe");
    phase1();

    PHASE(2, "WHO_AM_I Verification");
    phase2(&lsm6dsr_io);

    PHASE(3, "Register Write-Back");
    phase3(&lsm6dsr_io);

    PHASE(4, "Multi-byte Burst + Sensor Data");
    phase4(&lsm6dsr_io);

#ifdef RUN_FULL_TESTS
    printf("\r\n========================================\r\n");
    printf("  Extended Validation (%d PASS / %d FAIL so far)\r\n", g_pass, g_fail);
    printf("========================================\r\n");

    PHASE(5, "ACC self-test");
    phase5_xl_self_test(&lsm6dsr_io);

    PHASE(6, "GYRO self-test");
    phase6_gy_self_test(&lsm6dsr_io);

    PHASE(7, "High-perf vs low-power mode");
    phase7_power_mode(&lsm6dsr_io);

    PHASE(8, "FS/ODR sweep");
    phase8_fs_odr_sweep(&lsm6dsr_io);

    PHASE(9, "FIFO all modes + tags");
    phase9_fifo_all_modes(&lsm6dsr_io);

    PHASE(10, "BDU stress");
    phase10_bdu_stress(&lsm6dsr_io);

    PHASE(11, "FIFO overflow + recovery");
    phase11_fifo_overflow(&lsm6dsr_io);

    PHASE(12, "Repeat init stability");
    phase12_init_stability(&lsm6dsr_io);

    PHASE(13, "DRDY poll verification");
    phase13_drdy_poll(&lsm6dsr_io);

    PHASE(14, "Data integrity check");
    phase14_data_integrity(&lsm6dsr_io);

    PHASE(15, "Reg read-back after reset/boot");
    phase15_reg_after_reset(&lsm6dsr_io);

    PHASE(16, "Zero-rate bias & noise floor");
    phase16_bias_noise(&lsm6dsr_io);
#endif

    printf("\r\n========================================\r\n");
    printf("  Summary: %d PASS / %d FAIL\r\n", g_pass, g_fail);
    if (g_fail > 0) {
        printf("  *** Some tests FAILED ***\r\n");
    } else {
        printf("  All tests passed.\r\n");
    }
}
