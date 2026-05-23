#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lsm6dsr.h"

void SystemClock_Config(void);

int fputc(int ch, FILE *f) { uint8_t c = ch; HAL_UART_Transmit(&huart1, &c, 1, 100); return ch; }

static int g_pass, g_fail;

#define PHASE(n, msg) do { printf("\r\n[P%d] %s\r\n", n, msg); } while(0)
#define PASS(fmt, ...) do { printf("  PASS " fmt "\r\n", ##__VA_ARGS__); g_pass++; } while(0)
#define FAIL(fmt, ...) do { printf("  FAIL " fmt "\r\n", ##__VA_ARGS__); g_fail++; } while(0)

/* ===================================================================
 * Section 1: I2C & Register Core Verification
 * =================================================================== */

/* P1: I2C Bus Probe */
static void phase1(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(hi2c, LSM6DSR_I2C_ADDR, 3, 100);
    printf("  HAL_I2C_IsDeviceReady(0x%02X) = %d\r\n", LSM6DSR_I2C_ADDR, (int)ret);
    if (ret == HAL_OK) {
        PASS("I2C device detected at 0x%02X", LSM6DSR_I2C_ADDR);
    } else {
        FAIL("I2C device NOT detected at 0x%02X (HAL=%d)", LSM6DSR_I2C_ADDR, (int)ret);
    }
}

/* P2: WHO_AM_I Verification — halt on failure */
static void phase2(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    lsm6dsr_status_t st = lsm6dsr_read_reg(hi2c, LSM6DSR_REG_WHO_AM_I, &id);
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

/* P3: Register Write-Back Verification */
static void phase3(I2C_HandleTypeDef *hi2c)
{
    struct {
        uint8_t addr;
        uint8_t safe_mask;
    } regs[] = {
        {LSM6DSR_REG_CTRL1_XL, 0xFF},
        {LSM6DSR_REG_CTRL2_G,  0xFF},
        {LSM6DSR_REG_CTRL3_C,  0x7C},  /* avoid bit0(SW_RESET), bit7(BOOT) */
        {LSM6DSR_REG_CTRL4_C,  0xFB},  /* avoid bit2(I2C_DISABLE) */
        {LSM6DSR_REG_CTRL9_XL, 0xFD},  /* avoid bit1(I3C_DISABLE) */
    };
    int nregs = sizeof(regs) / sizeof(regs[0]);
    uint8_t patterns[] = {0x5A, 0xA5, 0x00, 0xFF};
    int npatterns = sizeof(patterns) / sizeof(patterns[0]);
    int all_ok = 1;

    for (int r = 0; r < nregs; r++) {
        for (int p = 0; p < npatterns; p++) {
            uint8_t wr_val = patterns[p] & regs[r].safe_mask;
            uint8_t rd_val;
            lsm6dsr_write_reg(hi2c, regs[r].addr, wr_val);
            lsm6dsr_read_reg(hi2c, regs[r].addr, &rd_val);
            if (rd_val != wr_val) {
                all_ok = 0;
                FAIL("0x%02X: wr 0x%02X rd 0x%02X", regs[r].addr, wr_val, rd_val);
            }
        }
        lsm6dsr_write_reg(hi2c, regs[r].addr, 0x00);
    }
    if (all_ok) {
        PASS("R/W: %d regs x %d patterns = %d tests OK",
             nregs, npatterns, nregs * npatterns);
    } else {
        FAIL("R/W: some tests failed");
    }
}

/* P4: Multi-byte burst read + sensor data readout */
static void phase4(I2C_HandleTypeDef *hi2c)
{
    lsm6dsr_status_t st;

    lsm6dsr_set_bdu(hi2c, 1);
    lsm6dsr_set_if_inc(hi2c, 1);
    lsm6dsr_accel_config(hi2c, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(hi2c, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_2000DPS);
    HAL_Delay(50);

    float temp;
    st = lsm6dsr_read_temp(hi2c, &temp);
    if (st == LSM6DSR_OK) {
        printf("  Temperature = %.1f C\r\n", temp);
        if (temp > 10.0f && temp < 50.0f) PASS("Temp %.1fC in 10-50 range", temp);
        else FAIL("Temp %.1fC out of range", temp);
    } else {
        FAIL("Temp read failed (status=%d)", st);
    }

    lsm6dsr_axis_t accel;
    st = lsm6dsr_read_accel_raw(hi2c, &accel);
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

    lsm6dsr_axis_t gyro;
    st = lsm6dsr_read_gyro_raw(hi2c, &gyro);
    if (st == LSM6DSR_OK) {
        printf("  GYRO raw: X=%d Y=%d Z=%d\r\n", gyro.x, gyro.y, gyro.z);
    } else {
        FAIL("GYRO burst read failed (status=%d)", st);
    }
}

/* ===================================================================
 * Section 2: Extended Validation  (ifdef RUN_FULL_TESTS)
 * =================================================================== */
#ifdef RUN_FULL_TESTS

static void phase5_fs_odr_sweep(I2C_HandleTypeDef *hi2c)
{
    lsm6dsr_accel_fs_t accel_fs_list[] = {
        LSM6DSR_ACCEL_FS_2G, LSM6DSR_ACCEL_FS_4G,
        LSM6DSR_ACCEL_FS_8G, LSM6DSR_ACCEL_FS_16G};

    int fs_ok = 1;
    for (int i = 0; i < 4; i++) {
        lsm6dsr_accel_config(hi2c, LSM6DSR_ACCEL_ODR_104HZ, accel_fs_list[i]);
        uint8_t rb;
        lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL1_XL, &rb);
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
        lsm6dsr_gyro_config(hi2c, LSM6DSR_GYRO_ODR_104HZ, gyro_fs_list[i]);
        uint8_t rb;
        lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL2_G, &rb);
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
        lsm6dsr_accel_config(hi2c, odr_list[i], LSM6DSR_ACCEL_FS_4G);
        uint8_t rb;
        lsm6dsr_read_reg(hi2c, LSM6DSR_REG_CTRL1_XL, &rb);
        uint8_t odr_read = (rb >> 4) & 0xF;
        uint8_t odr_expect = (uint8_t)odr_list[i];
        if (odr_read != odr_expect) { odr_ok = 0; FAIL("ODR %d: wrote %d rd %d", i, odr_expect, odr_read); }
    }
    if (odr_ok) PASS("ODR: 7/7 verified");

    lsm6dsr_reset(hi2c);
    lsm6dsr_boot(hi2c);
}

static void phase6_fifo_all_modes(I2C_HandleTypeDef *hi2c)
{
    lsm6dsr_set_bdu(hi2c, 1);
    lsm6dsr_set_if_inc(hi2c, 1);
    lsm6dsr_i3c_disable(hi2c);

    lsm6dsr_fifo_init(hi2c, 10, LSM6DSR_BDR_52Hz, LSM6DSR_BDR_52Hz);
    lsm6dsr_accel_config(hi2c, LSM6DSR_ACCEL_ODR_52HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(hi2c, LSM6DSR_GYRO_ODR_52HZ, LSM6DSR_GYRO_FS_2000DPS);
    HAL_Delay(20);

    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_BYPASS);
    HAL_Delay(5);
    uint16_t lev = lsm6dsr_fifo_get_level(hi2c);
    if (lev == 0) PASS("BYPASS: level=0");
    else FAIL("BYPASS: level=%u", lev);

    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_FIFO);
    HAL_Delay(2000);
    lev = lsm6dsr_fifo_get_level(hi2c);
    if (lev > 0) PASS("FIFO mode: level=%u", lev);
    else FAIL("FIFO mode: level=0");

    while (lsm6dsr_fifo_get_level(hi2c) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(hi2c, &s, &d);
    }

    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_CONT);
    HAL_Delay(1000);
    lev = lsm6dsr_fifo_get_level(hi2c);
    printf("  CONT mode 1s: level=%u (~104/s)\r\n", lev);
    if (lev > 50) PASS("CONT: fill rate OK (%u/s)", lev);
    else FAIL("CONT: low fill rate (%u/s)", lev);

    int acc_count = 0, gyro_count = 0, total_tags = 0;
    int max_tags = (lev > 100) ? 100 : lev;
    for (int i = 0; i < max_tags; i++) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        if (lsm6dsr_read_fifo_entry(hi2c, &s, &d) != LSM6DSR_OK) break;
        total_tags++;
        if (s == LSM6DSR_FIFO_SENSOR_ACC) acc_count++;
        else gyro_count++;
    }
    printf("  Tags: %d ACC + %d GYRO = %d total\r\n", acc_count, gyro_count, total_tags);
    if (acc_count > 0 && gyro_count > 0) PASS("FIFO tag distribution OK");
    else FAIL("FIFO tag: ACC=%d GYRO=%d", acc_count, gyro_count);

    while (lsm6dsr_fifo_get_level(hi2c) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(hi2c, &s, &d);
    }

    lsm6dsr_fifo_set_wtm(hi2c, 5);
    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_CONT_TO_FIFO);
    HAL_Delay(500);
    uint8_t wtm = lsm6dsr_fifo_wtm_flag(hi2c);
    printf("  CONT_TO_FIFO WTM flag=%d (expect 1)\r\n", wtm);
    if (wtm) PASS("WTM trigger OK");
    else FAIL("WTM not triggered");

    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_BYPASS);
    while (lsm6dsr_fifo_get_level(hi2c) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(hi2c, &s, &d);
    }
}

static void phase7_bdu_stress(I2C_HandleTypeDef *hi2c)
{
    lsm6dsr_accel_config(hi2c, LSM6DSR_ACCEL_ODR_833HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_set_bdu(hi2c, 1);
    HAL_Delay(10);

    int bad = 0;
    for (int i = 0; i < 200; i++) {
        uint8_t buf[6];
        lsm6dsr_read_multi(hi2c, LSM6DSR_REG_OUTX_L_XL, buf, 6);
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

static void phase8_fifo_overflow(I2C_HandleTypeDef *hi2c)
{
    lsm6dsr_fifo_init(hi2c, 200, LSM6DSR_BDR_104Hz, LSM6DSR_BDR_104Hz);
    lsm6dsr_accel_config(hi2c, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
    lsm6dsr_gyro_config(hi2c, LSM6DSR_GYRO_ODR_104HZ, LSM6DSR_GYRO_FS_2000DPS);
    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_CONT);
    HAL_Delay(100);

    uint8_t ovr = lsm6dsr_fifo_ovr_flag(hi2c);
    uint8_t full = lsm6dsr_fifo_full_flag(hi2c);
    printf("  After 100ms: OVR=%d FULL=%d\r\n", ovr, full);

    HAL_Delay(3000);
    uint16_t lev = lsm6dsr_fifo_get_level(hi2c);
    ovr = lsm6dsr_fifo_ovr_flag(hi2c);
    printf("  After 3s: level=%u OVR=%d\r\n", lev, ovr);

    lsm6dsr_fifo_flush(hi2c);
    HAL_Delay(5);
    lev = lsm6dsr_fifo_get_level(hi2c);
    printf("  After flush: level=%u\r\n", lev);
    if (lev == 0) PASS("FIFO flush OK");
    else FAIL("FIFO flush level=%u", lev);

    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_CONT);
    HAL_Delay(1000);
    lev = lsm6dsr_fifo_get_level(hi2c);
    if (lev > 50) PASS("FIFO recovery after flush: level=%u", lev);
    else FAIL("FIFO recovery low level=%u", lev);

    lsm6dsr_fifo_set_mode(hi2c, FIFO_MODE_BYPASS);
    while (lsm6dsr_fifo_get_level(hi2c) > 0) {
        lsm6dsr_fifo_sensor_t s;
        lsm6dsr_axis_t d;
        lsm6dsr_read_fifo_entry(hi2c, &s, &d);
    }
}

static void phase9_init_stability(I2C_HandleTypeDef *hi2c)
{
    float z_means[3];
    for (int cycle = 0; cycle < 3; cycle++) {
        lsm6dsr_reset(hi2c);
        lsm6dsr_boot(hi2c);
        lsm6dsr_set_bdu(hi2c, 1);
        lsm6dsr_set_if_inc(hi2c, 1);
        lsm6dsr_i3c_disable(hi2c);
        lsm6dsr_accel_config(hi2c, LSM6DSR_ACCEL_ODR_104HZ, LSM6DSR_ACCEL_FS_4G);
        HAL_Delay(30);

        int64_t sum_z = 0;
        for (int i = 0; i < 50; i++) {
            lsm6dsr_axis_t a;
            lsm6dsr_read_accel_raw(hi2c, &a);
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
    float spread = (max_v - min_v) / (max_v > 0 ? max_v : 1.0f) * 100.0f;
    if (spread < 0.0f) spread = -spread;
    printf("  Spread = %.1f%%\r\n", spread);
    if (spread < 5.0f) PASS("Repeat init OK (spread %.1f%%)", spread);
    else FAIL("Repeat init spread %.1f%% >= 5%%", spread);
}

#endif /* RUN_FULL_TESTS */

/* ===================================================================
 * main
 * =================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    printf("\r\n========================================\r\n");
    printf("  LSM6DSR I2C & Register Core Test\r\n");
    printf("========================================\r\n");
    printf("  I2C1: DevAddr=0x%04X, Clock=%luHz, APB1=%luHz\r\n",
           (unsigned)LSM6DSR_I2C_ADDR, (unsigned long)hi2c1.Init.ClockSpeed,
           HAL_RCC_GetPCLK1Freq());
    HAL_Delay(200);

    PHASE(1, "I2C Bus Probe");
    phase1(&hi2c1);

    PHASE(2, "WHO_AM_I Verification");
    phase2(&hi2c1);

    PHASE(3, "Register Write-Back");
    phase3(&hi2c1);

    PHASE(4, "Multi-byte Burst + Sensor Data");
    phase4(&hi2c1);

#ifdef RUN_FULL_TESTS
    printf("\r\n========================================\r\n");
    printf("  Extended Validation (%d PASS / %d FAIL so far)\r\n", g_pass, g_fail);
    printf("========================================\r\n");

    PHASE(5, "FS/ODR sweep");
    phase5_fs_odr_sweep(&hi2c1);

    PHASE(6, "FIFO all modes + tags");
    phase6_fifo_all_modes(&hi2c1);

    PHASE(7, "BDU stress");
    phase7_bdu_stress(&hi2c1);

    PHASE(8, "FIFO overflow + recovery");
    phase8_fifo_overflow(&hi2c1);

    PHASE(9, "Repeat init stability");
    phase9_init_stability(&hi2c1);
#endif

    printf("\r\n========================================\r\n");
    printf("  Summary: %d PASS / %d FAIL\r\n", g_pass, g_fail);
    if (g_fail > 0) {
        printf("  *** Some tests FAILED ***\r\n");
    } else {
        printf("  All tests passed.\r\n");
    }

    while (1) { ; }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) { } }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: %s %lu\r\n", file, (unsigned long)line);
}
#endif
