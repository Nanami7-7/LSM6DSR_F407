#ifndef TEST_LSM6DSR_H
#define TEST_LSM6DSR_H

#include "lsm6dsr.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PHASE(n, msg) do { printf("\r\n[P%d] %s\r\n", n, msg); } while(0)
#define PASS(fmt, ...) do { printf("  PASS " fmt "\r\n", ##__VA_ARGS__); g_pass++; } while(0)
#define FAIL(fmt, ...) do { printf("  FAIL " fmt "\r\n", ##__VA_ARGS__); g_fail++; } while(0)
#define RUN_FULL_TESTS

extern int g_pass;
extern int g_fail;
extern lsm6dsr_io_t lsm6dsr_io;
extern volatile uint8_t vofa_tx_busy;

void run_all_tests(void);
void phase1(void);
void phase2(lsm6dsr_io_t *io);
void phase3(lsm6dsr_io_t *io);
void phase4(lsm6dsr_io_t *io);
void phase5_xl_self_test(lsm6dsr_io_t *io);
void phase6_gy_self_test(lsm6dsr_io_t *io);
void phase7_power_mode(lsm6dsr_io_t *io);
void phase8_fs_odr_sweep(lsm6dsr_io_t *io);
void phase9_fifo_all_modes(lsm6dsr_io_t *io);
void phase10_bdu_stress(lsm6dsr_io_t *io);
void phase11_fifo_overflow(lsm6dsr_io_t *io);
void phase12_init_stability(lsm6dsr_io_t *io);
void phase13_drdy_poll(lsm6dsr_io_t *io);
void phase14_data_integrity(lsm6dsr_io_t *io);
void phase15_reg_after_reset(lsm6dsr_io_t *io);
void phase16_bias_noise(lsm6dsr_io_t *io);
void phase17_live_display(lsm6dsr_io_t *io);

#ifdef __cplusplus
}
#endif

#endif /* TEST_LSM6DSR_H */
