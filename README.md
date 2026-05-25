# LSM6DSR_F407_TEST

STM32F407VET6 + LSM6DSR (I2C) IMU 姿态估计工程，面向机器狗 (robot dog) 等强振动、强线性加速度场景。

## 硬件

| 组件 | 型号 | 说明 |
|------|------|------|
| MCU | STM32F407VET6 @168MHz | LQFP100 |
| IMU | ST LSM6DSR | 6-axis (accel ±4G / gyro ±250dps) |
| I2C | PB6=SCL, PB7=SDA | 400kHz Fast Mode |
| UART | PA9=TX, PA10=RX | 115200 8N1 |

## 三层架构

```
┌─ main.c ──────────────────────────────┐
│  HAL_Init → MX_* → bsp_init → loop:   │
│  bsp_update → vofa_format → UART IT   │
├───────────────────────────────────────┤
│  test_lsm6dsr.c/h     测试层          │
│  - 定义 lsm6dsr_io I/O 实例           │
│  - stm32_i2c_read/write I2C 桥接函数  │
│  - P1~P19 合格性 + 性能测试           │
├───────────────────────────────────────┤
│  bsp_lsm6dsr.c/h     业务层 (BSP)    │
│  - 互补滤波器 (pitch/roll/yaw)       │
│  - 自适应 α (运动0.99 / 静止0.30)    │
│  - 三重静止检测 (方差+幅值+陀螺)      │
│  - Runtime 陀螺偏置跟踪               │
│  - DWT 周期计数器计时 (~6ns)         │
│  - 生产 API (get_data/is_stationary/  │
│    vofa_format/calibrate)            │
├───────────────────────────────────────┤
│  lsm6dsr.c/h         驱动层           │
│  - lsm6dsr_io_t I/O 抽象层            │
│  - 单/多字节寄存器读写                │
│  - ACC/GYRO/TEMP 数据读取             │
│  - FIFO 全部模式操作                  │
│  - 自检 + 功耗模式控制                │
└───────────────────────────────────────┘
```

## 文件说明

### 驱动层 (Driver)

| 文件 | 作用 |
|------|------|
| `Core/Inc/lsm6dsr.h` | 寄存器映射、I/O 抽象 `lsm6dsr_io_t` 、枚举、灵敏度常量、所有驱动函数原型 |
| `Core/Src/lsm6dsr.c` | 实现——寄存器读写、ACC/GYRO/TEMP 读取、FIFO 控制、自检、功耗模式 |

### 业务层 (BSP)

| 文件 | 作用 |
|------|------|
| `Core/Inc/bsp_lsm6dsr.h` | 可配置宏、数据结构 `bsp_lsm6dsr_data_t` 、生产 API 声明 |
| `Core/Src/bsp_lsm6dsr.c` | 实现——互补滤波、偏置校准+跟踪、三重静止检测、DWT 计时、VOFA+ 格式化 |

### 测试层 (Test)

| 文件 | 作用 |
|------|------|
| `Core/Inc/test_lsm6dsr.h` | P1~P19 测试函数声明 + PASS/FAIL 宏 |
| `Core/Src/test_lsm6dsr.c` | I2C 桥接函数、`lsm6dsr_io` 实例定义、19 项传感器合格性+性能测试 |

### 应用层

| 文件 | 作用 |
|------|------|
| `Core/Src/main.c` | 生产入口——HAL 初始化、`bsp_lsm6dsr_init()`、VOFA+ 10 通道 IT 输出循环 |
| `Core/Inc/main.h` | STM32 HAL 包含 |

### CubeMX 生成 (不手动编辑)

| 文件 | 作用 |
|------|------|
| `Core/Src/i2c.c` | I2C1 初始化 (400kHz Fast Mode) |
| `Core/Src/usart.c` | USART1 初始化 (115200 8N1) |
| `Core/Src/gpio.c` | GPIO 初始化 |
| `Core/Src/stm32f4xx_it.c` | 中断服务 (USART1_IRQHandler) |
| `Core/Src/stm32f4xx_hal_msp.c` | HAL MSP 初始化 |
| `Core/Src/system_stm32f4xx.c` | 系统时钟配置 |
| `Core/Inc/stm32f4xx_hal_conf.h` | HAL 库配置 |

### 其他

| 文件 | 作用 |
|------|------|
| `docs/bsp_tuning_guide.md` | BSP 参数调参与 VOFA+ 使用指南 (中文) |
| `docs/debug_history.md` | 调试历史与已修复问题记录 (中文) |
| `lsm6dsr_STdC/` | ST 官方 LSM6DSR Standard C 驱动 (参考) |
| `LSM6DSR_F407_TEST.ioc` | CubeMX 工程配置 |
| `ST-LSM6DSR.pdf` | LSM6DSR 数据手册 |

## 快速开始

### 1. 编译烧录

1. 用 Keil MDK-ARM 打开 `MDK-ARM/LSM6DSR_F407_TEST.uvprojx`
2. 配置 Debug 为你的调试器 (ST-Link / J-Link)
3. Rebuild → Download

### 2. 串口输出

出厂配置：**115200 8N1**

复位后打印初始化信息：

```
  I2C: DevAddr=0x00D4  WHO_AM_I=0x6B
  ACC ODR=104Hz FS=4G  GYRO ODR=104Hz FS=250dps
  Initial pitch=-17.25  roll=86.10
  Calibrating gyro bias (100 samples, keep still)...
  Gyro bias: X=0.3021  Y=-0.2506  Z=0.1091 dps  (100/100 OK)
  GYRO residual after cal: X=0.0304  Y=0.1894  Z=-0.2666 dps
  BSP init done  (cal=OK, bias=0.3021,-0.2506,0.1091)  alpha=0.30
--- VOFA+ Live Data (10ch: ax,ay,az,gx,gy,gz,pitch,roll,yaw,temp) ---
```

### 3. VOFA+ 上位机

1. VOFA+ → Protocol: **FireWater** (逗号分隔换行)
2. Channel 0-9 依次为：ax, ay, az, gx, gy, gz, pitch, roll, yaw, temp
3. 波特率 115200，对应 COM 口

## API 参考 (BSP 层)

### `void bsp_lsm6dsr_init(void)`

传感器初始化：复位 → I3C 禁用 → IF_INC+BDU 使能 → ACC 104Hz/4G → GYRO 104Hz/250dps → 滤波状态初始化 → DWT 计时器使能 → 执行校准

### `void bsp_lsm6dsr_calibrate(void)`

陀螺零偏校准：采集 100 帧，用 ACC 静止检测（幅值+帧间差分）拒斥运动帧，有效帧 ≥50 时取均值。

可在运行时重新调用（机器狗站定时重新校准）。

### `void bsp_lsm6dsr_update(bsp_lsm6dsr_data_t *data)`

核心滤波循环：
1. DWT 计时 → dt
2. 读取 ACC/GYRO/TEMP
3. 方差滑动窗口静止检测
4. ACC 幅值静止二次确认
5. GYRO 幅值静止三次确认
6. Runtime 偏置跟踪（仅静止时）
7. 自适应 α 平滑过渡
8. 互补滤波器 (pitch/roll/yaw)
9. 填充 data 结构体 + 缓存

### `const bsp_lsm6dsr_data_t* bsp_lsm6dsr_get_data(void)`

返回指向 `last_data` 缓存的只读指针。适用于从不同上下文获取最新姿态。

### `int bsp_lsm6dsr_is_stationary(void)`

返回 `1`（静止）或 `0`（运动）。

### `int bsp_lsm6dsr_vofa_format(char *buf, int buf_size, const bsp_lsm6dsr_data_t *data)`

将 data 格式化为 VOFA+ FireWater 10 通道 CSV 字符串。

### `void bsp_lsm6dsr_get_bias(float *bx, float *by, float *bz)`

获取当前陀螺零偏值 (dps)。

### `float bsp_lsm6dsr_get_last_variance(void)`

获取上一帧 ACC 方差总和（用于调试静止检测门限）。

## 配置宏

所有宏在 `bsp_lsm6dsr.h` 中用 `#ifndef` 定义，可通过编译器 `-D` 参数覆盖。

### 校准参数

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BSP_CALIB_SAMPLES` | 100 | 校准采样帧数 |
| `BSP_CALIB_SETTLE_MS` | 50 | 配置后稳定等待 (ms) |
| `BSP_CALIB_ACC_MAG_REF` | 1000000.0 | ACC 幅值参考 (mg²) = 1G |
| `BSP_CALIB_ACC_MAG_TOL` | 65000.0 | 幅值容差 (±255mg) |
| `BSP_CALIB_ACC_DELTA_MAX` | 80.0 | 帧间差分阈值 (mg) |
| `BSP_CALIB_SAMPLE_DELAY_MS` | 9 | 采样间隔 (ms) |

### 滤波器参数

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BSP_ACC_VAR_WINDOW` | 10 | 方差滑动窗口大小 (帧) |
| `BSP_ACC_VAR_THRESHOLD` | 800.0 | 静止方差阈值 (mg²总和) |
| `BSP_ALPHA_MOVING` | 0.99 | 运动时互补滤波 α |
| `BSP_ALPHA_STATIONARY` | 0.30 | 静止时互补滤波 α |
| `BSP_ALPHA_SMOOTH_STEP` | 0.15 | α 每帧最大变化 |

### 偏置跟踪参数

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BSP_BIAS_STATIONARY_RATE` | 0.05 | X/Y 轴静止偏置跟踪速率 |
| `BSP_BIAS_STATIONARY_RATE_Z` | 0.005 | Z 轴静止偏置跟踪速率 (慢10倍，防A→B→A误差) |
| `BSP_GYRO_MOTION_THRESHOLD` | 5.0 | 陀螺幅值运动阈值 (dps) |

## 引脚映射

| 引脚 | 功能 |
|------|------|
| PB6 | I2C1_SCL (LSM6DSR) |
| PB7 | I2C1_SDA (LSM6DSR) |
| PA9 | USART1_TX (VOFA+) |
| PA10 | USART1_RX |
| PA13 | SWDIO |
| PA14 | SWCLK |
| PH0 | HSE 8MHz |
| PH1 | HSE 8MHz |
