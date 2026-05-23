# LSM6DSR 硬件接线说明

## 平台信息

| 项目 | 值 |
|------|-----|
| MCU | STM32F407VET6（梁山派天空星） |
| 传感器 | LSM6DSR（LGA-14L 封装） |
| 通信接口 | I²C1（硬件 I²C），Mode 1 |
| 调试输出 | USART1（115200 baud 8N1） |

---

## LSM6DSR 引脚定义（LGA-14L）

数据手册 Table 1，Mode 1 为本项目使用模式：

| 引脚# | 名称 | Mode 1 功能（本项目） | Mode 2 功能 | Mode 3/4 功能 |
|:---:|:---:|:---|:---|:---|
| 1 | **SDO/SA0** | SPI SDO / **I²C 地址 LSB** | 同 Mode 1 | 同 Mode 1 |
| 2 | **SDx** | **接 Vdd_IO 或 GND** | I²C 主机数据 (MSDA) | Aux SPI SDI/SDO |
| 3 | **SCx** | **接 Vdd_IO 或 GND** | I²C 主机时钟 (MSCL) | Aux SPI SPC |
| 4 | **INT1** | 可编程中断 1 | 同 Mode 1 | 同 Mode 1 |
| 5 | **Vdd_IO** | IO 电源 (1.62–3.6V) | 同 Mode 1 | 同 Mode 1 |
| 6 | **GND** | 地 | 同 Mode 1 | 同 Mode 1 |
| 7 | **GND** | 地 | 同 Mode 1 | 同 Mode 1 |
| 8 | **Vdd** | 电源 (1.71–3.6V) | 同 Mode 1 | 同 Mode 1 |
| 9 | **INT2** | 可编程中断 2 / DEN | 同 + MDRDY | 同 Mode 1 |
| 10 | **OCS_Aux** | 悬空 (NC) | 悬空 (NC) | Aux SPI 使能 |
| 11 | **SDO_Aux** | 悬空 (NC) | 悬空 (NC) | Aux SPI SDO |
| 12 | **CS** | **I²C/SPI 模式选择** | 同 Mode 1 | 同 Mode 1 |
| 13 | **SCL / SPC** | **I²C 时钟 (SCL)** / SPI 时钟 | 同 Mode 1 | 同 Mode 1 |
| 14 | **SDA / SDI** | **I²C 数据 (SDA)** / SPI 数据输入 | 同 Mode 1 | 同 Mode 1 |

> 引脚 2 (SDx) 和 3 (SCx) 是 **Mode 2 专用的 I²C 主机接口**，本项目不涉及。

---

## 接线表

### LSM6DSR ↔ STM32F407

| LSM6DSR 引脚# | 名称 | 连接目标 | 说明 |
|:---:|:---:|:---|---:|
| 14 | SDA | PB7 (I2C1_SDA) | I²C 数据线 |
| 13 | SCL | PB6 (I2C1_SCL) | I²C 时钟线，400kHz |
| 1 | SDO/SA0 | **GND** | 7-bit 地址 = **0x6A** |
| 12 | CS | **3.3V** | **必须高电平开启 I²C 模式** |
| 2 | SDx | **3.3V** | Mode 1 无用，接高电平 |
| 3 | SCx | **3.3V** | Mode 1 无用，接高电平 |
| 8 | Vdd | 3.3V | 主电源 |
| 5 | Vdd_IO | 3.3V | IO 电源 |
| 6, 7 | GND | GND | 地 |
| 4 | INT1 | NC | 悬空（本驱动未用中断） |
| 9 | INT2 | NC | 悬空 |
| 10 | OCS_Aux | NC | 悬空 |
| 11 | SDO_Aux | NC | 悬空 |

### 串口调试（USART1）

| STM32F407 引脚 | 功能 | 连接设备 |
|:---:|:---:|:---|
| PA9 (USART1_TX) | 发送 | USB 转串口 RX |
| PA10 (USART1_RX) | 接收 | USB 转串口 TX |
| GND | 地 | USB 转串口 GND |

---

## 接线示意图（Mode 1 I²C）

```
LSM6DSR (LGA-14L 底视图)          STM32F407VET6 (梁山派天空星)
┌───────────────────┐             ┌───────────────────────┐
│(1) SDO/SA0 ── GND │             │ 3.3V                  │
│(2) SDx      ── 3.3V│             │ GND                   │
│(3) SCx      ── 3.3V│             │ PB6 (I2C1_SCL) ←── SCL(13) │
│(4) INT1     ── NC  │             │ PB7 (I2C1_SDA) ←── SDA(14) │
│(5) Vdd_IO   ── 3.3V│             │                       │
│(6) GND      ── GND │             │ PA9(TX) ───→ USB串口RX│
│(7) GND      ── GND │             │ PA10(RX) ←── USB串口TX│
│(8) Vdd      ── 3.3V│             │                       │
│(9) INT2     ── NC  │             │                       │
│(10) OCS_Aux ── NC  │             │                       │
│(11) SDO_Aux ── NC  │             │                       │
│(12) CS      ── 3.3V│ ← I²C模式!  │                       │
│(13) SCL     ── PB6 │             │                       │
│(14) SDA     ── PB7 │             │                       │
└───────────────────┘             └───────────────────────┘
```

---

## CS 与 I²C / SPI 模式

| CS 电平 | 模式 | SCL (13) | SDA (14) |
|:---:|:---:|:---:|:---:|
| **3.3V** | **I²C** | SCL（I²C 时钟） | SDA（I²C 数据） |
| GND | SPI | SPC（SPI 时钟） | SDI（SPI 数据输入） |

- **CS=高** → 当前配置。I²C 模式，SCL/SDA 连接 MCU 的 I²C 外设
- **CS=低** → SPI 模式，SCL 变 SPC，SDA 变 SDI，此时 I²C 通信**禁用**

---

## I²C 地址配置

| SDO/SA0 (Pin 1) | 7-bit 地址 | 8-bit 写地址 | 8-bit 读地址 |
|:---:|:---:|:---:|:---:|
| **GND** | **0x6A** | **0xD4** | **0xD5** |
| 3.3V | 0x6B | 0xD6 | 0xD7 |

本项目 SDO/SA0 接 GND，地址 = **0x6A**。

---

## 检查清单

- [ ] **CS (Pin 12) 接 3.3V**（最常见问题！不接则 I²C 不通）
- [ ] SDA (Pin 14) ↔ PB7，SCL (Pin 13) ↔ PB6
- [ ] SDO/SA0 (Pin 1) 接 GND（地址 0x6A）
- [ ] SDx (Pin 2) 接 3.3V
- [ ] SCx (Pin 3) 接 3.3V
- [ ] Vdd (Pin 8) 和 Vdd_IO (Pin 5) 均已接 3.3V
- [ ] GND (Pin 6, 7) 共地
- [ ] 未用引脚（INT1, INT2, OCS_Aux, SDO_Aux）保持悬空
- [ ] USART1 PA9(TX) 连接 USB 转串口的 RX
- [ ] 串口助手设置 115200, 8N1

---

## 常见问题排查

| 现象 | 可能原因 |
|------|----------|
| WHO_AM_I 返回 0xFF 或 0x00 | CS 未接高电平，进入了 SPI 模式 |
| I²C 总线卡死（SCL 被拉低） | 接线错误或传感器未上电 |
| 加速度计数值全为 0 | BDU 未设置，读取时高低字节未对齐 |
| 陀螺仪有数据但不变化 | ODR 配置错误或传感器静止（静止时陀螺应接近 0） |
| printf 不输出或输出乱码 | 波特率不匹配或 USART1 引脚连接错误 |
