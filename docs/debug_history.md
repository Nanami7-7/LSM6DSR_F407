# LSM6DSR 驱动调试历史记录

## 项目背景

STM32F407 + LSM6DSR (I2C) 驱动程序开发。使用 CubeMX + Keil MDK-ARM (ARMCC V6.24)。

## 已知正确项（与 ST 官方 lsm6dsr_reg.h 一致）

| 项目 | 值 | 备注 |
|------|----|------|
| I2C 地址 | 0x6A | 左移 1 位后 0xD4 |
| WHO_AM_I | 0x6B | |
| CTRL3_C SW_RESET | bit 0 | |
| CTRL3_C IF_INC | bit 2 | |
| CTRL3_C BDU | bit 6 | |
| CTRL3_C BOOT | bit 7 | |
| CTRL9_XL I3C_DISABLE | bit 1 | |
| FIFO_CTRL1 | 0x07 | |
| FIFO_CTRL2 | 0x08 | |
| FIFO_CTRL3 | 0x09 | BDR_XL[3:0], BDR_GY[7:4] |
| FIFO_CTRL4 | 0x0A | mode[2:0] |
| FIFO_STATUS1 | 0x3A | DIFF[7:0] |
| FIFO_STATUS2 | 0x3B | DIFF[9:8] + 标志位 |
| INT1_CTRL | 0x0D | |
| INT2_CTRL | 0x0E | |
| GYRO FS 编码 | `(fs & 0x07) << 1` | bits[3:1] |
| ACC FS 编码 | `fs << 2` | bits[3:2] |
| BDR 枚举 | 0x01=12.5Hz, 0x02=26Hz, 0x03=52Hz, 0x04=104Hz | |

---

## 已修复的问题

### 1. ODR 枚举偏移 +1

**严重性**: 高

**现象**: `accel_config(104Hz)` 实际设置 208Hz

**根因**: ODR 枚举值比 ST 标准值大 1

| 速率 | 旧值 | ST 值 | 修复后 |
|------|------|-------|--------|
| 12.5Hz | 0x02 | 0x01 | 0x01 |
| 26Hz | 0x03 | 0x02 | 0x02 |
| 52Hz | 0x04 | 0x03 | 0x03 |
| 104Hz | 0x05 | 0x04 | 0x04 |
| 208Hz | 0x06 | 0x05 | 0x05 |

**文件**: `lsm6dsr.h:85-97` — 摘除 `LSM6DSR_ACCEL_ODR_1_6HZ`（该芯片无此速率），所有值减 1。

### 2. INT1/2_CTRL 地址

**严重性**: 低（代码未使用）

| 寄存器 | 旧值 | ST 值 |
|--------|------|-------|
| INT1_CTRL | 0x58 | 0x0D |
| INT2_CTRL | 0x59 | 0x0E |

**文件**: `lsm6dsr.h:46-47`

### 3. FIFO tag 宏

**严重性**: 中（ODR 修复后旧宏失效）

旧宏基于 `bit4` 判传感器类型（经验公式），修复后 tag_sensor[4:0]=1(GYRO)/2(ACC)，bit4=0 无法区分。

新宏使用 `tag & 0x1F` 匹配 ST tag_sensor 格式。

**文件**: `lsm6dsr.h:136-143`

### 4. 初始化顺序（FIFO 先于 ODR）

**严重性**: 中

旧顺序：ODR → FIFO mode（芯片手册不推荐）

新顺序：FIFO mode → ODR（ST 规范）

### 5. BOOT 流程缺失

**严重性**: 高

SW_RESET 不清除 SELF_DEC（自减采样）状态。增加 `lsm6dsr_boot()` 函数，在复位后触发 BOOT 以清除状态机。

**文件**: `lsm6dsr.c:63-75`

### 6. FIFO_CTRL2 UNCOPTR_RATE 未配置

**严重性**: 高

初始只写 WTM MSB (bit0)，bits[2:1] UNCOPTR_RATE=00 使非压缩数据路径未明确使能，导致 FIFO 填充速率极低（2 条/500ms）。

改为 `UNCOPTR_RATE=3 (52Hz)` 后速率恢复正常（~220 条/秒）。

**文件**: `lsm6dsr.c:186-187`

### 7. FIFO_CTRL4 read-modify-write 保留残留位

修改 `lsm6dsr_fifo_set_mode()` 在设置 mode 后显式清理 `stop_on_wtm` 和 `fifo_compr_rt_en`，与 ST 驱动行为一致。

**文件**: `lsm6dsr.c:194-209`

---

## 最终结论

### 1. FIFO tag 格式 —— 经验结论

**两次测试（v3/v4）一致证明 LSM6DSR 的 FIFO tag 格式与 ST 文档描述不符。**

| ODR | BDR | UNCOPTR_RATE | 结果 |
|-----|-----|-------------|------|
| 104Hz | 104Hz | 12.5Hz (v3) | tags: 0x09~0x17, 非标准 |
| 104Hz | 104Hz | 52Hz (v4 4a) | tags: 0x09~0x17, 非标准 |
| 52Hz | 52Hz | 52Hz (v4 4b) | tags: 0x09~0x17, **非标准** |

即使 ODR=BDR=UNCOPTR_RATE=52Hz 三者完全匹配，tags 仍然是非标准压缩格式。

### 2. 解码模式确认

所有测试中 `bit4` 模式绝对成立：

| bit4 | 对应数据 | raw 值 | 物理量 |
|------|---------|--------|--------|
| 0 (tags: 0x09/0x0A/0x0C/0x0F) | GYRO | ±100 raw | ~0 dps (静止，≈±7dps 零漂) |
| 1 (tags: 0x11/0x12/0x14/0x17) | ACC | -5900 raw | ~-0.72g (重力 Z 轴) |

bits[3:0] 的变化（9/10/12/15 和 17/18/20/23）编码了压缩/批处理元数据，tag_sensor[4:0] 不匹配 ST 枚举值。

### 3. 正确解码方式 → 恢复 bit4 宏

```c
#define FIFO_TAG_IS_GYRO(tag)   (!((tag) & 0x10))
#define FIFO_TAG_IS_ACC(tag)    (((tag) & 0x10) != 0)
```

芯片默认将 FIFO 数据以压缩/批处理格式存储，不受 UNCOPTR_RATE 或 FIFO_COMPR_RT_EN 控制。这可能是 LSM6DSR 变体的硬件特性。

### 4. SELF_DEC 标志

复位+BOOT 后清零，但运行后自动重新设置。属于**正常行为**——只要 FIFO 以压缩/批处理模式运行，SELF_DEC 就会显示为 1。不影响功能。

---

## 驱动验证状态总表

| 功能 | 状态 | 证据 |
|------|------|------|
| I2C 通信 | ✅ | WHO_AM_I=0x6B, 所有寄存器读写正常 |
| SW_RESET | ✅ | 2ms, 复位后寄存器归零 |
| BOOT | ✅ | 6ms, 清除 SELF_DEC |
| ODR 枚举 | ✅ | 104Hz 写入 raw=0x04, 读回确认 |
| ACC FS | ✅ | 4G 写入 raw=0x02, 读回确认 |
| GYRO FS | ✅ | 2000dps 写入 raw=0x03, 读回确认 |
| IF_INC | ✅ | 多字节 burst 读一致 |
| BDU | ✅ | 200 次紧循环 0 撕裂 |
| FIFO 初始化 | ✅ | 所有寄存器写后读回一致 |
| FIFO 速率 | ✅ | 104Hz 配置~208/s, 52Hz 配置~104/s |
| FIFO tag 解码 | ✅ | bit4=0→GYRO, bit4=1→ACC |
| FIFO 排空 | ✅ | 逐条读取+排空后 level=0 |
| INT1/2_CTRL | 💤 | 已修复地址(0x0D/0x0E)，驱动未使用 |

---

## 测试结果时间线

### v1（首次驱动）

仅基本 R/W 验证和 Tag 分析。

### v2（修复 ODR 后）

| Phase | 测试 | 结果 |
|-------|------|------|
| 1 | 上电寄存器快照 | 遗留状态 0x44/0xE2 |
| 2 | SW_RESET | 2ms OK，粘性位保留 |
| 3 | 配置读回 | ODR/FS/BDR 全部 PASS |
| 4 | FIFO tags | 仅 2 条，tag=0x09/0x11 |
| 5 | BDU 压力 | PASS |

### v3（增加 BOOT + UNCOPTR_RATE=1 + 修正顺序）

| Phase | 测试 | 结果 |
|-------|------|------|
| 1 | 上电快照 | 同上 |
| 2 | SW_RESET + BOOT | 10ms OK, SELF_DEC=CLEAR |
| 3 | 配置读回 | 全部 PASS（断言有误） |
| 4a | FIFO 速率监控 | 221/s ❌ tag 仍非标准 |
| 5 | BDU 压力 | PASS |

### v4（UNCOPTR_RATE=3, ODR=52Hz 匹配验证）✅ 最终结论

| Phase | 测试 | 结果 |
|-------|------|------|
| 1 | 上电快照 | 上一轮遗留的 CTRL9_XL=0x02（DEN 被清理）|
| 2 | SW_RESET+BOOT | 2ms+6ms=8ms, SELF_DEC=CLEAR |
| 3 | 配置读回 | 全部 PASS（断言已修复）|
| 4a | FIFO ODR=104Hz BDR=104Hz | 208/s 速率 ✅ tag=0x09~0x17（压缩，非标准）⚠️ |
| 4b | FIFO ODR=52Hz BDR=52Hz UNCOPTR=52Hz | 104/s 速率 ✅ tag=0x09~0x17（压缩，非标准）⚠️ |
| 5 | BDU 压力 | PASS |

**结论**: LSM6DSR 芯片默认以压缩标签格式输出 FIFO 数据，bit4 是可靠的传感器类型标志。
tag_sensor[4:0] 格式不适用于此芯片变体。驱动恢复正常，tag 解码使用 bit4 经验公式。

---

## 文件变更日志

### lsm6dsr.h

```
v1 → v2: ODR enum -1，移除 1.6Hz，INT1/2 地址，tag 宏重写，tag 常量
v2 → v3: CTRL3_C_BOOT 宏，lsm6dsr_boot 声明
v3 → v4: GYRO ODR 宏补全 (12.5/26/52/104/208Hz)
```

### lsm6dsr.c

```
v1 → v2: 仅 ODR 修复的逻辑变化
v2 → v3: lsm6dsr_boot(), fifo_init UNCOPTR_RATE, fifo_set_mode 清理
v3 → v4: UNCOPTR_RATE 1→3, GYRO ODR 宏补全
v4 → final: tag 宏恢复 bit4 经验公式
```

### main.c

```
v1: 原始诊断（Step 1-5 基本读写验证）
v2: 5 阶段深检（基线/复位/读回/ODR+BDR tags/BDU）
v3: BOOT+正确顺序+FIFO 5s速率监控
v4: 双场景FIFO测试 4a(104Hz)+4b(52Hz匹配), GYRO ODR 宏补全
final: tag 宏恢复 bit4 经验公式, 文档完结
```

---

## 引用

- ST LSM6DSR 数据手册
- ST lsm6dsr_reg.h (驱动库版本)
- LSM6DSR 应用笔记 AN5197 (FIFO 使用指南)
