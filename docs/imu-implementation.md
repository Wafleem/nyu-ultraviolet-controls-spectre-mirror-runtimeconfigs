# IMU 姿态估计实现文档

## 目录
- [系统概述](#系统概述)
- [硬件配置](#硬件配置)
- [数据单位与坐标系](#数据单位与坐标系)
- [软件架构](#软件架构)
- [校准流程](#校准流程)
- [姿态估计算法](#姿态估计算法)
- [已解决的关键问题](#已解决的关键问题)
- [使用说明](#使用说明)
- [性能指标](#性能指标)

---

## 系统概述

本系统使用 **BMI088 六轴 IMU** 实现高精度姿态估计，采用 **QuaternionEKF（四元数扩展卡尔曼滤波）** 算法融合陀螺仪和加速度计数据。

### 核心特性
- ✅ 六轴姿态融合（陀螺仪 + 加速度计）
- ✅ 动态时间步长自适应
- ✅ 两级零偏校准（静态 + 动态）
- ✅ 开机自动校准
- ✅ Roll/Pitch 长期稳定，Yaw 短期精度高
- ✅ 200Hz 更新频率（实际动态测量）

### 系统限制
- ⚠️ **Yaw 轴会有长期漂移**（6 轴 IMU 固有限制，无磁力计）
- ⚠️ 加速度计不能观测 Yaw 旋转，Z 轴零偏依赖静态校准
- ⚠️ 运动加速度会影响 Roll/Pitch 估计（使用低通滤波缓解）

---

## 硬件配置

### BMI088 IMU
- **芯片**: Bosch BMI088（六轴 IMU）
- **接口**: SPI
  - **加速度计 CS**: PA4 (GPIOA, Pin 4)
  - **陀螺仪 CS**: PB0 (GPIOB, Pin 0)
  - **SPI 总线**: SPI1
- **中断**: 未使用（轮询模式）

### 传感器配置
| 参数 | 配置值 |
|------|--------|
| **加速度计量程** | ±6g |
| **陀螺仪量程** | ±2000 °/s |
| **加速度计采样率** | 800 Hz |
| **陀螺仪采样率** | 1000 Hz |
| **加速度计带宽** | Normal |
| **陀螺仪带宽** | 1000Hz/116Hz (ODR/Filter) |

---

## 数据单位与坐标系

### 原始数据转换

#### 加速度计
```c
// 寄存器配置
BMI088_ACC_RANGE = BMI088_ACC_RANGE_6G  // ±6g 量程

// 灵敏度系数
BMI088_ACCEL_SEN = 0.00179443359375f  // LSB -> m/s²

// 转换公式
accel[i] = raw_data[i] * BMI088_ACCEL_SEN  // 单位: m/s²
```

**关键**: BMI088 加速度计输出的是 **m/s²**，而不是 g！
- 灵敏度 = 9.8 / 5461.33 ≈ 0.001794 m/s²/LSB
- 静止时 Z 轴读数 ≈ 9.8 m/s²

#### 陀螺仪
```c
// 寄存器配置
BMI088_GYRO_RANGE = BMI088_GYRO_2000  // ±2000 °/s 量程

// 灵敏度系数
BMI088_GYRO_SEN = 0.00106526443603169529841533860381f  // LSB -> rad/s

// 转换公式
gyro[i] = raw_data[i] * BMI088_GYRO_SEN  // 单位: rad/s
```

### 坐标系定义

**机体坐标系（Body Frame）**：
```
    X (前)
    ↑
    |
    |___→ Y (右)
   /
  ↙
 Z (下)
```

**欧拉角定义**：
- **Yaw (ψ)**: 绕 Z 轴旋转，水平方向角
- **Pitch (θ)**: 绕 Y 轴旋转，俯仰角
- **Roll (φ)**: 绕 X 轴旋转，横滚角

**旋转顺序**: Yaw → Pitch → Roll (ZYX 顺序)

---

## 软件架构

### 模块层次

```
┌─────────────────────────────────────────┐
│         Application Layer               │
│  (cmd_controller, gimbal, chassis, ...)│
└──────────────┬──────────────────────────┘
               │ subscribe TOPIC_IMU_UPDATE
               ↓
┌─────────────────────────────────────────┐
│         Module Layer                    │
│  ┌─────────────────────────────────┐   │
│  │  gyro_data.c (IMU 处理核心)    │   │
│  │  • gyro_calibrate()             │   │
│  │  • gyro_data_init()             │   │
│  │  • gyro_data_update()           │   │
│  └─────────────────────────────────┘   │
│         ↓               ↓               │
│  ┌──────────┐    ┌──────────────────┐  │
│  │ BMI088   │    │ QuaternionEKF    │  │
│  │ Driver   │    │ (姿态融合)       │  │
│  └──────────┘    └──────────────────┘  │
└──────────────┬──────────────────────────┘
               │
               ↓
┌─────────────────────────────────────────┐
│         Hardware Layer (HAL)            │
│  SPI1, GPIO, DMA, Interrupts            │
└─────────────────────────────────────────┘
```

### 核心文件

| 文件 | 功能 |
|------|------|
| `modules/imu/gyro_data.c/h` | IMU 数据处理、校准、姿态更新 |
| `modules/imu/bmi088driver.c/h` | BMI088 底层驱动（SPI 通信） |
| `modules/algorithm/QuaternionEKF.c/h` | 四元数 EKF 姿态估计算法 |
| `modules/imu/wt61c.c/h` | WT61C 底盘 IMU（备用） |

---

## 校准流程

### 开机自动校准

**时机**: `main.c` 初始化时调用 `gyro_calibrate()`

**要求**:
- 🛑 **IMU 必须静止不动**
- ⏱️ 持续时间 ≈ 2 秒
- 🌡️ 记录校准时温度（用于后续温补，当前未实现）

### 校准算法

```c
// 参数
#define CALIB_SAMPLES 2000    // 2000 个样本
#define CALIB_TIMEOUT_MS 15000  // 15 秒超时
#define MAX_RETRY 3           // 最多重试 3 次

// 校准流程
do {
    // 1. 采集 2000 个样本（2 秒，1ms 间隔）
    for (i = 0; i < 2000; i++) {
        BMI088_read(gyro, accel, &temp);
        g_norm += sqrt(ax² + ay² + az²);  // 累积重力范数
        gyro_offset[x/y/z] += gyro[x/y/z];  // 累积陀螺仪零偏

        // 每 500 个样本检查运动
        if (运动检测到) {
            重新开始本次采样;
        }
    }

    // 2. 计算平均值
    g_norm /= 2000;
    gyro_offset[x/y/z] /= 2000;

    // 3. 检查校准质量
} while (不满足质量要求 && 未超时 && 重试次数 < 3);
```

### 校准质量标准

| 检查项 | 阈值 | 说明 |
|--------|------|------|
| 重力范数变化 | < 0.5 m/s² | `g_norm_max - g_norm_min` |
| 重力范数绝对值 | 9.8±0.5 m/s² | `|g_norm - 9.80665|` |
| 陀螺仪数据跨度 | < 0.15 rad/s | `gyro_max[i] - gyro_min[i]` |
| **陀螺仪零偏** | **< 0.01 rad/s** | `|gyro_offset[i]|` |

### 校准输出

**成功校准后保存**:
```c
static float gyro_offset[3];  // 陀螺仪零偏 (rad/s)
static float accel_scale;     // 加速度计缩放 = 9.80665 / g_norm
static float g_norm;          // 实测重力范数 (m/s²)
```

**调试输出示例**:
```
[BMI088] Sample complete: gNorm=9.857 m/s² (1.01g), offset=[0.002243,-0.001789,0.002247]rad/s
[BMI088] ===== Calibration SUCCESS (attempts: 1) =====
Gyro offset: gx=0.002243, gy=-0.001789, gz=0.002247 rad/s
Accel: gNorm=9.857 m/s² (1.01g)
Temp when cali: 22.50 °C
```

---

## 姿态估计算法

### QuaternionEKF 概述

**算法**: 四元数扩展卡尔曼滤波（Quaternion Extended Kalman Filter）

**状态向量** (6 维):
```
x = [q0, q1, q2, q3, bias_x, bias_y]ᵀ
    └─────┬──────┘  └──────┬───────┘
      四元数(4)      陀螺仪零偏(2)
```

**注意**: Z 轴零偏 `bias_z` **不估计**（恒为 0）
- 原因：加速度计无法观测 Yaw 旋转
- 代码：`QEKF_INS.GyroBias[2] = 0;` (QuaternionEKF.c:199)

### 两级零偏校准

#### Level 1: 静态校准零偏（gyro_offset）
- **时机**: 开机校准时测量
- **应用**: 传给 EKF 前减去
- **覆盖**: X/Y/Z 三轴
- **特点**: 固定值，不随时间变化

```c
gyro_calibrated[i] = gyro[i] - gyro_offset[i];  // 在 gyro_data_update()
```

#### Level 2: 动态估计零偏（EKF.GyroBias）
- **时机**: EKF 运行时在线估计
- **应用**: EKF 内部自动减去
- **覆盖**: X/Y 轴（Z 轴恒为 0）
- **特点**: 动态调整，补偿温漂

```c
// QuaternionEKF.c:114-116
QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];  // X 轴动态估计
QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];  // Y 轴动态估计
QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];  // Z 轴 = 0
```

### 数据流程

```
┌──────────────┐
│ BMI088 原始  │
│ gyro: raw LSB│
│ accel: raw LSB│
└──────┬───────┘
       │ × 灵敏度系数
       ↓
┌──────────────┐
│ 物理单位     │
│ gyro: rad/s  │
│ accel: m/s²  │
└──────┬───────┘
       │ Level 1: 减去静态零偏
       ↓
┌──────────────────┐
│ 静态校准后       │
│ gyro_calibrated  │
│ accel_calibrated │
└──────┬───────────┘
       │ 传给 QuaternionEKF
       ↓
┌──────────────────────┐
│ QuaternionEKF        │
│ • Level 2: 减去动态零偏│
│ • 四元数更新         │
│ • 加速度计修正       │
└──────┬───────────────┘
       │
       ↓
┌──────────────┐
│ 姿态输出     │
│ Yaw/Pitch/Roll│
│ (度)         │
└──────────────┘
```

### EKF 参数配置

```c
IMU_QuaternionEKF_Init(
    init_quaternion,  // 初始四元数（从加速度计计算）
    10.0f,            // process_noise1: 四元数过程噪声
    0.001f,           // process_noise2: 陀螺仪零偏过程噪声
    1000000.0f,       // measure_noise: 加速度计量测噪声
    1.0f,             // lambda: 渐消因子（1 = 无渐消）
    0.0f              // lpf: 低通滤波系数（0 = 不使用）
);
```

**参数说明**:
- `measure_noise = 1000000`: 很大，意味着不太信任加速度计（运动时会有干扰）
- `lambda = 1.0`: 无渐消滤波，标准 EKF
- `lpf = 0`: 不使用低通滤波（可选：0.0085 用于平滑加速度）

### 动态时间步长

**关键修复**: 使用动态测量的 `dt`，而不是固定值。

```c
// 错误做法（会导致角度误差）
#define DT (0.005f)  // 固定 5ms
IMU_QuaternionEKF_Update(..., DT);

// 正确做法（动态测量）
uint32_t current_tick = HAL_GetTick();
dt = (current_tick - last_update_tick) * 0.001f;  // 实际时间间隔
last_update_tick = current_tick;

// 限制范围防止异常
if (dt < 0.0005f) dt = 0.0005f;  // 最小 0.5ms (2000Hz)
if (dt > 0.02f) dt = 0.02f;      // 最大 20ms (50Hz)

IMU_QuaternionEKF_Update(..., dt);  // 使用实际测量值
```

**为什么需要动态 dt?**
- 主循环执行时间不固定（任务处理时间变化）
- 固定 `dt = 5ms`，但实际周期可能是 `7.5ms`
- 角度误差 = 实际时间 / 假设时间 = 7.5 / 5 = 1.5 倍
- 示例：旋转 90°，读数只有 60° (60/90 = 2/3 = 5/7.5)

---

## 已解决的关键问题

### 问题 1: 加速度单位错误 ✅

**现象**: 所有调试信息显示 `accel=[x,y,z]g`，但实际单位是 m/s²。

**根因**:
- BMI088 灵敏度系数已包含 9.8 的转换
- `BMI088_ACCEL_6G_SEN = 0.001794...` (m/s²/LSB)，不是 g/LSB

**修复**:
```c
// 错误注释
accel[i] = raw * BMI088_ACCEL_SEN;  // 单位: g ❌

// 正确注释
accel[i] = raw * BMI088_ACCEL_SEN;  // 单位: m/s² ✅
```

**影响**:
- 误将 m/s² 当作 g，导致传给 EKF 的加速度是实际的 9.8 倍
- 严重影响姿态估计精度

---

### 问题 2: 静态零偏未减去 ✅

**现象**: 静止时 Yaw 轴持续漂移（14 秒漂移 1.3°）。

**根因**:
- 校准时计算了 `gyro_offset`，但从未使用
- QuaternionEKF 内部会减去 `GyroBias`，但 Z 轴恒为 0
- 结果：Z 轴零偏从未被消除

**修复**:
```c
// gyro_data_update()
gyro_calibrated[0] = gyro[0] - gyro_offset[0];
gyro_calibrated[1] = gyro[1] - gyro_offset[1];
gyro_calibrated[2] = gyro[2] - gyro_offset[2];  // Z 轴最重要！

IMU_QuaternionEKF_Update(gyro_calibrated[0], gyro_calibrated[1], gyro_calibrated[2], ...);
```

**为什么 Z 轴最重要?**
- EKF 会将 `GyroBias[2]` 强制设为 0（QuaternionEKF.c:199）
- X/Y 轴有动态估计补偿，Z 轴完全依赖静态校准
- 如果不减去 `gyro_offset[2]`，Yaw 会持续漂移

---

### 问题 3: 校准成功标志未设置 ✅

**现象**: 校准成功后，`accel_scale` 未被计算，使用未初始化的值。

**根因**:
```c
// 错误流程
do {
    // 校准采样...
} while (条件不满足);

if (calibrated) {  // calibrated 从未被设为 1！
    accel_scale = GRAVITY_ACCEL / g_norm;
}
```

**修复**:
```c
} while (条件不满足);

// 如果成功退出循环（不是因为超时/重试），设置成功标志
if (!calibrated && (未超时 && 重试次数 < MAX_RETRY)) {
    calibrated = 1;
}

if (calibrated) {
    accel_scale = GRAVITY_ACCEL / g_norm;  // 现在会执行
    ...
}
```

---

### 问题 4: QuaternionEKF 参数不一致 ✅

**现象**: 某些参数与 basic_framework 不同。

**修复**:
```c
// 错误
IMU_QuaternionEKF_Init(..., 0.9996f, ...);  // lambda = 0.9996 (渐消滤波)

// 正确
IMU_QuaternionEKF_Init(..., 1.0f, ...);     // lambda = 1.0 (标准 EKF)
```

---

### 问题 5: 固定 dt 导致角度误差 ✅ (最严重)

**现象**: 旋转 90°，读数只有 60°，误差比例 = 2/3。

**根因**:
```
假设: dt = 5ms (200Hz)
实际: 主循环周期 = 5ms (HAL_Delay) + 2.5ms (任务) = 7.5ms
角度积分误差 = 实际时间 / 假设时间 = 7.5 / 5 = 1.5 倍

旋转 90° × (5/7.5) = 60° ✅ 与测量一致
```

**修复**: 见"动态时间步长"章节。

**验证**:
- 修复前：旋转 90°，读数 60°
- 修复后：旋转 90°，读数 90°（误差 < 1°）

---

## 使用说明

> **⚠️ 注意**: 本文档中的代码示例基于**早期的裸机实现**。当前系统已迁移到 **FreeRTOS**，部分初始化流程和调用方式有所不同：
>
> **主要区别**:
> - `MsgCenter_Init()` 签名已改变：`MsgCenter_Init(128)` 而非 `MsgCenter_Init(g_msg_queue, MSG_CENTER_QUEUE_LEN)`
> - 不再需要在主循环中调用 `MsgCenter_Dispatch()` - 由专用的 MsgDispatch 任务自动处理
> - `gyro_data_update()` 现在在 Control 任务中调用（200Hz），而非主循环
> - 主循环只需调用 `osKernelStart()` 启动 FreeRTOS 调度器
>
> **完整的 FreeRTOS 架构请参考**:
> - `docs/pub-sub.md` - FreeRTOS 消息中心实现
> - `docs/freertos-architecture.md` - FreeRTOS 任务架构（待创建）
> - `Src/freertos.c` - 实际的任务创建代码

### 初始化流程

```c
// main.c

// 1. 硬件初始化（STM32CubeMX 生成）
HAL_Init();
SystemClock_Config();
MX_SPI1_Init();  // BMI088 使用 SPI1
// ...

// 2. BMI088 初始化（会返回错误码）
uint8_t bmi088_error = BMI088_init();
if (bmi088_error != BMI088_NO_ERROR) {
    // 初始化失败处理
}

// 3. 消息中心初始化
MsgCenter_Init(g_msg_queue, MSG_CENTER_QUEUE_LEN);

// 4. 应用层初始化
CmdController_Init();
// ...

// 5. IMU 校准（!!!IMU 必须静止!!!）
gyro_calibrate();  // 约 2 秒

// 6. IMU 数据模块初始化
gyro_data_init();

// 7. 主循环
while (1) {
    gyro_data_update(&sensor_data);  // 更新姿态
    MsgCenter_Dispatch();            // 分发 TOPIC_IMU_UPDATE
    HAL_Delay(5);                    // 目标 200Hz
}
```

### 订阅 IMU 数据

```c
// application/xxx/xxx_controller.c

static void on_imu_update(const MsgEvent *ev, void *user_data) {
    if (ev->size == sizeof(SensorData)) {
        SensorData imu_data;
        memcpy(&imu_data, ev->data, sizeof(SensorData));

        // 使用姿态角（度）
        float yaw = imu_data.yaw;
        float pitch = imu_data.pitch;
        float roll = imu_data.roll;

        // 使用累积 Yaw（多圈）
        float yaw_total = imu_data.yaw_total_angle;
        int yaw_rounds = imu_data.yaw_round_count;

        // 使用原始陀螺仪数据（rad/s）
        float gyro_x = imu_data.g_gx;
        float gyro_y = imu_data.g_gy;
        float gyro_z = imu_data.g_gz;

        // 使用原始加速度数据（m/s²）
        float accel_x = imu_data.g_ax;
        float accel_y = imu_data.g_ay;
        float accel_z = imu_data.g_az;
    }
}

void YourController_Init(void) {
    MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
}
```

### 数据结构

```c
typedef struct {
    // === BMI088 云台 IMU ===
    float yaw;              // Yaw 角度（度，-180 ~ 180）
    float pitch;            // Pitch 角度（度）
    float roll;             // Roll 角度（度）
    float yaw_total_angle;  // Yaw 累积角度（度，多圈）
    int yaw_round_count;    // Yaw 圈数
    float absolute_angle;   // 兼容：= yaw_total_angle

    float g_gx, g_gy, g_gz; // 陀螺仪原始数据（rad/s）
    float g_ax, g_ay, g_az; // 加速度计原始数据（m/s²）

    // === WT61C 底盘 IMU（如果使用）===
    int c_ax, c_ay, c_az;   // 加速度（mg）
    int c_gx, c_gy, c_gz;   // 角速度（0.1 °/s）
    int c_roll, c_pitch, c_yaw;  // 角度（0.1 °）
} SensorData;
```

---

## 性能指标

### 更新频率
- **目标频率**: 200 Hz (5ms)
- **实际频率**: 动态测量，通常 130-200 Hz（取决于主循环负载）
- **EKF 带宽**: 自适应 `dt`

### 姿态精度

| 轴 | 短期精度 | 长期稳定性 | 说明 |
|---|---------|-----------|------|
| **Roll** | ±0.5° | ✅ 稳定 | 加速度计可观测 |
| **Pitch** | ±0.5° | ✅ 稳定 | 加速度计可观测 |
| **Yaw** | ±1.0° | ⚠️ 漂移 | 加速度计不可观测 |

**Yaw 漂移速率**（典型）:
- 校准良好：< 0.5°/分钟
- 温度稳定：< 1°/分钟
- 温度变化：1-3°/分钟

### 角速度测量
- **量程**: ±2000 °/s (±34.9 rad/s)
- **分辨率**: 0.061 °/s (0.00106 rad/s)
- **零偏稳定性**: < 0.01 rad/s (校准后)

### 启动时间
- **BMI088 初始化**: < 200 ms
- **校准时间**: 2 秒
- **总启动时间**: < 3 秒

---

## 调试与诊断

### 调试输出

**校准过程**:
```
[BMI088] Verifying BMI088 initialization...
[BMI088] Initial reading: accel=[0.095,-0.226,9.821]m/s², gyro=[0.002,-0.003,0.010]rad/s, temp=22.1°C
[BMI088] Starting calibration, keep IMU still for 2 seconds...
[BMI088] Attempt 1: Collecting 2000 samples...
[BMI088] First sample: accel=[0.074,-0.206,9.790]m/s², gyro=[-0.002,-0.004,0.006]rad/s, temp=22.1°C
[BMI088] Sample complete: gNorm=9.857 m/s² (1.01g), offset=[0.002243,-0.001789,0.002247]rad/s
[BMI088] ===== Calibration SUCCESS (attempts: 1) =====
Gyro offset: gx=0.002243, gy=-0.001789, gz=0.002247 rad/s
Accel: gNorm=9.857 m/s² (1.01g)
Temp when cali: 22.50 °C
[QuaternionEKF] Initialized with q=[1.000,-0.013,-0.005,-0.000], roll=-1.54°, pitch=-0.53°
```

**运行时数据** (CDC 串口，10Hz):
```
IMU,<timestamp>,<yaw>,<pitch>,<roll>,<yaw_total>,<yaw_rounds>,<gx>,<gy>,<gz>
IMU,8000,0.00,-1.54,-0.60,0.00,0,-0.003,-0.004,0.002
IMU,8100,0.02,-1.41,-0.55,0.02,0,-0.001,0.004,0.005
...
```

### 常见问题

#### Q1: 校准失败，重试多次
**可能原因**:
- IMU 未静止（振动、倾斜）
- 陀螺仪零偏过大（> 0.01 rad/s）
- SPI 通信异常

**解决**:
1. 确保开机时 IMU 完全静止
2. 检查 SPI 接线和信号质量
3. 检查温度（建议室温 20-30°C）

#### Q2: Yaw 漂移很快
**可能原因**:
- 校准时 IMU 未静止
- 陀螺仪零偏未正确应用
- 温度变化（温漂）

**解决**:
1. 重新校准（静止环境）
2. 检查 `gyro_calibrated[2]` 是否正确减去 `gyro_offset[2]`
3. 等待温度稳定后再校准

#### Q3: 角度读数明显不准（如 90° 显示 60°）
**可能原因**:
- `dt` 固定值与实际不符
- 主循环执行时间过长

**解决**:
1. 确认使用动态 `dt` 计算（已修复）
2. 检查主循环中是否有阻塞操作
3. 测量实际循环频率（通过 LED 翻转计时）

#### Q4: Roll/Pitch 在运动时抖动
**可能原因**:
- 运动加速度干扰加速度计
- EKF 参数 `measure_noise` 过小

**解决**:
1. 正常现象，运动时加速度计不可靠
2. 可调大 `measure_noise`（当前 1000000 已很大）
3. 可启用低通滤波 `lpf = 0.0085`（gyro_data.c:60）

---

## 未来改进方向

### 短期优化
- [ ] 实现温度补偿（利用校准时温度数据）
- [ ] 添加 IMU 健康检测（掉线检测、数据有效性检查）
- [ ] 优化 EKF 参数（可针对不同应用场景调参）

### 中期改进
- [ ] 添加 IST8310 磁力计支持（9 轴融合）
- [ ] 实现磁场校准算法（椭球拟合）
- [ ] 自适应 EKF 参数（根据运动状态调整）

### 长期目标
- [ ] 视觉辅助 IMU（视觉 + IMU 融合）
- [ ] 机器学习零偏预测（温度-零飘模型）
- [ ] 完整的 AHRS 系统

---

## 参考文献

### 算法
- Wang Hongxi, "Quaternion-based Extended Kalman Filter for Attitude Estimation"
- Mahony et al., "Nonlinear Complementary Filters on SO(3)"
- Madgwick, S., "An efficient orientation filter for IMU and MARG sensor arrays"

### 硬件
- Bosch BMI088 Datasheet
- STM32F407 Reference Manual
- RoboMaster Development Board C Manual

### 代码参考
- [basic_framework](https://github.com/...) - SJTU RoboMaster 基础框架

---

## 版本历史

### v1.0 (Current) - 2024-12
- ✅ 修复加速度单位错误（g → m/s²）
- ✅ 修复陀螺仪零偏未应用
- ✅ 修复校准成功标志未设置
- ✅ 修复 EKF 参数不一致（lambda）
- ✅ 实现动态 `dt` 测量（修复角度误差）
- ✅ 与 basic_framework 完全统一

---

**文档维护**: 请在修改 IMU 相关代码后更新此文档。

**反馈**: 如有问题或建议，请联系开发团队。
