# rm_vision 集成方案修正

> **重要**: 针对之前集成指南的关键问题修正

---

## ⚠️ 发现的问题

### 问题1: sensor_data 获取方式不安全

**错误代码**:
```c
// vision_comm.c 中
extern SensorData sensor_data;  // ❌ 在中断中访问全局变量不安全

static void convert_position_to_angle_error(const VisionSendPacket *pkt, Vision_Recv_s *out)
{
    float current_pitch = sensor_data.pitch * M_PI / 180.0f;  // ❌ 可能读到旧数据
    float current_yaw = sensor_data.yaw_total_angle * M_PI / 180.0f;
    // ...
}
```

**问题**:
1. `VisionComm_RxCallback` 在 **UART中断** 中调用
2. `sensor_data` 在 **主循环** 中更新
3. 可能出现数据不一致或竞态条件

### 问题2: 角度计算缺少临界区保护

**风险**:
- IMU数据更新和视觉数据处理可能同时进行
- 可能读到一半新、一半旧的姿态数据

---

## ✅ 正确实现方案

### 方案1: 通过消息中心传递IMU数据 (推荐)

**思路**: 在视觉接收回调中，从最新的IMU消息获取姿态

#### 修改 `vision_comm.c`

```c
// modules/vision_comm/vision_comm.c
#include "vision_comm.h"
#include "rm_serial_protocol.h"
#include "message_center.h"
#include "gyro_data.h"
#include <string.h>
#include <math.h>

// 接收缓冲区
static uint8_t uart_recv_buff[VISION_RECV_SIZE];
static uint8_t uart_recv_processing[VISION_RECV_SIZE];

// 发送缓冲区
static uint8_t uart_send_buff[VISION_SEND_SIZE];

// 数据存储
static Vision_Recv_s recv_data = {0};
static Vision_Send_s send_data = {0};
static uint32_t last_send_time = 0;

// ✅ 新增: 本地保存最新的IMU数据
static SensorData latest_imu = {0};
static bool imu_data_valid = false;

/**
 * @brief ✅ IMU更新回调 - 保存最新IMU数据
 */
static void on_imu_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    if (ev->size == sizeof(SensorData)) {
        // 临界区保护
        __disable_irq();
        memcpy(&latest_imu, ev->data, sizeof(SensorData));
        imu_data_valid = true;
        __enable_irq();

        // 定时发送姿态给视觉系统
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_send_time >= 10) {  // 100Hz
            VisionComm_SetAltitude(
                latest_imu.yaw_total_angle * M_PI / 180.0f,
                latest_imu.pitch * M_PI / 180.0f,
                latest_imu.roll * M_PI / 180.0f
            );
            send_data.timestamp = current_time;
            send_data.game_time = 0;  // TODO: 从裁判系统获取

            VisionComm_SetFlag(COLOR_BLUE, TASK_MODE_AIM);
            VisionComm_Send();

            last_send_time = current_time;
        }
    }
}

/**
 * @brief ✅ 3D位置转换为角度误差 (改进版)
 */
static void convert_position_to_angle_error(const VisionSendPacket *pkt, Vision_Recv_s *out)
{
    if (pkt->state == 1 && imu_data_valid) {  // 跟踪装甲板且IMU数据有效
        // ✅ 临界区保护：读取IMU数据
        SensorData imu_snapshot;
        __disable_irq();
        memcpy(&imu_snapshot, &latest_imu, sizeof(SensorData));
        __enable_irq();

        // 保存原始信息
        out->target_x = pkt->x;
        out->target_y = pkt->y;
        out->target_z = pkt->z;
        out->target_yaw = pkt->yaw;
        out->target_vx = pkt->vx;
        out->target_vy = pkt->vy;
        out->target_vz = pkt->vz;
        out->armors_num = pkt->armors_num;
        out->target_type = (Target_Type_e)pkt->id;

        // ===== 核心算法: odom系位置 -> 云台角度误差 =====

        // 1. 计算目标在odom系下的绝对角度
        float horizontal_dist = sqrtf(pkt->x * pkt->x + pkt->y * pkt->y);
        float target_pitch_rad = atan2f(pkt->z, horizontal_dist);
        float target_yaw_rad = atan2f(pkt->y, pkt->x);

        // 2. 获取云台在odom系下的当前绝对角度 (度->弧度)
        float current_pitch_rad = imu_snapshot.pitch * M_PI / 180.0f;
        float current_yaw_rad = imu_snapshot.yaw_total_angle * M_PI / 180.0f;

        // 3. 【可选】弹道补偿 (重力下坠)
        // TODO: 根据实际弹速和距离计算
        // float bullet_speed = 15.0f;  // m/s
        // float flight_time = horizontal_dist / bullet_speed;
        // float gravity_drop = 0.5f * 9.8f * flight_time * flight_time;
        // target_pitch_rad += atan2f(gravity_drop, horizontal_dist);

        // 4. 【可选】移动提前量 (运动目标)
        // TODO: 根据目标速度计算
        // float lead_time = horizontal_dist / bullet_speed;
        // target_yaw_rad += atan2f(pkt->vy * lead_time, pkt->x + pkt->vx * lead_time);

        // 5. 计算角度误差 (目标角度 - 当前角度)
        out->pitch = target_pitch_rad - current_pitch_rad;
        out->yaw = target_yaw_rad - current_yaw_rad;

        // 6. 角度环绕到 [-π, π]
        while (out->yaw > M_PI) out->yaw -= 2.0f * M_PI;
        while (out->yaw < -M_PI) out->yaw += 2.0f * M_PI;
        while (out->pitch > M_PI) out->pitch -= 2.0f * M_PI;
        while (out->pitch < -M_PI) out->pitch += 2.0f * M_PI;

        // 7. 设置状态
        out->target_state = READY_TO_FIRE;

    } else if (pkt->state == 2) {  // 能量机关模式
        // TODO: 实现能量机关角度计算
        out->target_state = TARGET_CONVERGING;
        out->pitch = 0.0f;
        out->yaw = 0.0f;

    } else {  // 未跟踪
        out->pitch = 0.0f;
        out->yaw = 0.0f;
        out->target_state = NO_TARGET;
    }
}

/**
 * @brief 初始化视觉通信
 */
Vision_Recv_s *VisionComm_Init(void)
{
    memset(&recv_data, 0, sizeof(Vision_Recv_s));
    memset(&send_data, 0, sizeof(Vision_Send_s));

    // ✅ 订阅IMU更新
    MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

    VisionComm_StartReceive();
    return &recv_data;
}

/**
 * @brief UART接收回调
 */
void VisionComm_RxCallback(uint8_t *buf, uint32_t len)
{
    VisionSendPacket pkt;

    // 1. 复制到处理缓冲区
    if (len == VISION_RECV_SIZE) {
        memcpy(uart_recv_processing, buf, len);

        // 2. 解析协议
        if (RMSerial_ParseRecvPacket(uart_recv_processing, len, &pkt)) {
            // 3. ✅ 转换为角度误差 (内部会安全读取latest_imu)
            convert_position_to_angle_error(&pkt, &recv_data);
            recv_data.updated = 1;
            recv_data.timestamp = pkt.cap_timestamp;

            // 4. 发布到消息中心
            MsgCenter_Publish(TOPIC_VISION_DATA, &recv_data, sizeof(Vision_Recv_s));
        }
    }

    // 5. 重启接收
    VisionComm_StartReceive();
}

// ... 其余函数保持不变 ...
```

### 方案2: 使用原子操作 (轻量级)

如果不想修改太多代码，可以使用原子读取：

```c
// vision_comm.c
#include "gyro_data.h"

// 在main.c中定义
extern SensorData sensor_data;

static void convert_position_to_angle_error(const VisionSendPacket *pkt, Vision_Recv_s *out)
{
    if (pkt->state == 1) {
        // ✅ 原子读取: 临界区保护
        float current_pitch, current_yaw;
        __disable_irq();
        current_pitch = sensor_data.pitch;
        current_yaw = sensor_data.yaw_total_angle;
        __enable_irq();

        // 转换为弧度
        current_pitch *= M_PI / 180.0f;
        current_yaw *= M_PI / 180.0f;

        // ... 后续计算与方案1相同 ...
    }
}
```

---

## 📊 角度计算验证

### 测试用例1: 目标正前方

```
输入:
  pkt->x = 2.0, pkt->y = 0.0, pkt->z = 0.3
  current_yaw = 0.0, current_pitch = 0.0

计算:
  target_yaw = atan2(0, 2) = 0.0 rad
  target_pitch = atan2(0.3, 2) ≈ 0.148 rad ≈ 8.5°

  error_yaw = 0.0 - 0.0 = 0.0  ✅
  error_pitch = 0.148 - 0.0 = 0.148 rad ✅

结果: 云台抬高8.5° ✅
```

### 测试用例2: 目标在左侧

```
输入:
  pkt->x = 2.0, pkt->y = 1.0, pkt->z = 0.0
  current_yaw = 0.0, current_pitch = 0.0

计算:
  target_yaw = atan2(1, 2) ≈ 0.464 rad ≈ 26.6°
  target_pitch = atan2(0, sqrt(5)) = 0.0

  error_yaw = 0.464 - 0.0 = 0.464 rad ✅
  error_pitch = 0.0 - 0.0 = 0.0 ✅

结果: 云台左转26.6° ✅
```

### 测试用例3: 云台已转，目标仍在odom系原位置

```
输入:
  pkt->x = 2.0, pkt->y = 0.0, pkt->z = 0.0 (odom系正前方)
  current_yaw = 90.0° = 1.571 rad (云台已左转90°)
  current_pitch = 0.0

计算:
  target_yaw = atan2(0, 2) = 0.0 rad
  target_pitch = 0.0

  error_yaw = 0.0 - 1.571 = -1.571 rad ≈ -90° ✅
  error_pitch = 0.0 - 0.0 = 0.0 ✅

结果: 云台右转90°回到正前方 ✅
```

---

## 🎯 推荐实现

**使用方案1（消息中心）**，因为：
- ✅ 符合项目架构（发布-订阅模式）
- ✅ 线程安全（临界区保护）
- ✅ 数据一致性有保障
- ✅ 易于调试和扩展

**代码修改清单**:
1. `modules/vision_comm/vision_comm.c` - 添加 `latest_imu` 和临界区保护
2. 无需修改其他文件 ✅

---

## 📝 关键要点总结

### ✅ 正确理解

1. **odom是惯性坐标系**:
   - 原点在云台中心
   - 方向固定不动（由云台初始朝向定义）
   - 云台旋转时，gimbal_link旋转，但odom不变

2. **角度误差计算**:
   ```c
   误差 = 目标在odom系的角度 - 云台在odom系的当前角度
   ```

3. **单位转换**:
   - `sensor_data.pitch/yaw` 是**度**
   - 需要转换为**弧度**计算
   - 发送给云台控制器的误差是**弧度**

### ⚠️ 常见错误

1. ❌ 以为xyz是相对于云台当前朝向的
   - 实际: xyz在odom惯性系下

2. ❌ 误差 = 当前角度 - 目标角度
   - 正确: 误差 = 目标角度 - 当前角度

3. ❌ 直接在中断中访问全局变量
   - 正确: 使用临界区保护或消息中心

---

## 🔧 调试建议

### 1. 添加调试输出

```c
// vision_comm.c 中
void VisionComm_RxCallback(uint8_t *buf, uint32_t len)
{
    // ... 解析代码 ...

    #ifdef DEBUG_VISION_ANGLE
    char dbg[200];
    snprintf(dbg, 200,
        "[Vision] Target: x=%.2f y=%.2f z=%.2f | "
        "Yaw: target=%.2f current=%.2f err=%.3f | "
        "Pitch: target=%.2f current=%.2f err=%.3f\r\n",
        pkt.x, pkt.y, pkt.z,
        target_yaw_rad * 180.0f / M_PI,
        current_yaw_rad * 180.0f / M_PI,
        recv_data.yaw * 180.0f / M_PI,
        target_pitch_rad * 180.0f / M_PI,
        current_pitch_rad * 180.0f / M_PI,
        recv_data.pitch * 180.0f / M_PI
    );
    CDC_Transmit_FS((uint8_t*)dbg, strlen(dbg));
    #endif
}
```

### 2. Python可视化工具

```python
#!/usr/bin/env python3
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

ser = serial.Serial('/dev/ttyACM0', 115200)
yaw_errors = []
pitch_errors = []

def update(frame):
    if ser.in_waiting:
        line = ser.readline().decode().strip()
        if line.startswith('[Vision]'):
            # 解析调试输出
            # ...
            yaw_errors.append(yaw_err)
            pitch_errors.append(pitch_err)

            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(yaw_errors[-100:])
            plt.ylabel('Yaw Error (deg)')

            plt.subplot(2, 1, 2)
            plt.plot(pitch_errors[-100:])
            plt.ylabel('Pitch Error (deg)')

ani = FuncAnimation(plt.gcf(), update, interval=50)
plt.show()
```

---

**文档版本**: 1.1
**修正日期**: 2025-12-28
**关键改进**: 修正了IMU数据获取方式，添加了临界区保护
