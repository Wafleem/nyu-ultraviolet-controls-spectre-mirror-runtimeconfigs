# rm_vision 视觉框架集成指南

> **文档版本**: 1.0
> **生成日期**: 2025-12-28
> **目标项目**: robomaster-control (STM32F407)
> **视觉系统**: rm_vision_ws (ROS 2 Humble)
> **作者**: Claude Code

---

## 📋 目录

- [1. 概述](#1-概述)
- [2. 协议对比分析](#2-协议对比分析)
- [3. 架构改造方案](#3-架构改造方案)
- [4. 详细实施步骤](#4-详细实施步骤)
- [5. 代码修改清单](#5-代码修改清单)
- [6. 测试与验证](#6-测试与验证)
- [7. 常见问题](#7-常见问题)
- [8. 附录](#8-附录)

---

## 1. 概述

### 1.1 现状分析

**当前系统**:
- 项目: `robomaster-control`
- 通信协议: **Seasky 协议** (湖南大学)
- 串口: USART6 (115200 baud)
- 数据包: 接收18字节, 发送36字节
- 数据内容: **角度误差** (pitch/yaw, 弧度)

**目标系统**:
- 视觉框架: `rm_vision_ws`
- 通信协议: **rm_serial_driver 协议** (CRC16-MODBUS)
- 串口: /dev/ttyACM0 (115200 baud)
- 数据包: 接收34字节, 发送52字节
- 数据内容: **完整目标信息** (3D位置, 速度, 姿态, ID等)

### 1.2 改造目标

✅ **保持兼容**: 尽量不改动现有云台控制、PID算法、消息中心等核心模块
✅ **模块化**: 只替换 `vision_comm` 模块，其他代码保持不变
✅ **功能增强**: 支持能量机关模式、目标ID识别、完整的时间同步
✅ **向后兼容**: 可选择性编译Seasky或rm_serial_driver协议

### 1.3 改造范围

**需要修改的文件** (共7个):
```
modules/vision_comm/
├── vision_comm.h         ✏️ 修改数据结构和API
├── vision_comm.c         ✏️ 修改收发逻辑
├── rm_serial_protocol.h  ➕ 新增 (替代seasky_protocol.h)
├── rm_serial_protocol.c  ➕ 新增 (替代seasky_protocol.c)
└── crc16.c              ✏️ 修改CRC实现 (使用CRC16-MODBUS)

application/cmd/
└── cmd_controller.c      ✏️ 适配新数据结构

Src/
└── main.c               ⚠️ 可选修改 (调试输出)
```

**不需要修改**:
- ✅ `gimbal_controller.c` - 云台控制逻辑完全兼容
- ✅ `message_center.c` - 消息中心机制不变
- ✅ `can_comm.c` - CAN通信不受影响
- ✅ `pid.c` - PID算法不受影响

---

## 2. 协议对比分析

### 2.1 数据包结构对比

#### Seasky 协议 (当前使用)

**接收包** (视觉 → 下位机, 18字节):
```
┌────────────────────────────────────────────────────┐
│ [0xA5] [DataLen_L] [DataLen_H] [CRC8]             │ ← 帧头 (4字节)
│ [CmdID_L] [CmdID_H] = 0x0001                      │ ← 命令ID (2字节)
│ [Flags_L] [Flags_H]                               │ ← 标志寄存器 (2字节)
│   bit0-1: fire_mode                               │
│   bit2-3: target_state                            │
│   bit4-7: target_type                             │
│ [Pitch_float] [Yaw_float]                         │ ← 数据 (8字节)
│ [CRC16_L] [CRC16_H]                               │ ← 校验 (2字节)
└────────────────────────────────────────────────────┘
```

**发送包** (下位机 → 视觉, 36字节):
```
┌────────────────────────────────────────────────────┐
│ [0xA5] [DataLen_L] [DataLen_H] [CRC8]             │ ← 帧头 (4字节)
│ [CmdID_L] [CmdID_H] = 0x0002                      │ ← 命令ID (2字节)
│ [Flags_L] [Flags_H]                               │ ← 标志寄存器 (2字节)
│   bit0-7:  enemy_color                            │
│   bit8-11: work_mode                              │
│   bit12-15: bullet_speed                          │
│ [Yaw] [Pitch] [Roll]                              │ ← 姿态 (12字节)
│ [Float4] [Float5] [Float6]                        │ ← 扩展 (12字节)
│ [Reserved_L] [Reserved_H]                         │ ← 保留 (2字节)
│ [CRC16_L] [CRC16_H]                               │ ← 校验 (2字节)
└────────────────────────────────────────────────────┘
```

#### rm_serial_driver 协议 (目标)

**接收包** (视觉 → 下位机, 52字节):
```c
struct SendPacket {
    uint8_t header = 0xA5;                   // [0] 帧头
    uint8_t state : 2;                       // [1] bit0-1: 状态
            //  0=未跟踪 1=跟踪装甲板 2=跟踪能量机关
    uint8_t id : 3;                          //     bit2-4: 目标ID
            //  0=前哨站 1-5=机器人 6=哨兵 7=基地
    uint8_t armors_num : 3;                  //     bit5-7: 装甲板数量
            //  2=平衡步兵 3=前哨站 4=普通机器人

    // === 自瞄模式数据 ===
    float x;                                 // [2-5] 机器人中心X (m, odom坐标系)
    float y;                                 // [6-9] 机器人中心Y
    float z;                                 // [10-13] 机器人中心Z
    float yaw;                               // [14-17] 机器人偏航角 (rad)
    float vx;                                // [18-21] 速度X (m/s)
    float vy;                                // [22-25] 速度Y
    float vz;                                // [26-29] 速度Z
    float v_yaw;                             // [30-33] 角速度 (rad/s)
    float r1;                                // [34-37] 旋转半径参数1
    float r2;                                // [38-41] 旋转半径参数2
    float dz;                                // [42-45] 装甲板高度差

    // === 能量机关模式 (复用字段) ===
    // x, y, z  → 符文中心位置
    // yaw      → 符文角度θ
    // vx = a   → 速度拟合参数 (speed = a*sin(w*t)+b)
    // vy = b
    // vz = w

    uint32_t cap_timestamp;                  // [46-49] 图像时间戳 (ms)
    uint16_t t_offset;                       // [50-51] 能量机关时间偏移
    uint16_t checksum;                       // [52-53] CRC16
} __attribute__((packed));  // 总计54字节
```

**发送包** (下位机 → 视觉, 34字节):
```c
struct ReceivePacket {
    uint8_t header = 0x5A;                   // [0] 帧头
    uint8_t detect_color : 1;                // [1] bit0: 检测颜色
            //  0=检测蓝色(敌方红) 1=检测红色(敌方蓝)
    uint8_t task_mode : 2;                   //     bit1-2: 任务模式
            //  0=自动 1=强制自瞄 2=强制能量机关
    bool reset_tracker : 1;                  //     bit3: 重置跟踪器
    uint8_t is_play : 1;                     //     bit4: 比赛开始标志
    bool change_target : 1;                  //     bit5: 切换目标
    uint8_t reserved : 2;                    //     bit6-7: 保留

    float roll;                              // [2-5] 云台横滚角 (rad)
    float pitch;                             // [6-9] 云台俯仰角 (rad)
    float yaw;                               // [10-13] 云台偏航角 (rad)
    float aim_x;                             // [14-17] 瞄准点X (m, 预留)
    float aim_y;                             // [18-21] 瞄准点Y
    float aim_z;                             // [22-25] 瞄准点Z
    uint16_t game_time;                      // [26-27] 比赛时间 [0, 450]s
    uint32_t timestamp;                      // [28-31] 下位机时间戳 (ms)
    uint16_t checksum;                       // [32-33] CRC16
} __attribute__((packed));  // 总计34字节
```

### 2.2 关键差异对比表

| 维度 | Seasky 协议 | rm_serial_driver 协议 | 影响 |
|------|------------|----------------------|------|
| **接收帧头** | 0xA5 | 0xA5 | ✅ 相同 |
| **发送帧头** | 0xA5 | 0x5A | ⚠️ 不同 |
| **接收包大小** | 18字节 | 52字节 | ⚠️ 需调整缓冲区 |
| **发送包大小** | 36字节 | 34字节 | ⚠️ 需调整缓冲区 |
| **CRC校验** | CRC8+CRC16 | CRC16-MODBUS | ⚠️ 需重写CRC |
| **数据内容** | 角度误差 | 3D位置+速度 | ⚠️ 需转换逻辑 |
| **能量机关** | 不支持 | 支持 | ✅ 功能增强 |
| **时间同步** | 无 | 完整时间戳 | ✅ 功能增强 |

### 2.3 数据转换方案

**核心问题**: rm_vision发送的是**3D位置信息**，而云台控制器期望的是**角度误差**。

**解决方案**: 在下位机侧将3D位置转换为角度误差

```c
// 伪代码
void convert_position_to_angle_error(SendPacket *pkt, Vision_Recv_s *out) {
    if (pkt->state == 1) {  // 跟踪装甲板
        // 1. 计算目标到云台的相对位置
        float target_x = pkt->x;  // 已经在odom坐标系下
        float target_y = pkt->y;
        float target_z = pkt->z;

        // 2. 转换为云台角度
        float horizontal_dist = sqrtf(target_x * target_x + target_y * target_y);
        float pitch_angle = atan2f(target_z, horizontal_dist);
        float yaw_angle = atan2f(target_y, target_x);

        // 3. 加入弹道补偿 (可选)
        float bullet_speed = 15.0f;  // m/s
        float gravity_drop = calculate_ballistic_drop(horizontal_dist, bullet_speed);
        pitch_angle += gravity_drop;

        // 4. 计算误差 (相对当前云台角度)
        out->pitch = pitch_angle - current_gimbal_pitch;
        out->yaw = yaw_angle - current_gimbal_yaw;
        out->target_state = READY_TO_FIRE;
    }
}
```

**优势**:
- 视觉系统无需了解弹道模型，专注于目标识别
- 下位机可自主进行弹道补偿和提前量计算
- 支持多种弹速切换

---

## 3. 架构改造方案

### 3.1 改造策略

采用**最小侵入式改造**:

```
┌────────────────────────────────────────────────────────────┐
│                   robomaster-control                       │
│                                                            │
│  ┌──────────────────────────────────────────────────────┐ │
│  │  application/cmd/cmd_controller.c                    │ │
│  │  ✏️ 修改: 适配新的Vision_Recv_s结构                   │ │
│  └────────────────────┬─────────────────────────────────┘ │
│                       │ MsgCenter                          │
│                       ↓                                    │
│  ┌──────────────────────────────────────────────────────┐ │
│  │  modules/vision_comm/                                │ │
│  │  ┌────────────────────────────────────────────────┐  │ │
│  │  │ vision_comm.h  ✏️ 修改数据结构                  │  │ │
│  │  │ vision_comm.c  ✏️ 修改收发逻辑                  │  │ │
│  │  ├────────────────────────────────────────────────┤  │ │
│  │  │ rm_serial_protocol.h  ➕ 新增                  │  │ │
│  │  │ rm_serial_protocol.c  ➕ 新增                  │  │ │
│  │  └────────────────────────────────────────────────┘  │ │
│  └──────────────────────────────────────────────────────┘ │
│                       ↕ USART6                             │
│                       │                                    │
└───────────────────────┼────────────────────────────────────┘
                        │
                        ↕ 115200 baud, 8N1
                        │
┌───────────────────────┼────────────────────────────────────┐
│                rm_vision_ws                                │
│                       │                                    │
│  ┌────────────────────┴───────────────────────────────┐   │
│  │  rm_serial_driver                                  │   │
│  │  - 发送: ReceivePacket (0x5A, 34字节)             │   │
│  │  - 接收: SendPacket (0xA5, 52字节)                │   │
│  └────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────┘
```

### 3.2 模块职责划分

**vision_comm 模块**:
- ✅ 串口接收/发送 (USART6)
- ✅ 协议解析/打包 (rm_serial_driver格式)
- ✅ CRC16校验
- ✅ 3D位置 → 角度误差转换
- ✅ 发布 `TOPIC_VISION_DATA` 消息

**cmd_controller 模块**:
- ✅ 订阅 `TOPIC_VISION_DATA`
- ✅ 视觉数据超时检测 (80ms)
- ✅ 设置 `GimbalCmd.vision_valid` 和 `vision_yaw_err_rad`

**gimbal_controller 模块**:
- ✅ 无需修改！接收误差角度，更新目标角度
- ✅ PID控制保持不变

### 3.3 兼容性方案 (可选)

如果需要同时支持两种协议，可使用**编译时选择**:

```c
// vision_comm.h
#define VISION_PROTOCOL_SEASKY        0
#define VISION_PROTOCOL_RM_SERIAL     1

// 选择协议 (在此修改)
#define VISION_PROTOCOL  VISION_PROTOCOL_RM_SERIAL

#if (VISION_PROTOCOL == VISION_PROTOCOL_SEASKY)
    #define VISION_RECV_SIZE 18u
    #define VISION_SEND_SIZE 36u
#else
    #define VISION_RECV_SIZE 52u
    #define VISION_SEND_SIZE 34u
#endif
```

---

## 4. 详细实施步骤

### 步骤 1: 备份现有代码

```bash
cd /Users/pengyue/Codespace/robomaster-control
git checkout -b feature/rm-vision-integration
git add -A
git commit -m "backup: before rm_vision integration"
```

### 步骤 2: 创建新协议文件

#### 2.1 创建 `rm_serial_protocol.h`

```c
// modules/vision_comm/rm_serial_protocol.h
#ifndef RM_SERIAL_PROTOCOL_H
#define RM_SERIAL_PROTOCOL_H

#include <stdint.h>

#pragma pack(push, 1)

/**
 * @brief 接收数据包 (视觉 → 下位机, 52字节)
 */
typedef struct {
    uint8_t header;          // 0xA5
    uint8_t state : 2;       // 0=未跟踪 1=跟踪装甲板 2=跟踪能量机关
    uint8_t id : 3;          // 0=前哨站 1-5=机器人 6=哨兵 7=基地
    uint8_t armors_num : 3;  // 2=平衡 3=前哨站 4=普通
    float x;                 // 目标中心位置 (m)
    float y;
    float z;
    float yaw;               // 目标偏航角 (rad)
    float vx;                // 速度 (m/s)
    float vy;
    float vz;
    float v_yaw;             // 角速度 (rad/s)
    float r1;                // 旋转参数
    float r2;
    float dz;
    uint32_t cap_timestamp;  // 图像时间戳 (ms)
    uint16_t t_offset;       // 能量机关时间偏移
    uint16_t checksum;       // CRC16
} VisionSendPacket;

/**
 * @brief 发送数据包 (下位机 → 视觉, 34字节)
 */
typedef struct {
    uint8_t header;          // 0x5A
    uint8_t detect_color : 1;   // 0=检测蓝色 1=检测红色
    uint8_t task_mode : 2;      // 0=自动 1=自瞄 2=能量机关
    bool reset_tracker : 1;
    uint8_t is_play : 1;
    bool change_target : 1;
    uint8_t reserved : 2;
    float roll;              // 云台姿态 (rad)
    float pitch;
    float yaw;
    float aim_x;             // 瞄准点 (预留)
    float aim_y;
    float aim_z;
    uint16_t game_time;      // 比赛时间 [0, 450]s
    uint32_t timestamp;      // 下位机时间戳 (ms)
    uint16_t checksum;       // CRC16
} VisionRecvPacket;

#pragma pack(pop)

/**
 * @brief 解析接收到的视觉数据包
 * @param buf 接收缓冲区
 * @param len 数据长度
 * @param pkt 输出数据包
 * @return 1=成功 0=失败
 */
uint8_t RMSerial_ParseRecvPacket(const uint8_t *buf, uint32_t len, VisionSendPacket *pkt);

/**
 * @brief 打包发送数据包
 * @param pkt 输入数据包
 * @param buf 输出缓冲区
 * @param len 输出长度
 */
void RMSerial_PackSendPacket(const VisionRecvPacket *pkt, uint8_t *buf, uint32_t *len);

/**
 * @brief 计算CRC16-MODBUS
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC16值
 */
uint16_t RMSerial_CRC16(const uint8_t *data, uint32_t length);

/**
 * @brief 验证CRC16
 * @param data 数据指针 (包含末尾2字节CRC)
 * @param length 总长度
 * @return 1=通过 0=失败
 */
uint8_t RMSerial_VerifyCRC16(const uint8_t *data, uint32_t length);

#endif // RM_SERIAL_PROTOCOL_H
```

#### 2.2 创建 `rm_serial_protocol.c`

```c
// modules/vision_comm/rm_serial_protocol.c
#include "rm_serial_protocol.h"
#include <string.h>

// CRC16-MODBUS 查找表
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t RMSerial_CRC16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ data[i]) & 0xFF];
    }

    return crc;
}

uint8_t RMSerial_VerifyCRC16(const uint8_t *data, uint32_t length)
{
    if (length < 3) return 0;

    uint16_t calc_crc = RMSerial_CRC16(data, length - 2);
    uint16_t recv_crc = data[length - 2] | (data[length - 1] << 8);

    return (calc_crc == recv_crc) ? 1 : 0;
}

uint8_t RMSerial_ParseRecvPacket(const uint8_t *buf, uint32_t len, VisionSendPacket *pkt)
{
    // 1. 验证长度和帧头
    if (len != sizeof(VisionSendPacket) || buf[0] != 0xA5) {
        return 0;
    }

    // 2. 验证CRC16
    if (!RMSerial_VerifyCRC16(buf, len)) {
        return 0;
    }

    // 3. 拷贝数据
    memcpy(pkt, buf, sizeof(VisionSendPacket));

    return 1;
}

void RMSerial_PackSendPacket(const VisionRecvPacket *pkt, uint8_t *buf, uint32_t *len)
{
    // 1. 拷贝数据
    memcpy(buf, pkt, sizeof(VisionRecvPacket) - 2);  // 不包含CRC

    // 2. 计算并附加CRC16
    uint16_t crc = RMSerial_CRC16(buf, sizeof(VisionRecvPacket) - 2);
    buf[sizeof(VisionRecvPacket) - 2] = crc & 0xFF;
    buf[sizeof(VisionRecvPacket) - 1] = (crc >> 8) & 0xFF;

    *len = sizeof(VisionRecvPacket);
}
```

### 步骤 3: 修改 vision_comm 模块

#### 3.1 修改 `vision_comm.h`

```c
// modules/vision_comm/vision_comm.h
#ifndef VISION_COMM_H
#define VISION_COMM_H

#include "main.h"
#include "usart.h"
#include "rm_serial_protocol.h"
#include <stdint.h>

// Vision communication UART handle
#define VISION_UART_HANDLE huart6

#define VISION_RECV_SIZE 52u  // 修改: VisionSendPacket大小
#define VISION_SEND_SIZE 34u  // 修改: VisionRecvPacket大小

#pragma pack(1)

// Target state (保持兼容)
typedef enum {
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

// Target type (扩展)
typedef enum {
    TARGET_OUTPOST = 0,
    TARGET_HERO = 1,
    TARGET_ENGINEER = 2,
    TARGET_INFANTRY_3 = 3,
    TARGET_INFANTRY_4 = 4,
    TARGET_INFANTRY_5 = 5,
    TARGET_SENTRY = 6,
    TARGET_BASE = 7
} Target_Type_e;

// Vision receive data structure (修改: 添加完整信息)
typedef struct {
    Target_State_e target_state;  // 目标状态
    Target_Type_e target_type;    // 目标类型

    // 核心数据: 角度误差 (rad) - 兼容现有云台控制器
    float pitch;                  // 俯仰角误差
    float yaw;                    // 偏航角误差

    // 扩展数据: 原始3D信息 (可选使用)
    float target_x;               // 目标位置 (m)
    float target_y;
    float target_z;
    float target_yaw;             // 目标偏航角 (rad)
    float target_vx;              // 目标速度 (m/s)
    float target_vy;
    float target_vz;

    uint8_t armors_num;           // 装甲板数量 (2/3/4)
    uint8_t updated;              // 更新标志
    uint32_t timestamp;           // 时间戳
} Vision_Recv_s;

// Enemy color (保持不变)
typedef enum {
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED = 2,
} Enemy_Color_e;

// Task mode (修改: 适配rm_vision)
typedef enum {
    TASK_MODE_AUTO = 0,
    TASK_MODE_AIM = 1,
    TASK_MODE_BUFF = 2
} Task_Mode_e;

// Vision send data structure (保持兼容)
typedef struct {
    Enemy_Color_e enemy_color;
    Task_Mode_e task_mode;
    float yaw;
    float pitch;
    float roll;
    uint16_t game_time;
    uint32_t timestamp;
} Vision_Send_s;

#pragma pack()

// API (保持不变)
Vision_Recv_s *VisionComm_Init(void);
void VisionComm_Send(void);
void VisionComm_SetFlag(Enemy_Color_e enemy_color, Task_Mode_e task_mode);
void VisionComm_SetAltitude(float yaw, float pitch, float roll);
Vision_Recv_s *VisionComm_GetData(void);
void VisionComm_RxCallback(uint8_t *buf, uint32_t len);
void VisionComm_StartReceive(void);

#endif // VISION_COMM_H
```

#### 3.2 修改 `vision_comm.c`

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

// 当前云台姿态 (用于计算角度误差)
extern SensorData sensor_data;

/**
 * @brief 3D位置转换为角度误差
 */
static void convert_position_to_angle_error(const VisionSendPacket *pkt, Vision_Recv_s *out)
{
    if (pkt->state == 1) {  // 跟踪装甲板
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

        // 计算目标角度
        float horizontal_dist = sqrtf(pkt->x * pkt->x + pkt->y * pkt->y);
        float pitch_angle = atan2f(pkt->z, horizontal_dist);
        float yaw_angle = atan2f(pkt->y, pkt->x);

        // 【可选】弹道补偿 (根据实际情况调整)
        // float bullet_speed = 15.0f;  // m/s
        // float flight_time = horizontal_dist / bullet_speed;
        // float gravity_drop = 0.5f * 9.8f * flight_time * flight_time;
        // pitch_angle += atan2f(gravity_drop, horizontal_dist);

        // 计算误差 (相对当前云台角度)
        float current_pitch = sensor_data.pitch * M_PI / 180.0f;  // 度→弧度
        float current_yaw = sensor_data.yaw_total_angle * M_PI / 180.0f;

        out->pitch = pitch_angle - current_pitch;
        out->yaw = yaw_angle - current_yaw;

        // 角度环绕到[-π, π]
        while (out->yaw > M_PI) out->yaw -= 2.0f * M_PI;
        while (out->yaw < -M_PI) out->yaw += 2.0f * M_PI;

        out->target_state = READY_TO_FIRE;

    } else if (pkt->state == 2) {  // 能量机关模式
        // TODO: 实现能量机关角度计算
        out->target_state = TARGET_CONVERGING;

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

    // 订阅IMU更新 (用于定时发送)
    MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

    VisionComm_StartReceive();
    return &recv_data;
}

/**
 * @brief 启动UART接收
 */
void VisionComm_StartReceive(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&VISION_UART_HANDLE, uart_recv_buff, VISION_RECV_SIZE);
}

/**
 * @brief UART接收回调
 */
void VisionComm_RxCallback(uint8_t *buf, uint32_t len)
{
    VisionSendPacket pkt;

    // 1. 复制到处理缓冲区
    if (len >= VISION_RECV_SIZE && len <= VISION_RECV_SIZE) {
        memcpy(uart_recv_processing, buf, len);

        // 2. 解析协议
        if (RMSerial_ParseRecvPacket(uart_recv_processing, len, &pkt)) {
            // 3. 转换为角度误差
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

/**
 * @brief 设置姿态数据
 */
void VisionComm_SetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 设置标志位
 */
void VisionComm_SetFlag(Enemy_Color_e enemy_color, Task_Mode_e task_mode)
{
    send_data.enemy_color = enemy_color;
    send_data.task_mode = task_mode;
}

/**
 * @brief 发送数据
 */
void VisionComm_Send(void)
{
    VisionRecvPacket pkt = {0};
    uint32_t len;

    // 1. 填充数据包
    pkt.header = 0x5A;
    pkt.detect_color = (send_data.enemy_color == COLOR_BLUE) ? 1 : 0;
    pkt.task_mode = send_data.task_mode;
    pkt.reset_tracker = 0;
    pkt.is_play = 1;
    pkt.change_target = 0;
    pkt.roll = send_data.roll;
    pkt.pitch = send_data.pitch;
    pkt.yaw = send_data.yaw;
    pkt.aim_x = 0.0f;
    pkt.aim_y = 0.0f;
    pkt.aim_z = 0.0f;
    pkt.game_time = send_data.game_time;
    pkt.timestamp = send_data.timestamp;

    // 2. 打包并附加CRC16
    RMSerial_PackSendPacket(&pkt, uart_send_buff, &len);

    // 3. 发送
    HAL_UART_Transmit(&VISION_UART_HANDLE, uart_send_buff, len, 10);
}

/**
 * @brief IMU更新回调 (100Hz)
 */
static void on_imu_update(const MsgEvent *ev, void *user_data)
{
    if (ev->size == sizeof(SensorData)) {
        const SensorData *sd = (const SensorData *)ev->data;
        uint32_t current_time = HAL_GetTick();

        // 频率控制: 每10ms发送一次
        if (current_time - last_send_time >= 10) {
            // 设置姿态 (弧度)
            VisionComm_SetAltitude(
                sd->yaw_total_angle * M_PI / 180.0f,
                sd->pitch * M_PI / 180.0f,
                sd->roll * M_PI / 180.0f
            );

            // 设置时间戳和比赛时间
            send_data.timestamp = current_time;
            send_data.game_time = 0;  // TODO: 从裁判系统获取

            VisionComm_SetFlag(COLOR_BLUE, TASK_MODE_AIM);
            VisionComm_Send();

            last_send_time = current_time;
        }
    }
}

/**
 * @brief 获取视觉数据
 */
Vision_Recv_s *VisionComm_GetData(void)
{
    return &recv_data;
}
```

### 步骤 4: 修改 cmd_controller (可选)

如果需要使用新的扩展信息，可以修改 `cmd_controller.c`:

```c
// application/cmd/cmd_controller.c
// 在 on_vision_update 回调中

static void on_vision_update(const MsgEvent *ev, void *user_data) {
    if (ev->size == sizeof(Vision_Recv_s)) {
        memcpy(&s_last_vision, ev->data, sizeof(Vision_Recv_s));

        // 【新增】可以使用扩展信息
        // float target_distance = sqrtf(
        //     s_last_vision.target_x * s_last_vision.target_x +
        //     s_last_vision.target_y * s_last_vision.target_y +
        //     s_last_vision.target_z * s_last_vision.target_z
        // );
        // 根据距离调整PID参数等...
    }
}
```

### 步骤 5: 更新 CMakeLists.txt (如果使用CMake)

```cmake
# CMakeLists.txt
# 在 modules/vision_comm 的源文件列表中

set(VISION_COMM_SOURCES
    modules/vision_comm/vision_comm.c
    modules/vision_comm/rm_serial_protocol.c  # 新增
    modules/vision_comm/crc16.c
)
```

### 步骤 6: 编译和烧录

```bash
# 如果使用CMake
mkdir build && cd build
cmake ..
make

# 如果使用STM32CubeIDE
# 1. 在IDE中右键项目 -> Build Project
# 2. 右键项目 -> Run As -> STM32 C/C++ Application

# 烧录到板子
# st-link或J-Link
```

---

## 5. 代码修改清单

### 5.1 需要新建的文件

| 文件路径 | 作用 | 行数估计 |
|---------|------|---------|
| `modules/vision_comm/rm_serial_protocol.h` | rm_serial_driver协议定义 | ~100 |
| `modules/vision_comm/rm_serial_protocol.c` | 协议解析/打包实现 | ~150 |

### 5.2 需要修改的文件

| 文件路径 | 修改内容 | 影响范围 |
|---------|---------|---------|
| `modules/vision_comm/vision_comm.h` | 数据结构扩展 | 小 |
| `modules/vision_comm/vision_comm.c` | 收发逻辑重写 | 中 |
| `application/cmd/cmd_controller.c` | 可选扩展 | 极小 |

### 5.3 需要删除的文件 (可选)

| 文件路径 | 说明 |
|---------|------|
| `modules/vision_comm/seasky_protocol.h` | 旧协议 (可保留作备份) |
| `modules/vision_comm/seasky_protocol.c` | 旧协议 |

---

## 6. 测试与验证

### 6.1 单元测试 (Python脚本)

创建测试脚本 `script/test_rm_serial.py`:

```python
#!/usr/bin/env python3
import serial
import struct
import time

# CRC16-MODBUS表
CRC16_TABLE = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    # ... (完整256项)
]

def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]
    return crc

def pack_send_packet(state, id, x, y, z, yaw):
    """
    打包SendPacket (模拟视觉系统发送)
    """
    packet = bytearray()
    packet.append(0xA5)  # header

    # 标志位
    flags = (state & 0x03) | ((id & 0x07) << 2) | ((4 & 0x07) << 5)
    packet.append(flags)

    # float数据 (小端序)
    packet.extend(struct.pack('<f', x))
    packet.extend(struct.pack('<f', y))
    packet.extend(struct.pack('<f', z))
    packet.extend(struct.pack('<f', yaw))
    packet.extend(struct.pack('<f', 0.0))  # vx
    packet.extend(struct.pack('<f', 0.0))  # vy
    packet.extend(struct.pack('<f', 0.0))  # vz
    packet.extend(struct.pack('<f', 0.0))  # v_yaw
    packet.extend(struct.pack('<f', 0.0))  # r1
    packet.extend(struct.pack('<f', 0.0))  # r2
    packet.extend(struct.pack('<f', 0.0))  # dz
    packet.extend(struct.pack('<I', int(time.time() * 1000)))  # timestamp
    packet.extend(struct.pack('<H', 0))  # t_offset

    # CRC16
    crc = crc16_modbus(packet)
    packet.extend(struct.pack('<H', crc))

    return bytes(packet)

def parse_recv_packet(data):
    """
    解析ReceivePacket (下位机发送的)
    """
    if len(data) != 34 or data[0] != 0x5A:
        return None

    # 验证CRC16
    crc_recv = struct.unpack('<H', data[32:34])[0]
    crc_calc = crc16_modbus(data[0:32])
    if crc_recv != crc_calc:
        print(f"CRC错误: 期望{crc_calc:04X}, 收到{crc_recv:04X}")
        return None

    # 解析数据
    flags = data[1]
    detect_color = flags & 0x01
    task_mode = (flags >> 1) & 0x03

    roll, pitch, yaw = struct.unpack('<fff', data[2:14])
    aim_x, aim_y, aim_z = struct.unpack('<fff', data[14:26])
    game_time = struct.unpack('<H', data[26:28])[0]
    timestamp = struct.unpack('<I', data[28:32])[0]

    return {
        'detect_color': detect_color,
        'task_mode': task_mode,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw,
        'timestamp': timestamp
    }

# 主程序
if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

    print("测试rm_serial_driver协议通信...")
    print("发送目标位置: x=2.0m, y=0.5m, z=0.3m")

    try:
        while True:
            # 发送目标数据
            packet = pack_send_packet(
                state=1,     # 跟踪中
                id=3,        # 步兵3号
                x=2.0,       # 目标在前方2米
                y=0.5,       # 右侧0.5米
                z=0.3,       # 上方0.3米
                yaw=0.1      # 偏航角0.1rad ≈ 5.7°
            )
            ser.write(packet)
            print(f"→ 发送 {len(packet)}字节")

            # 接收下位机回传数据
            if ser.in_waiting >= 34:
                recv_data = ser.read(34)
                result = parse_recv_packet(recv_data)
                if result:
                    print(f"← 接收: yaw={result['yaw']:.3f}rad, "
                          f"pitch={result['pitch']:.3f}rad, "
                          f"mode={result['task_mode']}")

            time.sleep(0.1)  # 10Hz

    except KeyboardInterrupt:
        ser.close()
        print("\n测试结束")
```

### 6.2 集成测试步骤

#### 测试1: 串口回环测试

```bash
# 1. 连接USB-TTL模块到USART6 (PA11/PA12)
# 2. 短接TX和RX (回环模式)
# 3. 在main.c的while(1)中添加:

uint8_t test_buf[34] = {0x5A, ...};  // 填充测试数据
HAL_UART_Transmit(&huart6, test_buf, 34, 10);
HAL_Delay(100);

// 观察是否能收到回环数据
```

#### 测试2: 与rm_vision联调

```bash
# 1. 在Jetson上启动rm_vision
ros2 launch rm_vision_bringup vision_bringup.launch.py

# 2. 检查串口设备
ls -l /dev/ttyACM*  # 应该显示串口设备

# 3. 监控话题
ros2 topic echo /tracker/target

# 4. 观察下位机是否收到数据
# 在main.c中添加USB CDC打印:
extern USBD_HandleTypeDef hUsbDeviceFS;
char msg[100];
snprintf(msg, 100, "Vision: state=%d, x=%.2f, y=%.2f\r\n",
         recv_data.target_state, recv_data.target_x, recv_data.target_y);
CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
```

#### 测试3: 云台跟踪测试

```bash
# 1. 放置装甲板在前方2米
# 2. 启动自瞄模式 (左拨杆下位)
# 3. 观察云台是否正确转动
# 4. 检查瞄准精度:
#    - 水平误差 < 3cm
#    - 垂直误差 < 2cm
```

### 6.3 调试技巧

#### 使用USB CDC打印调试信息

```c
// 在vision_comm.c的RxCallback中添加:
#include "usbd_cdc_if.h"

void VisionComm_RxCallback(uint8_t *buf, uint32_t len)
{
    char dbg[128];
    snprintf(dbg, 128, "[RX] len=%lu, header=0x%02X\r\n", len, buf[0]);
    CDC_Transmit_FS((uint8_t*)dbg, strlen(dbg));

    // ... 原有代码 ...
}
```

#### 使用示波器观察波形

```
USART6_TX (PA11) → 示波器CH1
USART6_RX (PA12) → 示波器CH2

触发条件: 上升沿, 3.3V阈值
时基: 100us/div
```

---

## 7. 常见问题

### Q1: 编译错误 "undefined reference to RMSerial_CRC16"

**原因**: CMakeLists.txt未包含新文件

**解决**:
```cmake
# 在CMakeLists.txt中添加
set(SOURCES
    ...
    modules/vision_comm/rm_serial_protocol.c
)
```

### Q2: 收不到视觉数据

**排查步骤**:
```bash
# 1. 检查串口连接
ls -l /dev/ttyACM0  # Jetson侧
# 下位机使用USB CDC打印: "USART6 Init OK"

# 2. 检查波特率
# rm_vision默认115200, 确认STM32CubeMX配置一致

# 3. 检查帧头
# 在RxCallback中打印: buf[0]应为0xA5

# 4. 检查CRC
# 临时注释CRC校验, 看能否接收
```

### Q3: CRC校验失败

**原因**: CRC16-MODBUS多项式或初值错误

**验证**:
```python
# 用Python验证
data = bytes([0xA5, 0x01, ...])  # 填充实际数据
crc = crc16_modbus(data[0:-2])
print(f"期望CRC: {crc:04X}")
# 对比下位机计算的CRC
```

### Q4: 云台抖动

**原因**: 角度误差计算频繁变化

**解决**:
```c
// 在convert_position_to_angle_error中添加低通滤波
static float filtered_pitch = 0.0f;
static float filtered_yaw = 0.0f;

float alpha = 0.3f;  // 滤波系数 (0-1, 越小越平滑)
filtered_pitch = alpha * raw_pitch + (1 - alpha) * filtered_pitch;
filtered_yaw = alpha * raw_yaw + (1 - alpha) * filtered_yaw;

out->pitch = filtered_pitch;
out->yaw = filtered_yaw;
```

### Q5: 如何切换回Seasky协议?

**方法1**: 宏定义切换
```c
// vision_comm.h
#define USE_RM_SERIAL_PROTOCOL  0  // 改为0使用Seasky

#if USE_RM_SERIAL_PROTOCOL
    #include "rm_serial_protocol.h"
#else
    #include "seasky_protocol.h"
#endif
```

**方法2**: Git分支管理
```bash
git checkout -b feature/rm-serial
# 在此分支开发rm_serial版本

git checkout master
# master分支保持Seasky版本
```

---

## 8. 附录

### 8.1 完整的CRC16-MODBUS查找表

```c
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
```

### 8.2 参考资源

- **rm_vision项目**: https://github.com/chenjunnn/rm_vision
- **rm_serial_driver文档**: `/Users/pengyue/Codespace/rm_vision_ws/docs/串口通信协议详解.md`
- **robomaster-control项目**: `/Users/pengyue/Codespace/robomaster-control`
- **CRC16在线计算器**: http://www.ip33.com/crc.html (选择CRC-16/MODBUS)

### 8.3 协议抓包示例

**ReceivePacket (下位机→视觉, 34字节)**:
```
5A 01 00 00 00 3F 00 00 C8 41 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00
E8 03 10 27 00 00 AB CD

解析:
[0]    5A          - 帧头
[1]    01          - detect_color=1, task_mode=0
[2-5]  00 00 00 3F - roll = 0.5 rad
[6-9]  00 00 C8 41 - pitch = 25.0 rad
[10-13] ...        - yaw
[26-27] E8 03      - game_time = 1000s
[28-31] 10 27 00 00 - timestamp = 10000ms
[32-33] AB CD      - CRC16
```

**SendPacket (视觉→下位机, 52字节)**:
```
A5 05 00 00 00 40 00 00 80 3F CD CC CC 3D
...

解析:
[0]    A5          - 帧头
[1]    05          - state=1, id=1, armors_num=0
[2-5]  00 00 00 40 - x = 2.0m
[6-9]  00 00 80 3F - y = 1.0m
[10-13] CD CC CC 3D - z = 0.1m
...
```

---

## 总结

通过本指南，你可以将 `robomaster-control` 项目从 Seasky 协议完整迁移到 rm_vision 的 rm_serial_driver 协议。关键要点：

✅ **最小改动**: 只替换 `vision_comm` 模块，云台控制器无需修改
✅ **功能增强**: 支持完整的3D位置信息、能量机关、时间同步
✅ **向后兼容**: 通过宏定义可以切换回Seasky协议
✅ **充分测试**: 提供Python测试脚本和调试技巧

如有问题，请参考 `docs/mcu_integration_guide.md` 或联系技术支持。

---

**文档版本**: 1.0
**最后更新**: 2025-12-28
**维护者**: Claude Code
