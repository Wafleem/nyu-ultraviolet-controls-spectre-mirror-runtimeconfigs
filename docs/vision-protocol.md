# Vision Communication Protocol

## Overview

The vision system sends target information (pitch, yaw, target type, etc.) to the control board, and the control board sends attitude data (yaw, pitch, roll) back to the vision system. All communication happens over USB using the **Seasky protocol** (from Hunan University).

## Communication Channel

- **Physical interface**: USB Virtual COM Port (VCP), later we will migrate to USART
- **Protocol**: Seasky protocol with CRC8 and CRC16 checksums
- **Receive size**: 18 bytes
- **Send size**: 36 bytes

## Seasky Protocol Format

The Seasky protocol is a binary protocol with the following structure:

```
┌───────────────────────────────────────────┐
│ Frame Header (4 bytes)                    │
│  - SOF: 0xA5 (1 byte)                     │
│  - Data Length (2 bytes, little-endian)   │
│  - CRC8 checksum (1 byte)                 │
├───────────────────────────────────────────┤
│ Command ID (2 bytes, little-endian)       │
├───────────────────────────────────────────┤
│ Flags Register (2 bytes, little-endian)   │
├───────────────────────────────────────────┤
│ Data Payload (variable length)            │
│  - Float data (4 bytes per float)          │
├───────────────────────────────────────────┤
│ Frame Tail (2 bytes)                      │
│  - CRC16 checksum (2 bytes, little-endian)│
└───────────────────────────────────────────┘
```

### Frame Header

- **SOF (Start of Frame)**: Always `0xA5` - identifies the start of a frame
- **Data Length**: Length of data payload + flags register (2 bytes) + command ID (2 bytes) = total data section length
- **CRC8**: Checksum for the first 3 bytes (SOF + Data Length)

### Command ID

Identifies what type of data this frame contains:
- `0x0001` - Vision data from upper computer (receive)
- `0x0002` - Attitude data to upper computer (send)

### Flags Register

A 16-bit register that encodes various flags:
- For receive (`0x0001`): Encodes fire mode, target state, target type
- For send (`0x0002`): Encodes enemy color, work mode, bullet speed

### Data Payload

Contains the actual data as IEEE 754 float values (4 bytes each, little-endian).

### Frame Tail

- **CRC16**: Checksum for the entire frame (excluding the CRC16 bytes themselves)

## Receiving Vision Data

The control board receives vision data with command ID `0x0001`.

### Receive Data Structure

```c
typedef struct {
    Fire_Mode_e fire_mode;      // Fire mode (NO_FIRE, AUTO_FIRE, AUTO_AIM)
    Target_State_e target_state; // Target state (NO_TARGET, TARGET_CONVERGING, READY_TO_FIRE)
    Target_Type_e target_type;  // Target type (HERO1, INFANTRY3, etc.)
    float pitch;                // Pitch angle (radians)
    float yaw;                  // Yaw angle (radians)
    uint8_t updated;            // Flag indicating data was updated
} Vision_Recv_s;
```

### Fire Modes

- `NO_FIRE = 0` - Do not fire
- `AUTO_FIRE = 1` - Automatic fire
- `AUTO_AIM = 2` - Automatic aim

### Target States

- `NO_TARGET = 0` - No target detected
- `TARGET_CONVERGING = 1` - Target is converging
- `READY_TO_FIRE = 2` - Ready to fire

### Target Types

- `NO_TARGET_NUM = 0` - No target
- `HERO1 = 1` - Hero robot
- `ENGINEER2 = 2` - Engineer robot
- `INFANTRY3 = 3` - Infantry robot (ID 3)
- `INFANTRY4 = 4` - Infantry robot (ID 4)
- `INFANTRY5 = 5` - Infantry robot (ID 5)
- `OUTPOST = 6` - Outpost
- `SENTRY = 7` - Sentry
- `BASE = 8` - Base

### Parsing Receive Data

When data is received via USB:

1. The USB receive callback (`VisionComm_RxCallback`) is called
2. The data is copied to a buffer
3. `get_protocol_info()` parses the Seasky protocol:
   - Validates frame header (SOF = 0xA5, CRC8 check)
   - Validates frame tail (CRC16 check)
   - Extracts command ID, flags register, and float data
4. If command ID is `0x0001`, the flags are parsed:
   - Bits 0-1: Fire mode
   - Bits 2-3: Target state
   - Bits 4-7: Target type
5. The parsed data is published to `TOPIC_VISION_DATA` via message center

## Sending Attitude Data

The control board sends attitude data with command ID `0x0002` at 100Hz (every 10ms).

### Send Data Structure

```c
typedef struct {
    Enemy_Color_e enemy_color;  // Enemy color (COLOR_NONE, COLOR_BLUE, COLOR_RED)
    Work_Mode_e work_mode;       // Work mode (VISION_MODE_AIM, etc.)
    Bullet_Speed_e bullet_speed; // Bullet speed (10, 15, 16, 18, 30 m/s)
    float yaw;                   // Yaw angle (radians)
    float pitch;                 // Pitch angle (radians)
    float roll;                  // Roll angle (radians)
} Vision_Send_s;
```

### Work Modes

- `VISION_MODE_AIM = 0` - Aiming mode
- `VISION_MODE_SMALL_BUFF = 1` - Small buff mode
- `VISION_MODE_BIG_BUFF = 2` - Big buff mode

### Bullet Speeds

- `BULLET_SPEED_NONE = 0` - No bullet speed set
- `BIG_AMU_10 = 10` - 10 m/s (big ammo)
- `SMALL_AMU_15 = 15` - 15 m/s (small ammo)
- `BIG_AMU_16 = 16` - 16 m/s (big ammo)
- `SMALL_AMU_18 = 18` - 18 m/s (small ammo)
- `SMALL_AMU_30 = 30` - 30 m/s (small ammo)

### Sending Process

1. The vision module subscribes to `TOPIC_IMU_UPDATE` to get attitude data
2. Every 10ms (100Hz), when IMU data is received:
   - Attitude data (yaw, pitch, roll) is set from IMU sensor
   - Flags (enemy color, work mode, bullet speed) are set
   - `VisionComm_Send()` is called
3. `get_protocol_send_data()` packs the data into Seasky protocol format:
   - Builds frame header with CRC8
   - Adds command ID `0x0002`
   - Adds flags register
   - Adds float data (yaw, pitch, roll = 3 floats = 12 bytes)
   - Calculates and adds CRC16 checksum
4. The packet is sent via USB using `CDC_Transmit_FS()`

## CRC Checksums

The protocol uses two types of checksums to ensure data integrity:

### CRC8

- Used for frame header validation
- Calculated over: SOF (1 byte) + Data Length (2 bytes)
- Stored in byte 3 of the frame header

### CRC16

- Used for entire frame validation
- Calculated over: Frame header (4 bytes) + Command ID (2 bytes) + Flags (2 bytes) + Data payload
- Stored in the last 2 bytes of the frame (little-endian)

If either checksum fails, the frame is rejected and `get_protocol_info()` returns 0.

## Usage Example

### Receiving Vision Data

```c
// Subscribe to vision data in your controller
static void on_vision_data(const MsgEvent *ev, void *user_data) {
    if (ev->size == sizeof(Vision_Recv_s)) {
        Vision_Recv_s *vision = (Vision_Recv_s *)ev->data;
        
        if (vision->updated) {
            // Use the vision data
            float target_pitch = vision->pitch;
            float target_yaw = vision->yaw;
            
            // Control gimbal based on vision data
            // ...
            
            vision->updated = 0;  // Clear flag
        }
    }
}

void MyController_Init(void) {
    MsgCenter_Subscribe(TOPIC_VISION_DATA, on_vision_data, NULL);
}
```

### Sending Attitude Data

```c
// Set flags before sending
VisionComm_SetFlag(COLOR_RED, VISION_MODE_AIM, SMALL_AMU_18);

// Set attitude data (usually done automatically from IMU)
VisionComm_SetAltitude(yaw, pitch, roll);

// Send (usually called automatically at 100Hz)
VisionComm_Send();
```

## Protocol Implementation

The Seasky protocol is implemented in:
- **`modules/vision_comm/seasky_protocol.c`** - Protocol packing/unpacking
- **`modules/vision_comm/crc8.c`** - CRC8 calculation
- **`modules/vision_comm/crc16.c`** - CRC16 calculation
- **`modules/vision_comm/vision_comm.c`** - High-level vision communication API

## Testing Vision Communication
### Test Script

**`script/test_vision_comm.py`** - A complete example of sending vision data using the Seasky protocol.

### Example: Sending Vision Data (Python)

The test script demonstrates how to send vision data:

```python
# From test_vision_comm.py
def send_vision_data(self, pitch, yaw, fire_mode=0, target_state=0, target_type=0):
    # Prepare data
    float_data = [pitch, yaw]
    data_length = 2 + len(float_data) * 4  # flags(2) + floats(8) = 10
    
    # Build flag register
    flags = (fire_mode & 0x03) | \
            ((target_state & 0x03) << 2) | \
            ((target_type & 0x0F) << 4)
    
    # Build frame header with CRC8
    header = bytearray(4)
    header[0] = 0xA5  # SOF
    header[1] = data_length & 0xFF
    header[2] = (data_length >> 8) & 0xFF
    header[3] = self._crc8(header[0:3])
    
    # Build packet: header + cmd_id + flags + float data
    packet = bytearray()
    packet.extend(header)
    packet.append(0x01)  # cmd_id = 0x0001 (low byte)
    packet.append(0x00)  # cmd_id (high byte)
    packet.append(flags & 0xFF)  # flags (low byte)
    packet.append((flags >> 8) & 0xFF)  # flags (high byte)
    
    # Add float data (little endian)
    for f in float_data:
        packet.extend(struct.pack('<f', f))
    
    # Calculate and add CRC16
    crc16 = self._crc16(packet)
    packet.append(crc16 & 0xFF)
    packet.append((crc16 >> 8) & 0xFF)
    
    # Send via serial port
    self.ser.write(packet)
```