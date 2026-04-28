# Logger System Quick Start Guide

This guide explains how to use the unified logger system for debugging and data logging.

## Overview

The logger system provides:
- **12 log tags** for different subsystems (SYS, CMD, CHA, GIM, SHO, SEN, MOT, IMU, CAN, VIS, RC, DEBUG)
- **5 log levels** (ERROR, WARN, INFO, DEBUG, CSV)
- **Automatic rate limiting** to prevent USB CDC overflow
- **Compile-time enable/disable** for tags you don't need
- **CSV format** for data logging with automatic timestamping

## Firmware Usage

### Including the Logger

```c
#include "logger.h"
```

### Text Logging

```c
// System messages
LOG_INFO(LOG_TAG_SYS, "System initialized");
LOG_INFO(LOG_TAG_SYS, "CAN bus ready");

// Error messages
LOG_ERROR(LOG_TAG_MOT, "Motor %d not responding", motor_id);
LOG_WARN(LOG_TAG_CAN, "TX mailbox full");

// Debug messages (verbose, can be disabled)
LOG_DEBUG(LOG_TAG_YAW, "Yaw angle: %.2f degrees", yaw_angle);
```

Output format: `[TAG][LEVEL] message`
```
[SYS][INFO] System initialized
[MOT][ERROR] Motor 5 not responding
[GIM][DEBUG] Yaw angle: 45.20 degrees
```

### CSV Data Logging

For real-time data visualization and PID tuning:

```c
// Gimbal PID tuning
LOG_CSV(LOG_TAG_PITCH, "PITCH,%.2f,%.2f,%d,%.2f",
        angle_target, angle_current, speed_rpm, pid_output);

// IMU sensor data
LOG_CSV(LOG_TAG_IMU, "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f",
        yaw, pitch, roll, gyro_x, gyro_y, gyro_z);

// Command controller
LOG_CSV(LOG_TAG_CMD, "%u,%.2f,%.2f,%.2f",
        mode, vx_cmd, vy_cmd, wz_cmd);
```

Output format: `TAG,timestamp_ms,field1,field2,...`
```
GIM,123456,PITCH,45.20,44.80,150,12.50
IMU,123500,45.20,30.10,0.00,0.001,0.002,0.003
CMD,123600,1,0.50,0.30,0.00
```

**Note**: Timestamps are automatically added by the logger. Don't include them in your format string!

### Rate Limiting

Configure rate limits in `main.c` after `Logger_Init()`:

```c
Logger_Init();

// Configure rates (milliseconds between logs)
Logger_SetRate(LOG_TAG_CMD, 100);   // 10Hz
Logger_SetRate(LOG_TAG_IMU, 100);   // 10Hz
Logger_SetRate(LOG_TAG_YAW, 50);    // 20Hz (fast for PID tuning)
Logger_SetRate(LOG_TAG_SYS, 0);     // No limit (boot messages)
```

The logger will automatically skip logs that come too quickly based on these limits.

## Configuration

### Compile-Time Enable/Disable

Edit `modules/logger/logger_config.h`:

```c
// Enable only the tags you need
#define LOG_ENABLE_SYS    1  // System messages (keep enabled)
#define LOG_ENABLE_CMD    1  // Command controller
#define LOG_ENABLE_YAW    1  // Gimbal (for PID tuning)
#define LOG_ENABLE_IMU    1  // IMU data
#define LOG_ENABLE_CAN    0  // Disable CAN (too verbose)
#define LOG_ENABLE_DEBUG  0  // Disable debug in production
```

Disabled tags compile to no-ops (zero overhead).

### Buffer Size

Default buffer size is 256 bytes. If you have very long CSV lines, increase it in `logger_config.h`:

```c
#define LOG_BUFFER_SIZE    512  // For longer CSV lines
```

## Python Smart Logger Tool

### Installation

Ensure matplotlib is installed:
```bash
pip3 install matplotlib pyserial
```

### Basic Usage

```bash
# Auto-detect port, monitor all tags
python3 script/smart_logger.py

# Monitor specific tags
python3 script/smart_logger.py --tags GIM,IMU

# Monitor gimbal only (no IMU noise)
python3 script/smart_logger.py --tags GIM

# Save data without plotting (for later analysis)
python3 script/smart_logger.py --save tuning_data.csv --no-plot

# List all available tags
python3 script/smart_logger.py --list-tags
```

### Advanced Usage

```bash
# Specify port manually
python3 script/smart_logger.py /dev/ttyACM0 --tags GIM

# Change baud rate (default 115200)
python3 script/smart_logger.py --baud 921600 --tags all

# Monitor multiple tags with plotting
python3 script/smart_logger.py --tags CMD,GIM,IMU
```

### Tag Auto-Detection

The smart logger automatically detects CSV sub-formats:

**GIM tag** detects:
- `PITCH` format (6 fields): angle_target, angle_current, speed_rpm, cmd, error, rate
- `YAW` format (9 fields): angle_target, angle_current, speed_rpm, cmd_current, cmd_speed, rate, error, g_gz, c_gz
- `ENCODER` format (4 fields): yaw_raw, pitch_raw, yaw_target, pitch_target

**IMU tag** detects:
- Gimbal IMU (8 fields) + Chassis IMU (9 fields) = 17 total fields

**CMD tag** detects:
- Spin mode debug (11 fields)

## Common Workflows

### Gimbal PID Tuning

1. **Enable gimbal logging** in firmware:
   ```c
   // In gimbal_controller.c (already implemented)
   LOG_CSV(LOG_TAG_PITCH, "PITCH,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
           angle_target, angle_current, speed_rpm, cmd, error, rate);
   ```

2. **Flash firmware** with gimbal logging enabled

3. **Run smart logger**:
   ```bash
   python3 script/smart_logger.py --tags GIM
   ```

4. **Observe real-time plots**:
   - Target vs current angle (check tracking)
   - Speed (check responsiveness)
   - Error (check convergence)
   - Output command (check saturation)

5. **Adjust PID gains** in robot config and repeat

### IMU Calibration Check

```bash
# Monitor IMU data
python3 script/smart_logger.py --tags IMU

# Save to file for analysis
python3 script/smart_logger.py --tags IMU --save imu_calibration.csv
```

Check for:
- Gyro drift (should be near zero when stationary)
- Yaw accumulation accuracy
- Chassis vs gimbal alignment

### Spin Mode Debug

```bash
# Monitor command controller spin mode
python3 script/smart_logger.py --tags CMD
```

Plots show:
- Spin mode active/inactive
- Yaw error during spin
- Chassis velocity commands (vx, vy, wz)

## Migrating from Old Scripts

| Old Script | New Command |
|-----------|-------------|
| `plot_pitch.py` | `smart_logger.py --tags GIM` |
| `plot_yaw_data.py` | `smart_logger.py --tags GIM` |
| `read_imu.py --plot` | `smart_logger.py --tags IMU` |
| `read_dual_imu.py --plot` | `smart_logger.py --tags IMU` |
| `plot_cmd.py` | `smart_logger.py --tags CMD` |
| `plot_chassis_current.py` | `smart_logger.py --tags CHA` |

All old scripts are in `script/deprecated/` for reference.

## Troubleshooting

### No data appearing

1. Check tag is enabled in `logger_config.h`
2. Rebuild firmware after config changes
3. Verify serial port connection: `ls /dev/tty*`
4. Check baud rate matches (default 115200)

### Data too slow/fast

1. Adjust rate limit in `main.c`:
   ```c
   Logger_SetRate(LOG_TAG_YAW, 50);  // Faster (20Hz)
   Logger_SetRate(LOG_TAG_YAW, 200); // Slower (5Hz)
   ```

2. Rebuild and reflash firmware

### USB CDC buffer overflow

Symptoms: Garbled output, missing lines

Solutions:
1. Disable verbose tags (CAN, DEBUG) in `logger_config.h`
2. Increase rate limits (slower logging)
3. Monitor fewer tags simultaneously

### Plot not updating

1. Ensure matplotlib is installed: `pip3 install matplotlib`
2. Check data format matches expected (run with `--no-plot` first to see raw data)
3. Try `--tags all` to verify any tags are working

## Best Practices

1. **Use appropriate log levels**:
   - `ERROR` - Critical failures only
   - `WARN` - Unexpected but recoverable
   - `INFO` - Important state changes
   - `DEBUG` - Verbose diagnostic data
   - `CSV` - High-frequency sensor/control data

2. **Rate limit aggressively**:
   - 5-20Hz is usually sufficient for visualization
   - 100Hz+ can overflow USB CDC buffer

3. **Disable unused tags**:
   - Reduces flash usage
   - Improves performance
   - Cleaner output

4. **Use CSV for data, text for messages**:
   - CSV: Sensor readings, PID outputs, motor speeds
   - Text: Initialization status, errors, warnings

5. **Save important data**:
   ```bash
   # Record tuning session
   smart_logger.py --tags GIM --save gimbal_tune_$(date +%Y%m%d_%H%M).csv
   ```

## Examples

### Complete PID Tuning Session

```bash
# Terminal 1: Flash firmware
dfu-util -a 0 -s 0x08000000:leave -D build/Spectre_Controls.bin

# Terminal 2: Monitor gimbal
python3 script/smart_logger.py --tags GIM --save gimbal_session.csv

# Operate robot, observe plots, adjust gains, repeat
```

### Multi-Subsystem Debug

```bash
# Monitor command, gimbal, and IMU simultaneously
python3 script/smart_logger.py --tags CMD,GIM,IMU
```

You'll see:
- CMD plot: Spin mode state and velocity commands
- GIM plot: Pitch/yaw control performance
- IMU plot: Gimbal and chassis orientation

## Summary

The logger system provides a powerful, flexible debugging infrastructure:

✅ **C Firmware**: Use `LOG_*` macros with appropriate tags and levels
✅ **Python Tool**: Use `smart_logger.py` with `--tags` to filter
✅ **Configuration**: Edit `logger_config.h` to enable/disable tags
✅ **Rate Control**: Set limits in `main.c` to prevent overflow

Happy debugging! 🚀
