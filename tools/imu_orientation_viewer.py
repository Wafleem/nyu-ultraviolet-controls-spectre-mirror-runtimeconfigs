#!/usr/bin/env python3
import argparse
import math
import re
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation


COMPACT_RE = re.compile(
    r"A:\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*"
    r"G:\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*"
    r"(?:M:\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*)?"
    r"(?:E:\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?))?"
)

ACCEL_RE = re.compile(r"Accel:\s*X:\s*(-?\d+)\s*Y:\s*(-?\d+)\s*Z:\s*(-?\d+)")
GYRO_RE = re.compile(r"Gyro:\s*X:\s*(-?\d+)\s*Y:\s*(-?\d+)\s*Z:\s*(-?\d+)")
MAG_RE = re.compile(r"Mag:\s*X:\s*(-?\d+)\s*Y:\s*(-?\d+)\s*Z:\s*(-?\d+)")
EKF_RE = re.compile(
    r"EKF:\s*Roll:\s*(-?\d+(?:\.\d+)?)\s*Pitch:\s*(-?\d+(?:\.\d+)?)\s*Yaw:\s*(-?\d+(?:\.\d+)?)"
)


def parse_line(line: str, state: dict):
    m = COMPACT_RE.search(line)
    if m:
        ax, ay, az, gx, gy, gz = map(int, m.groups()[:6])
        if m.group(7) is not None:
            mx = int(m.group(7))
            my = int(m.group(8))
            mz = int(m.group(9))
        else:
            mx = 0
            my = 0
            mz = 0

        if m.group(10) is not None:
            ekf_roll = float(m.group(10))
            ekf_pitch = float(m.group(11))
            ekf_yaw = float(m.group(12))
        else:
            ekf_roll = 0.0
            ekf_pitch = 0.0
            ekf_yaw = 0.0

        return ax, ay, az, gx, gy, gz, mx, my, mz, ekf_roll, ekf_pitch, ekf_yaw

    m = ACCEL_RE.search(line)
    if m:
        state["ax"], state["ay"], state["az"] = map(int, m.groups())

    m = GYRO_RE.search(line)
    if m:
        state["gx"], state["gy"], state["gz"] = map(int, m.groups())

    m = MAG_RE.search(line)
    if m:
        state["mx"], state["my"], state["mz"] = map(int, m.groups())

    m = EKF_RE.search(line)
    if m:
        state["ekf_roll"], state["ekf_pitch"], state["ekf_yaw"] = map(float, m.groups())
        required = ("ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz")
        if all(k in state for k in required):
            return (
                state["ax"], state["ay"], state["az"],
                state["gx"], state["gy"], state["gz"],
                state["mx"], state["my"], state["mz"],
                state["ekf_roll"], state["ekf_pitch"], state["ekf_yaw"],
            )

    return None


def euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


BOX_VERTICES = np.array(
    [
        [-2.0, -1.0, -0.5],
        [2.0, -1.0, -0.5],
        [2.0, 1.0, -0.5],
        [-2.0, 1.0, -0.5],
        [-2.0, -1.0, 0.5],
        [2.0, -1.0, 0.5],
        [2.0, 1.0, 0.5],
        [-2.0, 1.0, 0.5],
    ],
    dtype=float,
)

BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def rotate_box(rot: np.ndarray) -> np.ndarray:
    return (rot @ BOX_VERTICES.T).T


def main():
    parser = argparse.ArgumentParser(description="Live 3D IMU orientation viewer from serial")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--gyro-lsb-per-dps", type=float, default=16.4, help="Gyro LSB per deg/s")
    parser.add_argument("--plot-hz", type=float, default=100.0, help="Plot update frequency in Hz")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as exc:
        print(f"Failed to open serial port: {exc}", file=sys.stderr)
        sys.exit(1)

    gyro_roll = 0.0
    gyro_pitch = 0.0
    gyro_yaw = 0.0
    ekf_roll = 0.0
    ekf_pitch = 0.0
    ekf_yaw = 0.0
    last_t = time.time()

    fig = plt.figure("IMU Orientation: Gyro vs EKF")
    ax3d_gyro = fig.add_subplot(121, projection="3d")
    ax3d_ekf = fig.add_subplot(122, projection="3d")

    def setup_axis(ax3d, title):
        ax3d.set_xlim(-2.5, 2.5)
        ax3d.set_ylim(-1.5, 1.5)
        ax3d.set_zlim(-1.0, 1.0)
        ax3d.set_xlabel("X")
        ax3d.set_ylabel("Y")
        ax3d.set_zlabel("Z")
        ax3d.set_box_aspect((4, 2, 1))
        ax3d.set_title(title)

    setup_axis(ax3d_gyro, "Gyro-only Orientation")
    setup_axis(ax3d_ekf, "Firmware EKF Orientation")

    box_lines_g = [ax3d_gyro.plot([0, 0], [0, 0], [0, 0], color="tab:blue", linewidth=2)[0] for _ in BOX_EDGES]
    box_lines_e = [ax3d_ekf.plot([0, 0], [0, 0], [0, 0], color="tab:orange", linewidth=2)[0] for _ in BOX_EDGES]

    info = fig.text(0.02, 0.02, "", ha="left", va="bottom", fontsize=10)

    latest_raw = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0)
    parse_state = {}

    def update(_frame):
        nonlocal gyro_roll, gyro_pitch, gyro_yaw
        nonlocal ekf_roll, ekf_pitch, ekf_yaw
        nonlocal last_t
        nonlocal box_lines_g, box_lines_e
        nonlocal latest_raw

        while ser.in_waiting:
            raw = ser.readline().decode(errors="ignore").strip()
            parsed = parse_line(raw, parse_state)
            if parsed is None:
                continue

            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, mx_raw, my_raw, mz_raw, ekf_r_deg, ekf_p_deg, ekf_y_deg = parsed
            latest_raw = parsed

            now = time.time()
            dt = max(1e-3, min(0.2, now - last_t))
            last_t = now

            gx = math.radians(gx_raw / args.gyro_lsb_per_dps)
            gy = math.radians(gy_raw / args.gyro_lsb_per_dps)
            gz = math.radians(gz_raw / args.gyro_lsb_per_dps)

            gyro_roll += gx * dt
            gyro_pitch += gy * dt
            gyro_yaw += gz * dt

            ekf_roll = math.radians(ekf_r_deg)
            ekf_pitch = math.radians(ekf_p_deg)
            ekf_yaw = math.radians(ekf_y_deg)

        rotated_gyro = rotate_box(euler_to_rot(gyro_roll, gyro_pitch, gyro_yaw))
        rotated_ekf = rotate_box(euler_to_rot(ekf_roll, ekf_pitch, ekf_yaw))

        for i, (v0, v1) in enumerate(BOX_EDGES):
            pg0, pg1 = rotated_gyro[v0], rotated_gyro[v1]
            box_lines_g[i].set_data_3d([pg0[0], pg1[0]], [pg0[1], pg1[1]], [pg0[2], pg1[2]])

            pe0, pe1 = rotated_ekf[v0], rotated_ekf[v1]
            box_lines_e[i].set_data_3d([pe0[0], pe1[0]], [pe0[1], pe1[1]], [pe0[2], pe1[2]])

        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, mx_raw, my_raw, mz_raw, ekf_r_deg, ekf_p_deg, ekf_y_deg = latest_raw
        info.set_text(
            f"Gyro-only [deg]: R={math.degrees(gyro_roll):7.1f} P={math.degrees(gyro_pitch):7.1f} Y={math.degrees(gyro_yaw):7.1f} | "
            f"EKF [deg]: R={ekf_r_deg:7.1f} P={ekf_p_deg:7.1f} Y={ekf_y_deg:7.1f}\n"
            f"A=[{ax_raw:5d},{ay_raw:5d},{az_raw:5d}]  G=[{gx_raw:5d},{gy_raw:5d},{gz_raw:5d}]  M=[{mx_raw:5d},{my_raw:5d},{mz_raw:5d}]"
        )

        return (*box_lines_g, *box_lines_e, info)

    interval_ms = max(1, int(1000.0 / max(1.0, args.plot_hz)))
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    _ = ani
    plt.show()


if __name__ == "__main__":
    main()
