import math
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

# ================================
# USER SETTINGS
# ================================

# Change this to your ESP32 port.
# Windows examples: "COM3", "COM4", "COM5"
# Mac examples: "/dev/tty.usbserial-0001", "/dev/tty.SLAB_USBtoUART"
# Linux examples: "/dev/ttyUSB0", "/dev/ttyACM0"
SERIAL_PORT = "COM3"

BAUD_RATE = 115200

# Complementary filter value
# Higher alpha trusts gyro more.
# Lower alpha trusts accelerometer more.
ALPHA = 0.98

# Plot update speed
ANIMATION_INTERVAL_MS = 20


# ================================
# STATE VARIABLES
# ================================

roll = 0.0
pitch = 0.0
yaw = 0.0

last_time = time.time()

latest_values = {
    "ax": 0.0,
    "ay": 0.0,
    "az": 1.0,
    "gx": 0.0,
    "gy": 0.0,
    "gz": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}


# ================================
# SERIAL SETUP
# ================================

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
time.sleep(2.0)
ser.reset_input_buffer()


# ================================
# ROTATION HELPERS
# ================================


def deg2rad(x):
    return x * math.pi / 180.0


def rotation_matrix_from_euler(roll_deg, pitch_deg, yaw_deg):
    """
    Roll  = rotation about X
    Pitch = rotation about Y
    Yaw   = rotation about Z

    Returns R = Rz * Ry * Rx
    """
    r = deg2rad(roll_deg)
    p = deg2rad(pitch_deg)
    y = deg2rad(yaw_deg)

    Rx = np.array(
        [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]]
    )

    Ry = np.array(
        [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]]
    )

    Rz = np.array(
        [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]]
    )

    return Rz @ Ry @ Rx


def update_attitude_from_imu(ax, ay, az, gx, gy, gz):
    """
    ax, ay, az in g
    gx, gy, gz in deg/s
    """
    global roll, pitch, yaw, last_time

    now = time.time()
    dt = now - last_time
    last_time = now

    if dt <= 0.0 or dt > 0.1:
        dt = 0.005

    # Accelerometer angle estimate
    # These formulas assume sensor axes are approximately:
    # X forward/back, Y left/right, Z up/down
    roll_acc = math.degrees(math.atan2(ay, az))
    pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

    # Gyro integration
    roll_gyro = roll + gx * dt
    pitch_gyro = pitch + gy * dt
    yaw_gyro = yaw + gz * dt

    # Complementary filter for roll and pitch
    roll = ALPHA * roll_gyro + (1.0 - ALPHA) * roll_acc
    pitch = ALPHA * pitch_gyro + (1.0 - ALPHA) * pitch_acc

    # MPU6050 has no magnetometer, so yaw will drift
    yaw = yaw_gyro

    latest_values["roll"] = roll
    latest_values["pitch"] = pitch
    latest_values["yaw"] = yaw


def read_serial_line():
    """
    Reads one valid CSV line from ESP32:
    ax,ay,az,gx,gy,gz
    """
    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            return None

        if line.startswith("ax_g") or line.startswith("ERROR"):
            return None

        parts = line.split(",")

        if len(parts) != 6:
            return None

        ax, ay, az, gx, gy, gz = [float(p) for p in parts]

        return ax, ay, az, gx, gy, gz

    except ValueError:
        return None
    except serial.SerialException:
        return None


# ================================
# DRONE MODEL GEOMETRY
# ================================

# Simple X-frame quadrotor in body coordinates
arm_length = 1.0
motor_height = 0.0

body_points = np.array(
    [
        [arm_length, arm_length, motor_height],
        [-arm_length, -arm_length, motor_height],
        [-arm_length, arm_length, motor_height],
        [arm_length, -arm_length, motor_height],
    ]
)

# Arms:
# Arm 1 connects motor 0 to motor 1
# Arm 2 connects motor 2 to motor 3
arm1_body = np.array(
    [
        [arm_length, arm_length, motor_height],
        [-arm_length, -arm_length, motor_height],
    ]
)

arm2_body = np.array(
    [
        [-arm_length, arm_length, motor_height],
        [arm_length, -arm_length, motor_height],
    ]
)

# Body direction arrow, showing "front"
front_arrow_body = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.4, 0.0, 0.0],
    ]
)

# Body vertical axis arrow
up_arrow_body = np.array(
    [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
)


def rotate_points(points, R):
    return points @ R.T


# ================================
# PLOT SETUP
# ================================

fig = plt.figure()
ax3d = fig.add_subplot(111, projection="3d")

ax3d.set_title("Live MPU6050 Drone Attitude Visualizer")

ax3d.set_xlim([-2, 2])
ax3d.set_ylim([-2, 2])
ax3d.set_zlim([-2, 2])

ax3d.set_xlabel("X")
ax3d.set_ylabel("Y")
ax3d.set_zlabel("Z")

# Initial lines
(arm1_line,) = ax3d.plot([], [], [], linewidth=4)
(arm2_line,) = ax3d.plot([], [], [], linewidth=4)
(front_line,) = ax3d.plot([], [], [], linewidth=3)
(up_line,) = ax3d.plot([], [], [], linewidth=2)

# Motor points
motor_scatter = ax3d.scatter([], [], [], s=80)

# Text info
info_text = ax3d.text2D(0.02, 0.95, "", transform=ax3d.transAxes)


def set_line_from_points(line, points):
    line.set_data(points[:, 0], points[:, 1])
    line.set_3d_properties(points[:, 2])


def animation_update(frame):
    # Read multiple lines per frame so the visualizer uses the newest data
    for _ in range(10):
        data = read_serial_line()
        if data is None:
            continue

        ax_imu, ay_imu, az_imu, gx_imu, gy_imu, gz_imu = data

        latest_values["ax"] = ax_imu
        latest_values["ay"] = ay_imu
        latest_values["az"] = az_imu
        latest_values["gx"] = gx_imu
        latest_values["gy"] = gy_imu
        latest_values["gz"] = gz_imu

        update_attitude_from_imu(ax_imu, ay_imu, az_imu, gx_imu, gy_imu, gz_imu)

    R = rotation_matrix_from_euler(
        latest_values["roll"], latest_values["pitch"], latest_values["yaw"]
    )

    arm1_world = rotate_points(arm1_body, R)
    arm2_world = rotate_points(arm2_body, R)
    motors_world = rotate_points(body_points, R)
    front_world = rotate_points(front_arrow_body, R)
    up_world = rotate_points(up_arrow_body, R)

    set_line_from_points(arm1_line, arm1_world)
    set_line_from_points(arm2_line, arm2_world)
    set_line_from_points(front_line, front_world)
    set_line_from_points(up_line, up_world)

    # Update scatter points
    motor_scatter._offsets3d = (
        motors_world[:, 0],
        motors_world[:, 1],
        motors_world[:, 2],
    )

    info_text.set_text(
        f"Roll:  {latest_values['roll']:.2f} deg\n"
        f"Pitch: {latest_values['pitch']:.2f} deg\n"
        f"Yaw:   {latest_values['yaw']:.2f} deg\n\n"
        f"Ax: {latest_values['ax']:.3f} g\n"
        f"Ay: {latest_values['ay']:.3f} g\n"
        f"Az: {latest_values['az']:.3f} g\n"
        f"Gx: {latest_values['gx']:.3f} deg/s\n"
        f"Gy: {latest_values['gy']:.3f} deg/s\n"
        f"Gz: {latest_values['gz']:.3f} deg/s"
    )

    return arm1_line, arm2_line, front_line, up_line, motor_scatter, info_text


ani = FuncAnimation(
    fig,
    animation_update,
    interval=ANIMATION_INTERVAL_MS,
    blit=False,
    cache_frame_data=False,
)

try:
    plt.show()
finally:
    ser.close()
