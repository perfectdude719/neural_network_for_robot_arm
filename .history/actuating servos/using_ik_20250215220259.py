import ik_with_collision_avoidance
import numpy as np
import pygame
import serial
import time

# === Configuration ===
SERIAL_PORT = 'COM9'  # Change as needed (e.g., 'COM3' on Windows)
BAUD_RATE = 9600
UPDATE_DELAY = 0.05      # Seconds between updates
EXECUTION_DELAY = 1.0    # Delay after executing a solution (in seconds)
STEP_SIZE = 1.0          # Increment (in degrees) for gradual servo movement
ALPHA = 0.25             # Low pass filter constant (0 < ALPHA <= 1)
JOYSTICK_DEADZONE = 0.1  # Joystick values below this are ignored to avoid noise
X_AXIS_RANGE = (-250, 250)  # Desired x-axis range
Y_AXIS_RANGE = (-100, 100)  # Desired y-axis range
Z_AXIS_RANGE = (-250, 250)  # Desired z-axis range

# === Serial Setup ===
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Opened serial port: {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print("Error opening serial port:", e)
    exit(1)

# === Pygame and Joystick Initialization ===
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a PS4 controller.")
    pygame.quit()
    exit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Joystick connected:", joystick.get_name())

# === Variables for Desired Pose ===
desired_pose = np.array([0.0, 0.0, 0.0])  # Initial desired pose (x, y, z)
filtered_axis_x = joystick.get_axis(0)  # Initialize filtered axis for x
filtered_axis_y = joystick.get_axis(1)  # Initialize filtered axis for y
filtered_axis_z = joystick.get_axis(2)  # Initialize filtered axis for z

def map_joystick_to_range(joystick_value, axis_range):
    """
    Map joystick value (-1.0 to 1.0) to a specified range and clamp the result.
    """
    # Scale the joystick value to the desired range
    mapped_value = joystick_value * (axis_range[1] - axis_range[0]) / 2.0
    # Clamp the mapped value to the range
    clamped_value = max(axis_range[0], min(axis_range[1], mapped_value))
    return clamped_value

def send_joint_angles_to_serial(joint_angles):
    """
    Send joint angles to the Arduino via serial port.
    """
    message = ",".join(map(str, joint_angles)) + "\n"
    ser.write(message.encode('utf-8'))
    print(f"Sent joint angles to serial: {message.strip()}")

# Example usage
n = 7  # Number of links
alpha = np.array([90, 0, 0, 90, -90, -90, 0])  # Link twist angles in degrees
a = np.array([0, 218, 196, 36, 0, 7.5, 11])  # Link lengths
d = np.array([147, 0, 0, 0, 425, 0, 0])  # Link offsets

# Define obstacles (x, y, z positions)
obstacles = [np.array([200, 0, 0]), np.array([0, 200, 0])]

try:
    while True:
        # Process pygame events (to keep internal states updated)
        for event in pygame.event.get():
            pass

        # Read raw joystick axes
        raw_axis_x = joystick.get_axis(0)  # X-axis (left stick horizontal)
        raw_axis_y = joystick.get_axis(1)  # Y-axis (left stick vertical)
        raw_axis_z = joystick.get_axis(2)  # Z-axis (right stick vertical)

        # Apply low-pass filtering to smooth out noise
        filtered_axis_x = ALPHA * raw_axis_x + (1 - ALPHA) * filtered_axis_x
        filtered_axis_y = ALPHA * raw_axis_y + (1 - ALPHA) * filtered_axis_y
        filtered_axis_z = ALPHA * raw_axis_z + (1 - ALPHA) * filtered_axis_z

        # Update desired pose based on joystick input
        if abs(filtered_axis_x) > JOYSTICK_DEADZONE:
            desired_pose[0] = map_joystick_to_range(filtered_axis_x, X_AXIS_RANGE)
        if abs(filtered_axis_y) > JOYSTICK_DEADZONE:
            desired_pose[1] = map_joystick_to_range(filtered_axis_y, Y_AXIS_RANGE)
        if abs(filtered_axis_z) > JOYSTICK_DEADZONE:
            desired_pose[2] = map_joystick_to_range(filtered_axis_z, Z_AXIS_RANGE)

        # For debugging: print joystick values and computed desired pose
        print(f"Raw X: {raw_axis_x:.2f}, Filtered X: {filtered_axis_x:.2f}, Desired X: {desired_pose[0]:.2f}")
        print(f"Raw Y: {raw_axis_y:.2f}, Filtered Y: {filtered_axis_y:.2f}, Desired Y: {desired_pose[1]:.2f}")
        print(f"Raw Z: {raw_axis_z:.2f}, Filtered Z: {filtered_axis_z:.2f}, Desired Z: {desired_pose[2]:.2f}")

        # Run IK with collision avoidance
        joint_angles = ik_with_collision_avoidance.inverse_kinematics_with_collision_avoidance(
            desired_pose, n, alpha, a, d, obstacles
        )
        if joint_angles is not None:
            print("Collision-free joint angles:", joint_angles)
            # Plot the robot
            ik_with_collision_avoidance.plot_robot(joint_angles, alpha, a, d, obstacles, desired_pose)
            # Send joint angles to serial port
            send_joint_angles_to_serial(joint_angles)
            # Add a delay to allow the robot to execute the solution
            time.sleep(EXECUTION_DELAY)

        # Wait before the next update
        time.sleep(UPDATE_DELAY)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    ser.close()
    pygame.quit()