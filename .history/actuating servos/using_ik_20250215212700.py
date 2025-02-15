from inverse_kinematics_for_robosurgeon import ik_with_collision_avoidance
import numpy as np

import pygame
import serial
import time

# === Configuration ===
SERIAL_PORT = 'COM9'  # Change as needed (e.g., 'COM3' on Windows)
BAUD_RATE = 9600
UPDATE_DELAY = 0.05      # Seconds between updates
STEP_SIZE = 10          # Increment (in degrees) for gradual servo movement
ALPHA = 0.25               # Low pass filter constant (0 < ALPHA <= 1)
JOYSTICK_DEADZONE = 0.1   # Joystick values below this are ignored to avoid noise
DELTA_SENSITIVITY = 2.0   # Degrees to change per update at full joystick deflection

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

# === Variables for Servo Angle ===
current_angle = 135  # Starting angle (e.g., mid-point)
target_angle = current_angle  # This value is "latched" and updated incrementally
# Initialize the filtered axis with the current joystick reading.
filtered_axis = joystick.get_axis(0)

def update_angle(current, target, step):
    """
    Gradually move the current angle toward the target angle.
    """
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current


# Example usage
n = 7  # Number of links
alpha = np.array([90, 0, 0, 90, -90, -90, 0])  # Link twist angles in degrees
a = np.array([0, 218, 196, 36, 0, 7.5, 11])  # Link lengths
d = np.array([147, 0, 0, 0, 425, 0, 0])  # Link offsets

# Define obstacles (x, y, z positions)
obstacles = [np.array([200, 0, 0]), np.array([0, 200, 0])]

# Desired end-effector position
desired_pose = np.array([300, 100, 200])

# Run IK with collision avoidance
joint_angles = inverse_kinematics_with_collision_avoidance(desired_pose, n, alpha, a, d, obstacles)
if joint_angles is not None:
    print("Collision-free joint angles:", joint_angles)
    # Plot the robot
    plot_robot(joint_angles, alpha, a, d, obstacles, desired_pose)
    
try:
    while True:
        # Process pygame events (to keep internal states updated)
        for event in pygame.event.get():
            pass

        # Read raw joystick axis (e.g., left stick horizontal axis at index 0)
        raw_axis_value = joystick.get_axis(0)

        # Apply low-pass filtering to smooth out noise.
        filtered_axis = ALPHA * raw_axis_value + (1 - ALPHA) * filtered_axis

        # If the joystick movement is beyond the deadzone, update the target angle.
        if abs(filtered_axis) > JOYSTICK_DEADZONE:
            # Compute a relative change from the filtered value.
            delta = filtered_axis * DELTA_SENSITIVITY
            target_angle += delta
            # Clamp the target angle to the valid range [0, 270].
            target_angle = max(0, min(270, target_angle))
        else:
            delta = 0  # No change when within the deadzone.

        # For debugging: print joystick values and computed angles.
        print(f"Raw: {raw_axis_value:.2f}, Filtered: {filtered_axis:.2f}, "
              f"Delta: {delta:.2f}, Target: {target_angle}")

        # Gradually update the servo's current angle toward the target angle.
        current_angle = update_angle(current_angle, target_angle, STEP_SIZE)

        # Send the current angle over serial to the Arduino.
        message = f"{current_angle}\n"
        ser.write(message.encode('utf-8'))
        print(f"Sending angle: {current_angle}")

        # Wait before the next update.
        time.sleep(UPDATE_DELAY)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    ser.close()
    pygame.quit()
