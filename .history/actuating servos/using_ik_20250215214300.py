import ik_with_collision_avoidance
import numpy as np
import pygame
import serial
import time

# === Configuration ===
SERIAL_PORT = 'COM9'  # Change as needed (e.g., 'COM3' on Windows)
BAUD_RATE = 9600
UPDATE_DELAY = 0.05      # Seconds between updates
STEP_SIZE = 1.0          # Increment (in degrees) for gradual servo movement
ALPHA = 0.25             # Low pass filter constant (0 < ALPHA <= 1)
JOYSTICK_DEADZONE = 0.1  # Joystick values below this are ignored to avoid noise
X_AXIS_RANGE = (-250, 250)  # Desired x-axis range

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
desired_pose = np.array([0.0, 100.0, 200.0])  # Initial desired pose (x, y, z)
filtered_axis = joystick.get_axis(0)  # Initialize filtered axis with joystick reading
latched_x = desired_pose[0]  # Latched x-axis value

def map_joystick_to_x_axis(joystick_value):
    """
    Map joystick value (-1.0 to 1.0) to x-axis range (-250 to 250).
    """
    return joystick_value * (X_AXIS_RANGE[1] - X_AXIS_RANGE[0]) / 2.0

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

        # Read raw joystick axis (e.g., left stick horizontal axis at index 0)
        raw_axis_value = joystick.get_axis(0)

        # Apply low-pass filtering to smooth out noise.
        filtered_axis = ALPHA * raw_axis_value + (1 - ALPHA) * filtered_axis

        # If the joystick movement is beyond the deadzone, update the latched x-axis.
        if abs(filtered_axis) > JOYSTICK_DEADZONE:
            # Compute the change in x-axis based on joystick input
            delta_x = filtered_axis * STEP_SIZE
            latched_x += delta_x
            # Clamp the latched x-axis value to the valid range
            latched_x = max(X_AXIS_RANGE[0], min(X_AXIS_RANGE[1], latched_x))
            desired_pose[0] = map_joystick_to_x_axis(latched_x)  # Update x-axis of desired pose
        else:
            delta_x = 0  # No change when within the deadzone.

        # For debugging: print joystick values and computed x-axis.
        print(f"Raw: {raw_axis_value:.2f}, Filtered: {filtered_axis:.2f}, "
              f"Delta X: {delta_x:.2f}, Latched X: {latched_x:.2f}")

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

        # Wait before the next update.
        time.sleep(UPDATE_DELAY)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    ser.close()
    pygame.quit()