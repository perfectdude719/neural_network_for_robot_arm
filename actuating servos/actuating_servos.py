import pygame
import serial
import time

# === Configuration ===
# Replace with your Arduino's serial port.
SERIAL_PORT = 'COM9'  # e.g., 'COM3' for Windows or '/dev/ttyUSB0' for Linux
BAUD_RATE = 9600
UPDATE_DELAY = 0.01  # seconds between updates
STEP_SIZE = 1        # incremental step for angle update

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
current_angle = 135  # Start at mid-point (135 degrees)
target_angle = current_angle

def scale_axis_to_angle(axis_value):
    """
    Scale the axis value (-1 to 1) to an angle (0 to 270).
    """
    # Transform axis_value from [-1, 1] to [0, 1] and then scale to 270.
    angle = int(((axis_value + 1) / 2) * 270)
    return angle

def update_angle(current, target, step):
    """
    Incrementally update the current angle toward the target angle.
    """
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

try:
    while True:
        # Process any pygame events (e.g., joystick movement)
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # For example, use the left stick horizontal axis (axis 0)
                axis_value = joystick.get_axis(0)
                target_angle = scale_axis_to_angle(axis_value)
                print(f"Joystick axis: {axis_value:.2f} -> Target angle: {target_angle}")

        # Incrementally update the current angle toward the target
        current_angle = update_angle(current_angle, target_angle, STEP_SIZE)

        # Send the current angle over serial to Arduino.
        # We add a newline so the Arduino can read line-by-line.
        message = f"{current_angle}\n"
        ser.write(message.encode('utf-8'))
        print(f"Sending angle: {current_angle}")

        # Wait a bit before the next update
        time.sleep(UPDATE_DELAY)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    ser.close()
    pygame.quit()
