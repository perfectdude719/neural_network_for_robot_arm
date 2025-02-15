import pygame
import serial
import time

# === Configuration ===
SERIAL_PORT = 'COM9'  # Change as needed (e.g., 'COM3' for Windows)
BAUD_RATE = 9600
UPDATE_DELAY = 0.05  # Increased delay between updates to slow the change rate (in seconds)
STEP_SIZE = 1        # Incremental step for angle update (in degrees). Lower this value for slower change.
ALPHA = 0.2          # Low pass filter constant (0 < ALPHA <= 1)

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

# === Low Pass Filter Setup ===
# Initialize the filtered value with the current reading.
filtered_axis = joystick.get_axis(0)

try:
    while True:
        # Process events to update internal pygame state.
        for event in pygame.event.get():
            pass  # Keeping the event loop alive

        # Read raw axis value (e.g., left stick horizontal axis at index 0)
        raw_axis_value = joystick.get_axis(0)

        # Apply low pass filtering to smooth out noise:
        filtered_axis = ALPHA * raw_axis_value + (1 - ALPHA) * filtered_axis

        # Scale the filtered value to a target angle between 0 and 270.
        target_angle = scale_axis_to_angle(filtered_axis)
        print(f"Raw axis: {raw_axis_value:.2f}, Filtered: {filtered_axis:.2f}, Target angle: {target_angle}")

        # Incrementally update the current angle toward the target angle.
        current_angle = update_angle(current_angle, target_angle, STEP_SIZE)

        # Send the current angle over serial to the Arduino.
        message = f"{current_angle}\n"
        ser.write(message.encode('utf-8'))
        print(f"Sending angle: {current_angle}")

        # Wait before the next update to slow down the change rate.
        time.sleep(UPDATE_DELAY)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    ser.close()
    pygame.quit()
