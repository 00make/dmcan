import time
from dmcan import Motor, MotorControl, DM_Motor_Type, Control_Type
import serial
import wave
import numpy as np
import pydub
from pydub.playback import play

"""
This script controls a motor to play the melody of "Twinkle Twinkle Little Star" using a serial communication interface.

Modules:
    time: Provides various time-related functions.
    dmcan: A custom module for motor control.
    serial: A module for serial communication.

Constants:
    note_to_frequency (dict): A dictionary mapping musical notes to their corresponding frequencies.
    notes (list): A list of musical notes representing the melody of "Twinkle Twinkle Little Star".
    durations (list): A list of durations for each note in the melody.
    x_speed (float): A multiplier for the speed of the motor.

Variables:
    serial_device (serial.Serial): The serial communication interface.
    motor (Motor): The motor object.
    motor_control (MotorControl): The motor control object.

Functions:
    hz_to_timems(hz): Converts frequency in Hz to time in ms.

Execution:
    1. Initializes the serial communication interface.
    2. Creates and configures the motor and motor control objects.
    3. Plays the melody by controlling the motor's position and speed according to the notes and durations.
    4. Disables the motor and closes the serial communication interface.
"""

# Mapping of musical notes to their corresponding frequencies
note_to_frequency = {
    'C4': 261.63,
    'D4': 293.66,
    'E4': 329.63,
    'F4': 349.23,
    'G4': 392.00,
    'A4': 440.00,
    'B4': 493.88,
    'C5': 523.25
}


def hz_to_timems(hz):
    """Converts frequency in Hz to time in ms."""
    return (float(hz) + 137.85) / 125.438-0.1


# Example usage
hz = 440.00  # A4 note
timems = hz_to_timems(hz)
print(f"timems for {hz} Hz: {timems}")

# Notes and durations for "Twinkle Twinkle Little Star"
notes = ['C4', 'C4', 'G4', 'G4', 'A4', 'A4', 'G4',
         'F4', 'F4', 'E4', 'E4', 'D4', 'D4', 'C4']
durations = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1]

# Initialize serial communication
serial_device = serial.Serial(
    '/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)

# Initialize motor and motor control
motor = Motor(DM_Motor_Type.DM4310, 0x02, 0x22)
motor_control = MotorControl(serial_device)
motor_control.addMotor(motor)

# Switch to velocity control mode
if motor_control.switchControlMode(motor, Control_Type.VEL):
    print("Switched to VEL mode successfully")

# Set zero position and enable motor
motor_control.set_zero_position(motor)
motor_control.enable(motor)

# Speed multiplier
x_speed = 0.001


# Play the melody
for note, duration in zip(notes, durations):
    speed = hz_to_timems(note_to_frequency[note])
    print(f"Note: {note}, Speed: {speed}, Duration: {duration}")
    motor_control.control_Vel(motor, speed)
    time.sleep(duration)
    motor_control.control_Vel(motor, 0)
    time.sleep(duration * 0.1)

# Disable motor and close serial communication
motor_control.disable(motor)
serial_device.close()
