import scipy.signal
import time
from dmcan import Motor, MotorControl, DM_Motor_Type, Control_Type
import serial
import numpy as np
import pydub
from pydub.playback import play
from threading import Thread


def hz_to_timems(hz):
    """Converts frequency in Hz to time in ms."""
    return (float(hz) + 137.85) / 125.438


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

# Load the wave file using pydub
audio = pydub.AudioSegment.from_wav("examples/sample-6s.wav")

# Convert to mono if the audio is multi-channel
if audio.channels > 1:
    audio = audio.set_channels(1)

# Extract raw audio data
samples = np.array(audio.get_array_of_samples())

# Frame rate and frame duration
frame_rate = audio.frame_rate
frame_duration = 1.0 / frame_rate

# Window size for FFT (e.g., 1024 samples)
window_size = 1024

# Function to control motor speed in real-time


def control_motor():
    for i in range(0, len(samples), window_size):
        window_samples = samples[i:i + window_size]
        if len(window_samples) < window_size:
            break

        # Compute the FFT of the windowed samples
        fft_result = np.fft.fft(window_samples)
        frequencies = np.fft.fftfreq(len(window_samples), d=1/frame_rate)

        # Find the peak frequency in the window
        peak_frequency = frequencies[np.argmax(np.abs(fft_result))]

        # Convert peak frequency to motor speed
        speed = hz_to_timems(peak_frequency)*6

        # Control the motor speed
        motor_control.control_Vel(motor, speed)
        time.sleep(frame_duration * window_size)

# Play audio and control motor simultaneously


motor_thread = Thread(target=control_motor)
motor_thread.start()

play(audio)

motor_thread.join()

# Stop the motor
motor_control.control_Vel(motor, 0)

# Disable motor and close serial communication
motor_control.disable(motor)
serial_device.close()
