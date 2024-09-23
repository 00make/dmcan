import time
"""
This script controls a motor to play the melody of "Twinkle Twinkle Little Star" using a serial communication interface.

Modules:
    time: Provides various time-related functions.
    dmcan: A custom module for motor control.
    serial: A module for serial communication.

Constants:
    note_to_speed (dict): A dictionary mapping musical notes to their corresponding speeds.
    notes (list): A list of musical notes representing the melody of "Twinkle Twinkle Little Star".
    durations (list): A list of durations for each note in the melody.
    x_speed (float): A multiplier for the speed of the motor.
    x_duration (float): A multiplier for the duration of each note.

Variables:
    serial_device (serial.Serial): The serial communication interface.
    motor (Motor): The motor object.
    motor_control (MotorControl): The motor control object.
    current_position (float): The current position of the motor.

Functions:
    None

Execution:
    1. Initializes the serial communication interface.
    2. Creates and configures the motor and motor control objects.
    3. Plays the melody by controlling the motor's position and speed according to the notes and durations.
    4. Disables the motor and closes the serial communication interface.
"""
from dmcan import Motor, MotorControl, DM_Motor_Type, Control_Type
import serial

# 音符与速度映射
note_to_speed = {
    'C': 100,
    'D': 200,
    'E': 300,
    'F': 400,
    'G': 500,
    'A': 600,
    'B': 700,
    'C5': 800
}

# 小星星的音符和时长
notes = ['C', 'C', 'G', 'G', 'A', 'A', 'G', 'F', 'F', 'E', 'E', 'D', 'D', 'C']
durations = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1]

# 速度和间隔倍数
x_speed = 1.0
x_duration = 1.0

# 初始化串口通信
serial_device = serial.Serial(
    '/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)

# 创建电机对象
motor = Motor(DM_Motor_Type.DM4310, 0x09, 0x99)

# 创建电机控制对象
motor_control = MotorControl(serial_device)

# 添加电机到控制对象
motor_control.addMotor(motor)

# 电机初始化

if motor_control.switchControlMode(motor, Control_Type.Torque_Pos):
    print("切换到POS_VEL模式成功")


motor_control.set_zero_position(motor)
motor_control.enable(motor)

# 播放音符
current_position = 0.0
for note, duration in zip(notes, durations):
    speed = note_to_speed[note] * x_speed
    target_position = current_position + speed * duration * x_duration
    motor_control.control_Pos_Vel(motor, target_position, speed)
    time.sleep(duration * x_duration)
    current_position = target_position

# 结束控制
motor_control.disable(motor)

# 关闭串口
serial_device.close()
