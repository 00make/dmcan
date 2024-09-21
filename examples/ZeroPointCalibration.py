import math
"""
零点校准.py

This script initializes and calibrates three DM4310 motors using the dmcan library. 
It sets the zero position, enables the motors, performs a calibration process, 
and then disables the motors and closes the serial device.

Modules:
    math
    dmcan
    serial
    time

Classes:
    None

Functions:
    None

Variables:
    Motor1 (Motor): The first motor initialized with DM4310 type and specific IDs.
    Motor2 (Motor): The second motor initialized with DM4310 type and specific IDs.
    Motor3 (Motor): The third motor initialized with DM4310 type and specific IDs.
    serial_device (serial.Serial): The serial device for motor control communication.
    MControl (MotorControl): The motor control object managing the motors.

Usage:
    Run the script to initialize the motors, set their zero positions, enable them, 
    perform a calibration process, and then disable the motors and close the serial device.
"""
from dmcan import *
import serial
import time

# 初始化电机
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)
Motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x13)

# 初始化串口设备
serial_device = serial.Serial(
    '/dev/tty.Bluetooth-Incoming-Port', 921600, timeout=0.5)
MControl = MotorControl(serial_device)
MControl.addMotor(Motor1)
MControl.addMotor(Motor2)
MControl.addMotor(Motor3)

# 切换控制模式
if MControl.switchControlMode(Motor1, Control_Type.Torque_Pos):
    print("Motor1: switch POS_VEL success")
if MControl.switchControlMode(Motor2, Control_Type.Torque_Pos):
    print("Motor2: switch POS_VEL success")
if MControl.switchControlMode(Motor3, Control_Type.Torque_Pos):
    print("Motor3: switch POS_VEL success")

# 设置零点位置
print("Setting zero position...")
MControl.set_zero_position(Motor1)
MControl.set_zero_position(Motor2)
MControl.set_zero_position(Motor3)
print("Zero position set.")

# 启用电机
MControl.enable(Motor1)
MControl.enable(Motor2)
MControl.enable(Motor3)
print("Motors enabled.")

# 校准过程
i = 0
while i < 100:
    i += 1
    MControl.control_pos_force(Motor1, 0, 1000, 400)
    MControl.control_pos_force(Motor2, 0, 1000, 400)
    MControl.control_pos_force(Motor3, 0, 1000, 400)

    if Motor1.getTorque() > 1:
        MControl.disable(Motor1)
        MControl.disable(Motor2)
        MControl.disable(Motor3)
        print("Torque limit exceeded, motors disabled.")
        break

# 校准完成提示
print("Calibration complete.")

# 禁用电机
MControl.disable(Motor1)
MControl.disable(Motor2)
MControl.disable(Motor3)
print("Motors disabled.")

# 关闭串口设备
serial_device.close()
print("Serial device closed.")
