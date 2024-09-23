import math
from dmcan import *
import serial
import time

# 创建两个电机对象
Motor1 = Motor(DM_Motor_Type.DM4310, 0x09, 0x99)
Motor2 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)

# 初始化串口通信
serial_device = serial.Serial('/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)

# 创建电机控制对象
MotorControl1 = MotorControl(serial_device)

# 添加电机到控制对象
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)

# 切换电机控制模式
if MotorControl1.switchControlMode(Motor1, Control_Type.POS_VEL):
    print("切换到位置速度控制模式成功")
if MotorControl1.switchControlMode(Motor2, Control_Type.VEL):
    print("切换到速度控制模式成功")

# 读取电机参数
print("子版本号:", MotorControl1.read_motor_param(Motor1, DM_variable.sub_ver))
print("减速比:", MotorControl1.read_motor_param(Motor1, DM_variable.Gr))

# 读取Motor1参数
print("最大位置:", MotorControl1.read_motor_param(Motor1, DM_variable.PMAX))
print("主机ID:", MotorControl1.read_motor_param(Motor1, DM_variable.MST_ID))
print("最大速度:", MotorControl1.read_motor_param(Motor1, DM_variable.VMAX))
print("最大扭矩:", MotorControl1.read_motor_param(Motor1, DM_variable.TMAX))

# 读取Motor2参数
print("Motor2:")
print("最大位置:", MotorControl1.read_motor_param(Motor2, DM_variable.PMAX))
print("主机ID:", MotorControl1.read_motor_param(Motor2, DM_variable.MST_ID))
print("最大速度:", MotorControl1.read_motor_param(Motor2, DM_variable.VMAX))
print("最大扭矩:", MotorControl1.read_motor_param(Motor2, DM_variable.TMAX))

# 保存电机参数
MotorControl1.save_motor_param(Motor1)
MotorControl1.save_motor_param(Motor2)

# 使能电机
MotorControl1.enable(Motor1)
MotorControl1.enable(Motor2)

# 控制循环
i = 0
while i < 10000:
    q = math.sin(time.time())
    i = i + 1

    # 控制Motor1的位置和速度
    MotorControl1.control_Pos_Vel(Motor1, q*8, 30)

    # 控制Motor2的速度
    MotorControl1.control_Vel(Motor2, 8*q)

    time.sleep(0.001)

# 关闭串口
serial_device.close()
