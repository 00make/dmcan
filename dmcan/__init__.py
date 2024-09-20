"""
达妙电机控制包

这个包提供了用于控制DM系列电机的Python接口。
它支持多种控制模式,包括MIT模式、位置速度模式、速度模式和转矩位置模式。

主要类:
- Motor: 电机对象
- MotorControl: 电机控制对象

主要枚举:
- DM_Motor_Type: 电机类型
- Control_Type: 控制类型
- DM_variable: 电机变量

使用示例:
from dm_motor_control import Motor, MotorControl, DM_Motor_Type, Control_Type

# 创建电机对象
motor = Motor(DM_Motor_Type.DM4310, SlaveID=1, MasterID=0x11)

# 创建电机控制对象
import serial
ser = serial.Serial('COM1', 921600, timeout=0.5)
motor_control = MotorControl(ser)

# 添加电机到控制对象
motor_control.addMotor(motor)

# 使能电机
motor_control.enable(motor)

# 控制电机
motor_control.controlMIT(motor, kp=0.5, kd=0.1, q=1.0, dq=0.0, tau=0.0)

更多详细信息,请参阅文档。
"""

# 导入主要的类和枚举
from .dmcan import Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable

# 定义版本号
__version__ = "0.1.0"

# 定义所有可以直接从包中导入的对象
__all__ = ['Motor', 'MotorControl',
           'DM_Motor_Type', 'Control_Type', 'DM_variable']
