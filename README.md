# DM电机控制

这个项目提供了一个用于控制DM系列电机的Python库。它支持多种控制模式,包括MIT模式、位置速度模式、速度模式和转矩位置模式。

### 特性

* 支持多种DM电机型号

* 多种控制模式:MIT、位置速度、速度、转矩位置

* 参数读取和设置

* 电机状态监控

* 兼容新旧固件版本

* 支持电机零点设置

* 支持电机内部参数修改和保存

### 安装

```bash
pip install dmcan
```

### 使用方法

1. 导入必要的模块:

```python
from dmcan import Motor, MotorControl, DM_Motor_Type, Control_Type
```

1. 创建电机对象:

```python
motor = Motor(DM_Motor_Type.DM4310, SlaveID=0x01, MasterID=0x11)
```

1. 创建电机控制对象:

```python
import serial
ser = serial.Serial('COM8', 921600, timeout=0.5)  # 根据实际情况修改串口
motor_control = MotorControl(ser)
```

1. 添加电机到控制对象:

```python
motor_control.addMotor(motor)
```

1. 使能电机:

```python
motor_control.enable(motor)
```

1. 控制电机:

```python
# MIT模式控制
motor_control.controlMIT(motor, kp=50, kd=0.3, q=0, dq=0, tau=0)

# 位置速度模式控制
motor_control.control_Pos_Vel(motor, P_desired=10, V_desired=2)

# 速度模式控制
motor_control.control_Vel(motor, Vel_desired=5)

# 力位混合模式控制
motor_control.control_pos_force(motor, pos=10, vel=1000, torque=100)
```

1. 读取电机状态:

```python
motor_control.refresh_motor_status(motor)
print("位置:", motor.getPosition(), "速度:", motor.getVelocity(), "力矩:", motor.getTorque())
```

1. 修改电机内部参数:

```python
motor_control.switchControlMode(motor, Control_Type.POS_VEL)
motor_control.change_motor_param(motor, DM_variable.KP_APR, 54)
motor_control.save_motor_param(motor)
```

### 示例

请参考 `DM_Motor_Test.py` 文件以获取更多使用示例。

### 注意事项

* MasterID不要设置为0x00

* 建议在每帧控制后延迟1-2ms

* 参数修改请在失能状态下进行

* 电机状态需要发送控制帧或刷新状态后才会更新

### 贡献

欢迎提交问题报告和拉取请求。对于重大更改,请先开issue讨论您想要更改的内容。

### 许可证

本项目采用 MIT 许可证 - 有关详细信息,请查看 `LICENSE` 文件。

### 技术支持

如需技术交流，欢迎加入QQ群：677900232
