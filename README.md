# DM Motor Control

This project provides a Python library for controlling DM series motors (DM-J4310 etc). It supports multiple control modes, including MIT mode, position-velocity mode, velocity mode, and torque-position mode.

## Features

* Support for multiple DM motor models

* Various control modes: MIT, position-velocity, velocity, torque-position

* Parameter reading and setting

* Motor status monitoring

* Compatible with both new and old firmware versions

* Support for motor zero point setting

* Support for modifying and saving internal motor parameters

### Installation

```bash
pip install dmcan
```

### Usage

Import the necessary modules:

```python
from dmcan import Motor, MotorControl, DM_Motor_Type, Control_Type
```

Create a motor object:

```python
motor = Motor(DM_Motor_Type.DM4310, SlaveID=0x01, MasterID=0x11)
```

Create a motor control object:

```python
import serial
ser = serial.Serial('COM8', 921600, timeout=0.5)  # Modify the port according to your setup
motor_control = MotorControl(ser)
```

Add the motor to the control object:

```python
motor_control.addMotor(motor)
```

Enable the motor:

```python
motor_control.enable(motor)
```

Control the motor:

```python
# MIT mode control
motor_control.controlMIT(motor, kp=50, kd=0.3, q=0, dq=0, tau=0)

# Position-velocity mode control
motor_control.control_Pos_Vel(motor, P_desired=10, V_desired=2)

# Velocity mode control
motor_control.control_Vel(motor, Vel_desired=5)

# Force-position hybrid mode control
motor_control.control_pos_force(motor, pos=10, vel=1000, torque=100)
```

Read the motor status:

```python
motor_control.refresh_motor_status(motor)
print("Position:", motor.getPosition(), "Velocity:", motor.getVelocity(), "Torque:", motor.getTorque())
```

Modify internal motor parameters:

```python
motor_control.switchControlMode(motor, Control_Type.POS_VEL)
motor_control.change_motor_param(motor, DM_variable.KP_APR, 54)
motor_control.save_motor_param(motor)
```

### Examples

Please refer to the `DM_Motor_Test.py` file for more usage examples.

### Notes

* Do not set MasterID to 0x00

* It is recommended to delay 1-2ms after each control frame

* Parameter modifications should be performed when the motor is disabled

* Motor status is updated only after sending a control frame or refreshing the status

### Contributing

Issues and pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

### License

This project is licensed under the MIT License - see the `LICENSE` file for details.

### Technical Support

For technical discussions, please contact the author via email: <towardsrwby@gmail.com>
