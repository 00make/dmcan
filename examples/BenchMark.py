import time
import math
from dmcan import *
import serial

# 初始化电机
Motor1 = Motor(DM_Motor_Type.DM4310, 0x02, 0x22)

# 初始化串口通信
serial_device = serial.Serial(
    '/dev/tty.usbmodem00000000050C1', 921600, timeout=0.5)

# 创建电机控制对象
motor_control = MotorControl(serial_device)

# 添加电机到控制对象
motor_control.addMotor(Motor1)
max_speed = 0
max_torque = 0

num_iterations = 30000  # 迭代次数


def max_speed_test():
    # 切换到速度控制模式

    if motor_control.switchControlMode(Motor1, Control_Type.VEL):
        print("切换到速度控制模式成功")
    else:
        print("切换到速度控制模式失败")
    # 最大速度测试模式
    print("开始最大速度测试模式")
    motor_control.enable(Motor1)
    max_speed = 0

    for i in range(num_iterations):
        speed = (i / num_iterations) * 30  # 速度缓慢抬升到100
        motor_control.control_Vel(Motor1, speed)
        motor_control.refresh_motor_status(Motor1)
        current_speed = Motor1.getVelocity()
        if current_speed > max_speed:
            max_speed = current_speed

    time.sleep(0.01)
    motor_control.disable(Motor1)
    print(f"最大速度: {max_speed} rad/s")
    return max_speed


def max_torque_test():

    # 切换到速度控制模式
    if motor_control.switchControlMode(Motor1, Control_Type.VEL):
        print("切换到速度控制模式成功")
    else:
        print("切换到速度控制模式失败")

    # 最大力矩测试模式
    print("开始最大力矩测试模式")
    motor_control.enable(Motor1)
    max_torque = 0
    # 缓慢爬升到最大速度
    for i in range(num_iterations):
        speed = (i / num_iterations) * max_speed  # 速度缓慢抬升到最大
        motor_control.control_Vel(Motor1, speed)
        motor_control.refresh_motor_status(Motor1)
    
    # 测量最大力矩
    time.sleep(0.001)
    current_torque = Motor1.getTorque()
    if current_torque > max_torque:
        max_torque = current_torque
    # 测量最大力矩
    time.sleep(0.001)
    current_torque = Motor1.getTorque()
    if current_torque > max_torque:
        max_torque = current_torque
    # 突然反向刹车
    motor_control.control_Vel(Motor1, -max_speed)
    # 测量最大力矩
    time.sleep(0.001)
    current_torque = Motor1.getTorque()
    if current_torque > max_torque:
        max_torque = current_torque
    # 测量最大力矩
    time.sleep(0.001)
    current_torque = Motor1.getTorque()
    if current_torque > max_torque:
        max_torque = current_torque

    print(f"最大力矩: {max_torque} Nm")
    time.sleep(0.01)

    motor_control.disable(Motor1)
    return max_torque


def feedback_frequency_test():

    # 存储反馈频率
    feedback_frequencies = []

    # 基准测试开始
    start_time = time.time()
    for i in range(num_iterations):
        q = math.sin(time.time())
        control_start_time = time.time()

        # 控制电机位置和速度
        motor_control.control_Pos_Vel(Motor1, q, 30)
        motor_control.refresh_motor_status(Motor1)
        # 记录反馈频率
        response_time = time.time() - control_start_time
        feedback_frequency = 1 / response_time
        feedback_frequencies.append(feedback_frequency)

    # 基准测试结束
    end_time = time.time()
    total_duration = end_time - start_time

    # 计算平均反馈频率
    average_feedback_frequency = len(feedback_frequencies)/total_duration

    # 输出测试结果
    print(f"反馈频率基准测试持续时间: {total_duration:.2f} 秒")
    print(f"平均反馈频率: {average_feedback_frequency:.2f} Hz")
    motor_control.disable(Motor1)


while True:
    print("\n请选择电机测试模式（部分功能需要更新固件到 5015 以上）:")
    print("为了测试安全请确保电机固定在一个位置，不要有任何负载。")
    print("1. 反馈频率")
    print("2. 最大速度")
    # print("3. 最大力矩")
    print("4. 退出")

    choice = input("输入选项 (1/2/3/4): ")

    if choice == '1':
        feedback_frequency_test()
    elif choice == '2':
        max_speed = max_speed_test()
    elif choice == '3':
        max_torque = max_torque_test()
    elif choice == '4':
        break
    else:
        print("无效选项，请重新选择。")


# 关闭串口
serial_device.close()
