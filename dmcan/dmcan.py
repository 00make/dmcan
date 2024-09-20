from time import sleep
import numpy as np
from enum import IntEnum
from struct import unpack
from struct import pack


class Motor:
    def __init__(self, MotorType, SlaveID, MasterID):
        """
        定义电机对象
        :param MotorType: 电机类型
        :param SlaveID: 电机CAN ID
        :param MasterID: 主机ID (建议不要设为0)
        """
        self.Pd = float(0)  # 期望位置
        self.Vd = float(0)  # 期望速度
        self.state_q = float(0)  # 当前位置
        self.state_dq = float(0)  # 当前速度
        self.state_tau = float(0)  # 当前力矩
        self.SlaveID = SlaveID  # 电机CAN ID
        self.MasterID = MasterID  # 主机ID
        self.MotorType = MotorType  # 电机类型
        self.isEnable = False  # 是否使能
        self.NowControlMode = Control_Type.MIT  # 当前控制模式
        self.temp_param_dict = {}  # 临时参数字典

    def recv_data(self, q: float, dq: float, tau: float):
        """
        接收电机反馈数据
        :param q: 位置
        :param dq: 速度
        :param tau: 力矩
        """
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau

    def getPosition(self):
        """
        获取电机位置
        :return: 电机位置
        """
        return self.state_q

    def getVelocity(self):
        """
        获取电机速度
        :return: 电机速度
        """
        return self.state_dq

    def getTorque(self):
        """
        获取电机力矩
        :return: 电机力矩
        """
        return self.state_tau

    def getParam(self, RID):
        """
        获取电机内部参数,需要提前读取
        :param RID: 电机参数ID
        :return: 电机参数值
        """
        if RID in self.temp_param_dict:
            return self.temp_param_dict[RID]
        else:
            return None


class MotorControl:
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)
    #                4310           4310_48        4340           4340_48
    Limit_Param = [[12.5, 30, 10], [12.5, 50, 10], [12.5, 8, 28], [12.5, 10, 28],
                   # 6006           8006           8009            10010L         10010
                   [12.5, 45, 20], [12.5, 45, 40], [12.5, 45, 54], [12.5, 25, 200], [12.5, 20, 200]]

    def __init__(self, serial_device):
        """
        定义电机控制对象
        :param serial_device: 串口对象
        """
        self.serial_ = serial_device
        self.motors_map = dict()  # 电机映射字典
        self.data_save = bytes()  # 保存未处理的数据
        if self.serial_.is_open:  # 打开串口
            print("串口已打开")
            serial_device.close()
        self.serial_.open()

    def controlMIT(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        """
        达妙电机MIT控制模式函数
        :param DM_Motor: 电机对象
        :param kp: 位置增益
        :param kd: 速度增益
        :param q: 期望位置
        :param dq: 期望速度
        :param tau: 期望力矩
        """
        if DM_Motor.SlaveID not in self.motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX = self.Limit_Param[MotorType][0]
        DQ_MAX = self.Limit_Param[MotorType][1]
        TAU_MAX = self.Limit_Param[MotorType][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00], np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        self.__send_data(DM_Motor.SlaveID, data_buf)
        self.recv()  # receive the data from serial port

    def control_delay(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float, delay: float):
        """
        达妙电机MIT控制模式函数带延迟
        :param DM_Motor: 电机对象
        :param kp: 位置增益
        :param kd: 速度增益
        :param q: 期望位置
        :param dq: 期望速度
        :param tau: 期望力矩
        :param delay: 延迟时间 (单位: 秒)
        """
        self.controlMIT(DM_Motor, kp, kd, q, dq, tau)
        sleep(delay)

    def control_Pos_Vel(self, Motor, P_desired: float, V_desired: float):
        """
        电机位置速度控制模式
        :param Motor: 电机对象
        :param P_desired: 期望位置
        :param V_desired: 期望速度
        """
        if Motor.SlaveID not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        # time.sleep(0.001)
        self.recv()  # receive the data from serial port

    def control_Vel(self, Motor, Vel_desired):
        """
        电机速度控制模式
        :param Motor: 电机对象
        :param Vel_desired: 期望速度
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def control_pos_force(self, Motor, Pos_des: float, Vel_des, i_des):
        """
        电机力位混合模式
        :param Pos_des: 期望位置 (单位: rad)
        :param Vel_des: 期望速度 (单位: rad/s, 放大100倍)
        :param i_des: 期望电流标幺值 (放大10000倍)
        电流标幺值: 实际电流值除以最大电流值, 最大电流见上电打印
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(Pos_des)
        data_buf[0:4] = Pos_desired_uint8s
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xff
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xff
        data_buf[7] = ides_uint >> 8
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def enable(self, Motor):
        """
        使能电机
        最好在上电后几秒后再使能电机
        :param Motor: 电机对象
        """
        self.__control_cmd(Motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def enable_old(self, Motor, ControlMode):
        """
        使能电机旧版本固件, 这个是为了旧版本电机固件的兼容性
        可恶的旧版本固件使能需要加上偏移量
        最好在上电后几秒后再使能电机
        :param Motor: 电机对象
        """
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff,
                            0xff, 0xff, 0xfc], np.uint8)
        enable_id = ((int(ControlMode)-1) << 2) + Motor.SlaveID
        self.__send_data(enable_id, data_buf)
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def disable(self, Motor):
        """
        失能电机
        :param Motor: 电机对象
        """
        self.__control_cmd(Motor, np.uint8(0xFD))
        sleep(0.01)

    def set_zero_position(self, Motor):
        """
        设置电机0位
        :param Motor: 电机对象
        """
        self.__control_cmd(Motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def recv(self):
        # 把上次没有解析完的剩下的也放进来

        data_recv = b''.join([self.data_save, self.serial_.read_all()])
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (
                packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_packet(data, CANID, CMD)

    def recv_set_param_data(self):
        data_recv = self.serial_.read_all()
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (
                packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_set_param_packet(data, CANID, CMD)

    def __process_packet(self, data, CANID, CMD):
        if CMD == 0x11:
            if CANID in self.motors_map:
                q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                MotorType_recv = self.motors_map[CANID].MotorType
                Q_MAX = self.Limit_Param[MotorType_recv][0]
                DQ_MAX = self.Limit_Param[MotorType_recv][1]
                TAU_MAX = self.Limit_Param[MotorType_recv][2]
                recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau)

    def __process_set_param_packet(self, data, CANID, CMD):
        if CMD == 0x11 and (data[2] == 0x33 or data[2] == 0x55):
            masterid = CANID
            slaveId = ((data[1] << 8) | data[0])
            if CANID == 0x00:  # 防止有人把MasterID设为0稳一手
                masterid = slaveId

            if masterid not in self.motors_map:
                if slaveId not in self.motors_map:
                    return
                else:
                    masterid = slaveId

            RID = data[3]
            # 读取参数得到的数据
            if is_in_ranges(RID):
                # uint32类型
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num

            else:
                # float类型
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num

    def addMotor(self, Motor):
        """
        添加电机到电机控制对象
        :param Motor: 电机对象
        """
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor
        return True

    def __control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array(
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
        self.__send_data(Motor.SlaveID, data_buf)

    def __send_data(self, motor_id, data):
        """
        发送数据到电机
        :param motor_id:
        :param data:
        """
        self.send_data_frame[13] = motor_id & 0xff
        self.send_data_frame[14] = (motor_id >> 8) & 0xff  # id high 8 bits
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame.T))

    def __read_RID_param(self, Motor, RID):
        can_id_l = Motor.SlaveID & 0xff  # id low 8 bits
        can_id_h = (Motor.SlaveID >> 8) & 0xff  # id high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(
            can_id_h), 0x33, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)

    def __write_motor_param(self, Motor, RID, data):
        can_id_l = Motor.SlaveID & 0xff  # id low 8 bits
        can_id_h = (Motor.SlaveID >> 8) & 0xff  # id high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(
            can_id_h), 0x55, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        if not is_in_ranges(RID):
            # data is float
            data_buf[4:8] = float_to_uint8s(data)
        else:
            # data is int
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switchControlMode(self, Motor, ControlMode):
        """
        切换电机控制模式
        :param Motor: 电机对象
        :param ControlMode: 电机控制模式 (例如: MIT:Control_Type.MIT)
        """
        RID = 10
        self.__write_motor_param(Motor, RID, np.uint8(ControlMode))
        sleep(0.1)
        self.recv_set_param_data()
        if Motor.SlaveID in self.motors_map:
            if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                if self.motors_map[Motor.SlaveID].temp_param_dict[RID] == ControlMode:
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False

    def save_motor_param(self, Motor):
        """
        保存所有电机参数
        :param Motor: 电机对象
        """
        can_id_l = Motor.SlaveID & 0xff  # id low 8 bits
        can_id_h = (Motor.SlaveID >> 8) & 0xff  # id high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(
            can_id_h), 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.disable(Motor)  # before save disable the motor
        self.__send_data(0x7FF, data_buf)
        sleep(0.01)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        """
        改变电机的PMAX VMAX TMAX
        :param Motor_Type:
        :param PMAX: 电机的PMAX
        :param VMAX: 电机的VMAX
        :param TMAX: 电机的TMAX
        """
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def refresh_motor_status(self, Motor):
        """
        获得电机状态
        """
        can_id_l = Motor.SlaveID & 0xff  # id low 8 bits
        can_id_h = (Motor.SlaveID >> 8) & 0xff  # id high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(
            can_id_h), 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)
        self.recv()  # receive the data from serial port

    def change_motor_param(self, Motor, RID, data):
        """
        改变电机的参数
        :param Motor: 电机对象
        :param RID: 电机参数
        :param data: 电机参数的值
        :return: True or False ,True means success, False means fail
        """
        self.__write_motor_param(Motor, RID, data)
        sleep(0.1)
        self.recv_set_param_data()
        if Motor.SlaveID in self.motors_map and RID in self.motors_map[Motor.SlaveID].temp_param_dict:
            if abs(self.motors_map[Motor.SlaveID].temp_param_dict[RID] - data) < 0.1:
                return True
            else:
                return False
        else:
            return False

    def read_motor_param(self, Motor, RID):
        """
        读取电机的内部信息例如 版本号等
        :param Motor: 电机对象
        :param RID: 电机参数
        :return: 电机参数的值
        """
        self.__read_RID_param(Motor, RID)
        sleep(0.08)
        self.recv_set_param_data()
        if Motor.SlaveID in self.motors_map:
            if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                return self.motors_map[Motor.SlaveID].temp_param_dict[RID]
            else:
                return None
        else:
            return None

    # -------------------------------------------------
    # 从串行数据中提取数据包
    def __extract_packets(self, data):
        frames = []
        header = 0xAA  # 帧头
        tail = 0x55    # 帧尾
        frame_length = 16  # 帧长度
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        return frames


# 限制值在最小值和最大值之间
def LIMIT_MIN_MAX(x, min, max):
    if x <= min:
        x = min
    elif x > max:
        x = max


# 将浮点数转换为无符号整数
def float_to_uint(x: float, x_min: float, x_max: float, bits):
    LIMIT_MIN_MAX(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


# 将无符号整数转换为浮点数
def uint_to_float(x: np.uint16, min: float, max: float, bits):
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)


# 将浮点数转换为4个uint8
def float_to_uint8s(value):
    # 将浮点数打包为4字节
    packed = pack('f', value)
    # 将字节解包为4个uint8值
    return unpack('4B', packed)


# 将数据转换为4个uint8
def data_to_uint8s(value):
    # 检查值是否在uint32范围内
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # 将uint32打包为4字节
        packed = pack('I', value)
    else:
        raise ValueError("值必须是uint32范围内的整数")

    # 将字节解包为4个uint8值
    return unpack('4B', packed)


# 检查数字是否在指定范围内
def is_in_ranges(number):
    """
    检查数字是否在uint32的范围内
    :param number:
    :return:
    """
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False


# 将4个uint8转换为uint32
def uint8s_to_uint32(byte1, byte2, byte3, byte4):
    # 以小端序将4个uint8值打包为单个uint32值
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # 将打包的字节解包为uint32值
    return unpack('<I', packed)[0]


# 将4个uint8转换为浮点数
def uint8s_to_float(byte1, byte2, byte3, byte4):
    # 以小端序将4个uint8值打包为单个浮点值
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # 将打包的字节解包为浮点值
    return unpack('<f', packed)[0]


# 以十六进制格式打印数据
def print_hex(data):
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))


# 通过索引获取枚举值
def get_enum_by_index(index, enum_class):
    try:
        return enum_class(index)
    except ValueError:
        return None


# 电机类型枚举
class DM_Motor_Type(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM4340_48V = 3
    DM6006 = 4
    DM8006 = 5
    DM8009 = 6
    DM10010L = 7
    DM10010 = 8


# 电机变量枚举
class DM_variable(IntEnum):
    UV_Value = 0    # 欠压值
    KT_Value = 1    # 转矩常数
    OT_Value = 2    # 过温值
    OC_Value = 3    # 过流值
    ACC = 4         # 加速度
    DEC = 5         # 减速度
    MAX_SPD = 6     # 最大速度
    MST_ID = 7      # 主机ID
    ESC_ID = 8      # 电调ID
    TIMEOUT = 9     # 超时时间
    CTRL_MODE = 10  # 控制模式
    Damp = 11       # 阻尼
    Inertia = 12    # 惯量
    hw_ver = 13     # 硬件版本
    sw_ver = 14     # 软件版本
    SN = 15         # 序列号
    NPP = 16        # 极对数
    Rs = 17         # 定子电阻
    LS = 18         # 定子电感
    Flux = 19       # 磁通量
    Gr = 20         # 减速比
    PMAX = 21       # 最大功率
    VMAX = 22       # 最大电压
    TMAX = 23       # 最大转矩
    I_BW = 24       # 电流带宽
    KP_ASR = 25     # 速度环比例增益
    KI_ASR = 26     # 速度环积分增益
    KP_APR = 27     # 位置环比例增益
    KI_APR = 28     # 位置环积分增益
    OV_Value = 29   # 过压值
    GREF = 30       # 参考增益
    Deta = 31       # 增量
    V_BW = 32       # 电压带宽
    IQ_c1 = 33      # Q轴电流补偿1
    VL_c1 = 34      # 电压补偿1
    can_br = 35     # CAN波特率
    sub_ver = 36    # 子版本
    u_off = 50      # U相偏移
    v_off = 51      # V相偏移
    k1 = 52         # 系数1
    k2 = 53         # 系数2
    m_off = 54      # 磁偏移
    dir = 55        # 方向
    p_m = 80        # 位置模式
    xout = 81       # 输出


# 控制类型枚举
class Control_Type(IntEnum):
    MIT = 1         # MIT模式
    POS_VEL = 2     # 位置速度模式
    VEL = 3         # 速度模式
    Torque_Pos = 4  # 转矩位置模式
