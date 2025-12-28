import serial
import time
import sys
sys.path.append('/home/sunrise/YDLidar-SDK/build/python')
import ydlidar
from typing import *
from params import *

# ============================================================
# 数据帧类（负责单帧数据的封装、解析、校验）
# ============================================================

class RobotFrame:
    """OriginBot 上下位机通讯协议的数据帧类"""

    def __init__(self):
        # 初始化数据帧的各个字段
        self._header = FrameParams.FRAME_HEADER  # 帧头(0x55)
        self._tail = FrameParams.FRAME_TAIL      # 帧尾(0xBB)
        self._cmd_id = 0x00                      # 指令ID
        self._length_data = FrameParams.FRAME_LENGTH_Data  # 数据段长度(固定6字节)
        self._data = [0x00] * FrameParams.FRAME_LENGTH_Data  # 数据内容
        self._checksum = 0x00                    # 计算得到的校验值
        self._original_checksum = 0x00           # 接收帧中的原始校验值
        self._frame = [0x00] * FrameParams.FRAME_LENGTH_TOTAL  # 整个帧数据
        self._data_integrity = False             # 数据是否有效标志
        self._update_frame()                     # 初始化构造帧

    # --------------------- 属性定义 ------------------------

    @property
    def cmd_id(self):
        """获取指令ID"""
        return self._cmd_id

    @cmd_id.setter
    def cmd_id(self, v):
        """设置指令ID"""
        self._cmd_id = v
        self._update_frame()

    @property
    def data(self):
        """获取数据段"""
        return self._data

    @data.setter
    def data(self, d: List[int]):
        """设置数据段（自动重新计算校验和）"""
        if len(d) != FrameParams.FRAME_LENGTH_Data:
            raise ValueError(f"数据长度错误，应为 {FrameParams.FRAME_LENGTH_Data} 字节")
        self._data = [int(x) & 0xFF for x in d]
        self._calculate_checksum()
        self._update_frame()
        self._data_integrity = True

    @property
    def frame(self):
        """获取完整帧"""
        return self._frame

    @frame.setter
    def frame(self, frame_list: List[int]):
        """设置完整帧并自动解析其中数据"""
        if len(frame_list) != FrameParams.FRAME_LENGTH_TOTAL:
            raise ValueError(f"帧长度错误，应为 {FrameParams.FRAME_LENGTH_TOTAL} 字节")
        self._frame = [int(x) & 0xFF for x in frame_list]
        self._update_non_frame()
        self._verify_data_integrity()

    @property
    def data_integrity(self):
        """返回该帧数据是否有效"""
        return self._data_integrity

    # --------------------- 内部方法 ------------------------

    def _calculate_checksum(self):
        """计算校验和：所有数据字节相加后取低8位"""
        self._checksum = sum(self._data) & 0xFF
        return self._checksum

    def _update_frame(self):
        """根据当前数据字段重新生成完整帧"""
        self._calculate_checksum()
        f = [0x00] * FrameParams.FRAME_LENGTH_TOTAL
        f[0] = self._header
        f[1] = self._cmd_id
        f[2] = self._length_data
        f[3:3 + self._length_data] = self._data
        f[-2] = self._checksum
        f[-1] = self._tail
        self._frame = f

    def _update_non_frame(self):
        """当frame被整体赋值时，反向解析出header、cmd_id、data等"""
        self._header = self._frame[0]
        self._cmd_id = self._frame[1]
        self._length_data = self._frame[2]
        self._data = self._frame[3:3 + self._length_data]
        self._original_checksum = self._frame[-2]
        self._tail = self._frame[-1]

    def _verify_data_integrity(self):
        """验证帧的完整性（帧头、帧尾、长度、校验值）"""
        checksum_valid = (sum(self._data) & 0xFF) == self._original_checksum
        integrity = (
            self._header == FrameParams.FRAME_HEADER and
            self._tail == FrameParams.FRAME_TAIL and
            checksum_valid and
            len(self._data) == FrameParams.FRAME_LENGTH_Data and
            len(self._frame) == FrameParams.FRAME_LENGTH_TOTAL
        )
        self._data_integrity = integrity
        return integrity


# ============================================================
# 控制器类（负责与STM32通信、发送指令、接收解析）
# ============================================================

class RobotController:
    """
    OriginBot 上位机控制类
    - 负责串口连接与通信
    - 支持速度控制、电机反馈、IMU、传感器、电压等多种指令
    """

    def __init__(self, port=MotionBoardParams.PORT, baudrate=MotionBoardParams.BAUDRATE):
        """
        初始化函数
        参数：
            port: 串口端口路径
            baudrate: 串口波特率（默认115200）
        """
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._running = False

        self.init()  # 自动初始化串口

    # ============================================================
    # 串口相关函数
    # ============================================================

    def init(self) -> bool:
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[信息] 已连接到串口 {self.port} 波特率 {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"[错误] 串口连接失败: {e}")
            self.ser = None
            return False

    def close(self):
        """
        安全关闭串口连接，并在关闭前自动让小车停机、关闭外设。
        该函数应在程序结束时调用，保证所有硬件恢复静止状态，防止误动作。

        执行顺序：
        1. 停止监听线程（防止在关闭过程中继续读取）
        2. 发送“速度为 0”指令，使左右轮停止
        3. 发送“关闭外设”指令（LED 灭、蜂鸣器关、IMU 不校准）
        4. 等待少许时间，确保 STM32 执行完命令
        5. 关闭串口资源
        """

        # 标记监听线程退出
        self._running = False
        print("[系统] 正在安全关闭设备...")

        # 如果串口仍处于连接状态，则执行停机命令
        if self.is_connected():
            try:
                # 1. 停止小车运动
                print("[系统] 发送停机指令：速度=0")
                self.send_speed_control(0, 0)
                time.sleep(0.1)

                # 2. 关闭LED、蜂鸣器、IMU校准
                print("[系统] 发送外设关闭指令")
                self.send_device_control(
                    led_enable=0xFF, led_state=0x00,  # LED 关闭
                    buz_enable=0xFF, buz_state=0x00,  # 蜂鸣器关闭
                    imu_enable=0xFF, imu_state=0x00   # 停止IMU校准
                )
                time.sleep(0.1)

            except Exception as e:
                print(f"[警告] 关闭设备时发送指令失败: {e}")

        # 3. 关闭串口
        if self.ser is not None:
            try:
                self.ser.close()
                print("[系统] 串口已关闭")
            except Exception as e:
                print(f"[警告] 串口关闭失败: {e}")

        print("[系统] 已安全关闭，所有设备已停止。")

    def is_connected(self) -> bool:
        """检查串口是否处于连接状态"""
        return self.ser is not None and self.ser.is_open

    # ============================================================
    # 数据发送函数（上位机 -> STM32）
    # ============================================================

    def send_frame(self, frame: RobotFrame) -> bool:
        """发送完整数据帧"""
        if not self.is_connected():
            print("[警告] 未连接串口，无法发送数据")
            return False
        try:
            self.ser.write(bytes(frame.frame))
            return True
        except serial.SerialException as e:
            print(f"[错误] 串口写入失败: {e}")
            return False

    def send_speed_control(self, left_mm_s: int, right_mm_s: int) -> bool:
        """
        发送速度控制命令
        参数：
            left_mm_s: 左轮速度（mm/s，带符号）
            right_mm_s: 右轮速度（mm/s，带符号）
        """
        def pack_side(v):
            if v >= 0:
                dir_b = 0xFF
                mag = int(v)
            else:
                dir_b = 0x00
                mag = -int(v)
            return [dir_b, mag & 0xFF, (mag >> 8) & 0xFF]

        frame = RobotFrame()
        frame.cmd_id = FrameParams.CMD_SPEED_CONTROL
        frame.data = pack_side(left_mm_s) + pack_side(right_mm_s)

        if self.send_frame(frame):
            print(f"[发送] 速度控制 -> 左轮={left_mm_s}mm/s 右轮={right_mm_s}mm/s")
            return True
        return False

    def send_device_control(self, led_enable: int, led_state: int,
                            buz_enable: int, buz_state: int,
                            imu_enable: int, imu_state: int) -> bool:
        """
        发送LED、蜂鸣器、IMU校准控制命令
        参数：
            led_enable: LED使能字段
            led_state:  LED状态字段
            buz_enable: 蜂鸣器使能字段
            buz_state:  蜂鸣器状态字段
            imu_enable: IMU校准使能字段
            imu_state:  IMU校准状态字段
        """
        frame = RobotFrame()
        frame.cmd_id = FrameParams.CMD_DEVICE_CONTROL
        frame.data = [
            led_enable & 0xFF, led_state & 0xFF,
            buz_enable & 0xFF, buz_state & 0xFF,
            imu_enable & 0xFF, imu_state & 0xFF,
        ]
        if self.send_frame(frame):
            print(f"[发送] 外设控制 -> LED({led_enable},{led_state}) 蜂鸣器({buz_enable},{buz_state}) IMU({imu_enable},{imu_state})")
            return True
        return False