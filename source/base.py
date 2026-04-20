import serial
import time
import sys
sys.path.append('/home/sunrise/YDLidar-SDK/build/python')
import ydlidar
from typing import *
from params import *
import math

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
    
    def twist_to_wheel_speeds(self, v, omega, wheel_base = ChassisParams.wheel_base):
        """
        将线速度 v 和角速度 omega 转换为差速驱动机器人的左右轮速度。

        参数：
            v (float): 机器人线速度（米/秒），向前为正。
            omega (float): 机器人角速度（弧度/秒），逆时针为正。
            wheel_base (float): 左右轮中心距（米）。

        返回：
            tuple: (left_speed, right_speed) 左轮速度（米/秒）和右轮速度（米/秒）。
        """
        left_speed = v - (omega * wheel_base) / 2
        right_speed = v + (omega * wheel_base) / 2
        left_mm_s = int(round(left_speed * 1000))
        right_mm_s = int(round(right_speed * 1000))
        self.send_speed_control(left_mm_s, right_mm_s)
        return left_speed, right_speed

    def move_to_pose(self, target_pose, linear_speed=0.2, angular_speed=0.5):
        """
        开环移动到目标位姿 (x, y, theta)，theta 顺时针为正，单位度。
        运动顺序：原地旋转 -> 直线 -> 原地旋转。
        
        参数：
            target_pose (tuple): (x, y, z) 单位：米，米，度
            linear_speed (float): 直线移动速度（m/s），默认 0.2
            angular_speed (float): 原地旋转速度（rad/s，顺时针为正），默认 0.5
        
        返回：
            
        """
        x, y = target_pose[0], target_pose[1]
        dist = math.hypot(x, y)
        
        # 如果距离大于 1厘米，才执行指向目标的旋转和直线行走
        if dist > 0.01:
            theta = math.atan2(x, y) 
            print(f"[信息] 目标距离 {dist:.3f}m，正在转向目标方向...")
            self._rotate_angle(theta, angular_speed)
            self._move_straight(dist, linear_speed)
            
            # 如果有最终朝向要求，计算剩余旋转量
            if len(target_pose) >= 3:
                # 这里的 target_pose[2] 是弧度还是角度？
                # 建议在 move_to_pose 统一使用弧度计算
                z_rad = math.radians(target_pose[2])
                end_theta = z_rad - theta
                self._rotate_angle(end_theta, angular_speed)
        else:
            # 距离太近，视为原地旋转
            print("[信息] 距离极近，执行原地旋转...")
            if len(target_pose) >= 3:
                # 直接旋转到目标要求的相对角度
                target_angle_rad = math.radians(target_pose[2])
                self._rotate_angle(target_angle_rad, angular_speed)

    def navigate_waypoints(self, target_poses, linear_speed=0.15, angular_speed=0.5):
        """
        按照全局坐标点序列进行导航。
        
        参数：
            target_poses (list): 坐标列表，如 [[x0, y0], [x1, y1, z1]]。
                                 第一个点作为机器人当前的起始参考位姿。
            linear_speed (float): 线速度 (m/s)。
            angular_speed (float): 角速度 (rad/s)。
        """
        if len(target_poses) < 2:
            print("[错误] 导航至少需要一个起点和一个终点。")
            return

        # 1. 设定初始状态（从第一个点获取当前的 X, Y 和 朝向）
        curr_x, curr_y = target_poses[0][0], target_poses[0][1]
        curr_theta_deg = target_poses[0][2] if len(target_poses[0]) >= 3 else 0.0

        # 遍历从第二个点开始的所有目标点
        for i in range(1, len(target_poses)):
            target = target_poses[i]
            tar_x, tar_y = target[0], target[1]
            
            # 计算当前点到下一个目标的位移向量和距离
            dx = tar_x - curr_x
            dy = tar_y - curr_y
            dist = math.hypot(dx, dy)

            print(f"\n[导航] 正在前往第 {i} 个目标点: ({tar_x}, {tar_y})")

            # 阶段 A: 如果距离超过 1mm，则执行移动逻辑
            if dist > 0.001:
                # 计算目标点在父坐标系下的绝对角度 (基于 Y+ 为 0°, X+ 为 90°)
                abs_target_rad = math.atan2(dx, dy)
                abs_target_deg = math.degrees(abs_target_rad)

                # 角度归一化处理：计算当前朝向与目标朝向的最短角度差
                # 公式将结果限制在 [-180, 180] 之间，防止机器人为了转小角度而绕圈 360°
                rel_angle_deg = (abs_target_deg - curr_theta_deg + 180) % 360 - 180
                
                # 第一步：原地旋转，对准目标点方向
                self._rotate_angle(math.radians(rel_angle_deg), angular_speed)
                
                # 第二步：直线行进，到达目标坐标
                self._move_straight(dist, linear_speed)
                
                # 行进完成后，更新当前的绝对朝向
                curr_theta_deg = abs_target_deg
            
            # 阶段 B: 如果该目标点指定了最终朝向 (z轴角度)
            if len(target) >= 3:
                final_z_deg = target[2]
                # 同样使用最短路径算法计算需要旋转的角度
                rel_final_deg = (final_z_deg - curr_theta_deg + 180) % 360 - 180
                self._rotate_angle(math.radians(rel_final_deg), angular_speed)
                # 更新当前朝向为设定的目标角度
                curr_theta_deg = final_z_deg

            # 更新当前坐标，为计算下一个路点做准备
            curr_x, curr_y = tar_x, tar_y

        print("\n[导航] 所有航路点任务已完成。")

    def _rotate_angle(self, angle_rad, angular_speed):
        """
        原地旋转指定角度（顺时针为正）。
        参数：
            angle_rad (float): 旋转角度（弧度），正为顺时针
            angular_speed (float): 旋转速度大小（rad/s，顺时针为正）
        """
        if abs(angle_rad) < 0.001: # 角度太小就不转
            return
            
        duration = abs(angle_rad) / angular_speed
        # 确定方向
        omega_cmd = -angular_speed if angle_rad > 0 else angular_speed
        
        print(f"[动作] 开始旋转: 弧度={angle_rad:.2f}, 时长={duration:.2f}s")
        self.twist_to_wheel_speeds(0.0, omega_cmd)
        time.sleep(duration)
        
        # 强制停止
        self.send_speed_control(0, 0)
        time.sleep(0.1) # 给硬件一点反应时间
    
    def _move_straight(self, distance, linear_speed):
        """
        直线移动指定距离（向前为正）。
        参数：
            distance (float): 距离（米）
            linear_speed (float): 线速度（m/s）
        """
        if distance == 0:
            return
        duration = distance / linear_speed
        self.twist_to_wheel_speeds(linear_speed, 0.0)
        time.sleep(duration * ChassisParams.行进补偿)
        self.send_speed_control(0, 0)
        time.sleep(0.05)
