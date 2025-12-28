import sys
from typing import *
from dataclasses import *
sys.path.append('/home/sunrise/YDLidar-SDK/build/python')
import ydlidar

@dataclass
class FrameParams:
    FRAME_HEADER = 0x55
    FRAME_TAIL = 0xBB
    FRAME_LENGTH_Data = 0x06
    FRAME_LENGTH_TOTAL = 1 + 1 + 1 + FRAME_LENGTH_Data + 1 + 1  # header + id + len + data(6) + checksum + tail = 11

    # Command IDs
    CMD_SPEED_CONTROL = 0x01      # 上位机 -> STM32 : 速度控制
    CMD_SPEED_FEEDBACK = 0x02     # STM32 -> 上位机 : 速度反馈
    CMD_IMU_ACCEL = 0x03          # STM32 -> 上位机 : 加速度 (Ax,Ay,Az)
    CMD_IMU_GYRO = 0x04           # STM32 -> 上位机 : 角速度 (Wx,Wy,Wz)
    CMD_IMU_EULER = 0x05          # STM32 -> 上位机 : 欧拉角 (Roll,Pitch,Yaw)
    CMD_SENSOR_DATA = 0x06        # STM32 -> 上位机 : 传感器数据（电压 + 2 预留）
    CMD_DEVICE_CONTROL = 0x07     # 上位机 -> STM32 : LED/Buzzer/IMU校准
    CMD_PID_PARAMS = 0x08         # 上位机 -> STM32 : PID 参数 (P,I,D)

@dataclass
class MotionBoardParams:
    PORT = "/dev/ttyS3"
    BAUDRATE = 115200

class LidarParams:
    """雷达配置管理器"""
    
    # 雷达类型定义
    X3 = "X3"
    TMINI = "TMini"
    
    DEFAULT_PORT = "/dev/ttyUSB0"

    # 雷达配置数据库
    CONFIGS: Dict[str, Dict[str, Any]] = {
        X3: {
            'baudrate': 115200,
            'lidar_type': ydlidar.TYPE_TRIANGLE,
            'device_type': ydlidar.YDLIDAR_TYPE_SERIAL,
            'scan_frequency': 6.0,  # Hz
            'sample_rate': 3,
            'single_channel': True,
            'max_angle': 180.0,     # 度
            'min_angle': -180.0,    # 度
            'max_range': 8.0,       # 米
            'min_range': 0.10,      # 米
            'intensity': False
        },
        TMINI: {
            'baudrate': 230400,
            'lidar_type': ydlidar.TYPE_TRIANGLE,
            'device_type': ydlidar.YDLIDAR_TYPE_SERIAL,
            'scan_frequency': 10.0,  # 提高扫描频率到10Hz
            'sample_rate': 4,
            'max_angle': 180.0,
            'min_angle': -180.0,
            'max_range': 12.0,
            'min_range': 0.05,
            'intensity': True,
            'auto_reconnect': True,
            'fixed_resolution': True
        },
    }
    
    # 默认雷达端口
    
    @classmethod
    def get_config(cls, lidar_type: str) -> Dict[str, Any]:
        """获取指定雷达类型的配置"""
        if lidar_type not in cls.CONFIGS:
            raise ValueError(f"不支持的雷达类型: {lidar_type}。支持的雷达: {list(cls.CONFIGS.keys())}")
        return cls.CONFIGS[lidar_type].copy()
    
    @classmethod
    def get_supported_types(cls) -> list:
        """获取支持的雷达类型列表"""
        return list(cls.CONFIGS.keys())
    
    @classmethod
    def get_baudrate(cls, lidar_type: str) -> int:
        """获取指定雷达的波特率"""
        config = cls.get_config(lidar_type)
        return config['baudrate']