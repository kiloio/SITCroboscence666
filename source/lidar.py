import sys
sys.path.append('/home/sunrise/YDLidar-SDK/build/python')
import ydlidar
import time
from params import FrameParams, MotionBoardParams, LidarParams
from typing import List, Tuple, Dict, Optional
import math
import numpy

class YDLidarScanner:
    """
    YDLIDAR 激光雷达扫描器类
    用于获取完整的旋转一周扫描数据
    """
    
    def __init__(self, lidar_type: str = "X3"):
        """
        初始化激光雷达扫描器
        
        Args:
            lidar_type: 雷达类型，支持 "X3", "TMini", "X4"
        """
        self.port = LidarParams.DEFAULT_PORT
        self.laser = None
        self.is_connected = False
        self.is_scanning = False
        
        # 默认配置参数
        self.config = LidarParams.get_config(lidar_type)
        self._initialize()
        self._start_scanning()
    
    def _initialize(self) -> bool:
        """
        初始化激光雷达连接
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            # 初始化操作系统
            ydlidar.os_init()
            
            # 如果未指定端口，自动检测
            if self.port is None:
                ports = ydlidar.lidarPortList()
                if ports:
                    # 选择第一个检测到的端口
                    self.port = list(ports.values())[0]
                    print(f"自动检测到端口: {self.port}")
                else:
                    print("未检测到激光雷达设备")
                    return False
            
            # 创建激光雷达实例
            self.laser = ydlidar.CYdLidar()
            
            # 设置配置参数
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
            if "baudrate" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.config['baudrate'])
            if "lidar_type" in self.config:    
                self.laser.setlidaropt(ydlidar.LidarPropLidarType, self.config['lidar_type'])
            if "device_type" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropDeviceType, self.config['device_type'])
            if "scan_frequency" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, self.config['scan_frequency'])
            if "sample_rate" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropSampleRate, self.config['sample_rate'])
            if "single_channel" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, self.config['single_channel'])
            if "max_angle" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.config['max_angle'])
            if "min_angle" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.config['min_angle'])
            if "max_range" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.config['max_range'])
            if "min_range" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.config['min_range'])
            if "intensity" in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, self.config['intensity'])
            if 'auto_reconnect' in self.config:
                self.laser.setlidaropt(ydlidar.LidarPropAutoReconnect, self.config['auto_reconnect'])
            if 'fixed_resolution' in self.config:   
                self.laser.setlidaropt(ydlidar.LidarPropFixedResolution, self.config['fixed_resolution'])

            # 初始化激光雷达
            ret = self.laser.initialize()
            if ret:
                self.is_connected = True
                print(f"激光雷达初始化成功，端口: {self.port}")
                return True
            else:
                print("激光雷达初始化失败")
                return False
                
        except Exception as e:
            print(f"初始化过程中发生错误: {e}")
            return False
    
    def _start_scanning(self) -> bool:
        """
        开始扫描
        
        Returns:
            bool: 启动扫描是否成功
        """
        if not self.is_connected:
            print("激光雷达未连接")
            return False
        
        try:
            ret = self.laser.turnOn()
            if ret:
                self.is_scanning = True
                print("开始扫描...")
                return True
            else:
                print("启动扫描失败")
                return False
                
        except Exception as e:
            print(f"启动扫描过程中发生错误: {e}")
            return False
    
    def get_single_scan(self) -> Optional[List[Tuple[float, float]]]:
        """
        获取单次扫描数据（旋转一周的数据）
        
        Returns:
            Optional[List[Tuple[float, float]]]: 角度和距离的列表，每个元素为(角度, 距离)
                  如果获取失败则返回None
        """
        if not self.is_scanning:
            print("激光雷达未在扫描状态")
            return None
        
        try:
            scan = ydlidar.LaserScan()
            if ydlidar.os_isOk():
                r = self.laser.doProcessSimple(scan)
                if r:
                    # 计算扫描频率
                    frequency = 1.0 / scan.config.scan_time if scan.config.scan_time > 0 else 0
                    
                    # 收集所有数据点
                    scan_data = []
                    for point in scan.points:
                        scan_data.append((point.angle, point.range))
                    
                    print(f"扫描完成: {len(scan_data)} 个点, 频率: {frequency:.2f} Hz")
                    return scan_data
                else:
                    print("获取激光雷达数据失败")
                    return None
                    
        except Exception as e:
            print(f"获取扫描数据过程中发生错误: {e}")
            return None
    
    def get_continuous_scans(self, num_scans: int = 10, interval: float = 0.1) -> List[List[Tuple[float, float]]]:
        """
        获取连续多次扫描数据
        
        Args:
            num_scans: 扫描次数
            interval: 每次扫描之间的间隔（秒）
            
        Returns:
            List[List[Tuple[float, float]]]: 多次扫描数据的列表
        """
        scans = []
        
        for i in range(num_scans):
            print(f"正在获取第 {i+1}/{num_scans} 次扫描...")
            scan_data = self.get_single_scan()
            if scan_data:
                scans.append(scan_data)
            
            # 等待指定间隔
            if i < num_scans - 1:
                time.sleep(interval)
        
        return scans
    
    # def 数据处理_获取区间弧度均值(self, 数据, 弧度区间):
    #     '''
    #     弧度区间 = [(起始弧度1, 结束弧度1), (起始弧度2, 结束弧度2), ...]
    #     '''
    #     暂存数据 = []
    #     for 单弧度区间 in 弧度区间:
    #         本区间数据 = []
    #         for 单数据 in 数据:
    #             if 单弧度区间[0] <= 单数据[0] <= 单弧度区间[1] and self.config['min_range']<= 单数据[1] <= self.config['max_range']:
    #                 本区间数据.append(单数据[1])
    #         if len(本区间数据) > 0:
    #             暂存数据.append(sum(本区间数据) / len(本区间数据))
    #     if len(暂存数据) == 0:
    #         return None
    #     # print("数据处理暂存数据：",暂存数据,"\n")
    #     return 暂存数据

    def 数据处理_获取区间弧度均值(self, 数据, 弧度区间):
        '''
        弧度区间 = [(起始弧度1, 结束弧度1), (起始弧度2, 结束弧度2), ...]
        '''
        暂存数据 = []
        for 单弧度区间 in 弧度区间:
            本区间数据 = []
            for 单数据 in 数据:
                if 单弧度区间[0] <= 单数据[0] <= 单弧度区间[1]:
                    if self.config['min_range'] <= 单数据[1] <= self.config['max_range']:
                        本区间数据.append(单数据[1])
            
            if len(本区间数据) > 0:
                暂存数据.append(sum(本区间数据) / len(本区间数据))
        
        if len(暂存数据) == 0:
            return None
        # print("数据处理暂存数据：",暂存数据,"\n")
        return 暂存数据

    def 数据处理_获取区间角度均值(self, 数据, 角度区间):
        '''
        角度区间 = [(起始角度1, 结束角度1), (起始角度2, 结束角度2), ...]
        '''
        return self.数据处理_获取区间弧度均值(数据, numpy.radians(角度区间))

    def 获取数据_获取单次旋转雷达区间角度数据(self, 角度区间):
        '''
        角度区间 = [(起始角度1, 结束角度1), (起始角度2, 结束角度2), ...]
        '''
        return self.数据处理_获取区间角度均值(self.get_single_scan(), 角度区间)

    def stop_scanning(self):
        """停止扫描"""
        if self.is_scanning and self.laser:
            self.laser.turnOff()
            self.is_scanning = False
            print("停止扫描")
    
    def disconnect(self):
        """断开连接"""
        if self.is_connected and self.laser:
            self.stop_scanning()
            self.laser.disconnecting()
            self.is_connected = False
            self.laser = None
            print("断开激光雷达连接")
    
    def __del__(self):
        """析构函数，确保资源被正确释放"""
        self.disconnect()