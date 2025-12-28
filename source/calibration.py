# calibration.py
"""
导航校准模块
包含基于激光雷达的墙面校准功能
"""

import time
import math

class WallCalibration:
    """
    墙面校准类
    负责基于激光雷达数据进行机器人位姿校准
    """
    
    def __init__(self, robot_controller, lidar_scanner):
        """
        初始化校准器
        
        Args:
            robot_controller: RobotController实例，用于控制机器人移动
            lidar_scanner: YDLidarScanner实例，用于获取激光雷达数据
        """
        self.robot = robot_controller
        self.lidar = lidar_scanner
        self.running = False
        
    def align_to_wall(self, tolerance_percent=0.05, max_rotation_speed=50):
        """
        校准机器人平行于前墙
        
        Args:
            tolerance_percent: 两侧距离差的容忍百分比（默认0.05%）
            max_rotation_speed: 最大旋转速度（默认50）
        """
        tolerance = tolerance_percent / 100
        self.running = True
        
        while self.running:
            # 获取左右两侧的雷达数据
            lidar_data = self.lidar.获取数据_获取单次旋转雷达区间角度数据([
                (-30, 0),
                (0, 30)
            ])
            
            print(f"雷达左右两侧距离：{lidar_data}")
            
            # 数据有效性检查
            if lidar_data is None or len(lidar_data) < 2:
                print("雷达数据无效，继续尝试...")
                time.sleep(0.1)
                continue
            
            left_distance = lidar_data[0]
            right_distance = lidar_data[1]
            
            # 计算两侧距离差
            distance_diff = right_distance - left_distance
            
            # 确定旋转方向
            if abs(distance_diff) < tolerance:
                # 已达到平行状态，停止旋转
                self.robot.send_speed_control(0, 0)
                print("已平行于墙面，校准完成")
                break
            
            # 计算旋转速度（基于距离差的比例控制）
            rotation_speed = min(max_rotation_speed, abs(distance_diff) * 1000)
            
            # 确定旋转方向
            if distance_diff < 0:
                # 右侧比左侧近，顺时针旋转
                left_speed = rotation_speed
                right_speed = -rotation_speed
            else:
                # 左侧比右侧近，逆时针旋转
                left_speed = -rotation_speed
                right_speed = rotation_speed
            
            # 发送速度指令
            self.robot.send_speed_control(left_speed, right_speed)
            
            # 短暂等待，避免过度响应
            time.sleep(0.1)
        
        self.running = False
    
    def approach_to_wall(self, target_distance_cm=20, tolerance_cm=0.5, max_speed=50):
        """
        校准机器人前进到指定距离的墙面
        
        Args:
            target_distance_cm: 目标距离，单位厘米（默认20cm）
            tolerance_cm: 距离容忍度，单位厘米（默认0.5cm）
            max_speed: 最大前进速度（默认50）
        """
        self.running = True
        
        while self.running:
            # 获取正前方的雷达数据
            front_data = self.lidar.获取数据_获取单次旋转雷达区间角度数据([(-10, 10)])
            
            print(f"前方距离：{front_data}")
            
            # 数据有效性检查
            if front_data is None or len(front_data) < 1:
                print("前方雷达数据无效，继续尝试...")
                time.sleep(0.1)
                continue
            
            # 将米转换为厘米
            current_distance_cm = front_data[0] * 100
            
            # 计算距离差
            distance_diff = current_distance_cm - target_distance_cm
            
            # 检查是否达到目标距离
            if abs(distance_diff) < tolerance_cm:
                self.robot.send_speed_control(0, 0)
                print(f"已到达目标距离 {target_distance_cm}cm，校准完成")
                break
            
            # 计算前进速度（基于距离差的比例控制）
            speed = min(max_speed, abs(distance_diff) * 50)
            
            # 确定前进方向
            if distance_diff > 0:
                # 当前距离大于目标距离，向前移动
                left_speed = speed
                right_speed = speed
            else:
                # 当前距离小于目标距离，向后移动
                left_speed = -speed
                right_speed = -speed
            
            # 发送速度指令
            self.robot.send_speed_control(left_speed, right_speed)
            
            # 短暂等待
            time.sleep(0.1)
        
        self.running = False
    
    def full_calibration(self, approach_distance=20,align_tolerance=0.05, approach_tolerance=0.5):
        """
        执行完整校准流程：先平行于墙面，再前进到指定距离
        
        Args:
            approach_distance: 前进目标距离（厘米）
            align_tolerance: 平行校准容忍度
            approach_tolerance: 前进校准容忍度（厘米）
        """

        self.align_to_wall(tolerance_percent=align_tolerance)
        
        time.sleep(0.5)  # 等待稳定

        self.approach_to_wall(
            target_distance_cm=approach_distance,
            tolerance_cm=approach_tolerance
        )
    
    def stop_calibration(self):
        """
        停止校准过程
        """
        self.running = False
        self.robot.send_speed_control(0, 0)
        print("校准已停止")
    
    def check_connection(self):
        """
        检查所有设备连接状态
        
        Returns:
            bool: 所有设备连接正常返回True，否则返回False
        """
        robot_connected = self.robot.is_connected()
        lidar_connected = self.lidar.is_connected
        
        print(f"机器人控制器连接: {'正常' if robot_connected else '异常'}")
        print(f"激光雷达连接: {'正常' if lidar_connected else '异常'}")
        
        return robot_connected and lidar_connected