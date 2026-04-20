import cv2
import numpy as np
from hobot_vio import libsrcampy as srcampy
import time
from dataclasses import dataclass
from params import *

class Camera:

    def __init__(self):
        pass

    def get_bgr_photo(self):
        """
        获取BGR格式图像
        返回：BGR格式图像
        """
        cam = srcampy.Camera()
        cam.open_cam(0, -1, 30, [1920], [1080])
        img_nv12 = cam.get_img(2, 1920, 1080)
        cam.close_cam()
        # 转换为 numpy 数组并 reshape
        h, w = 1080, 1920
        yuv = np.frombuffer(img_nv12, dtype=np.uint8).reshape((h * 3 // 2, w))
        # NV12 转 BGR
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
        return bgr
    
    def save_bgr_photo(self, path: str = "photo.jpg"):
        """
        保存BGR格式图像于本地
        返回：BGR格式图像
        """
        bgr = self.get_bgr_photo()
        cv2.imwrite(path, bgr)
        print("已保存BGR图像于"+str(path))
        return bgr

    def detect_color_objects(self, image):
        """
        检测图像中的指定颜色物体，支持每个颜色多个HSV范围
        参数: image - BGR 格式图像
        返回: 识别内容物，绘制后的图像
        """
        objects_list = []
        i = 0
        
        # 转换为 HSV 颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        result = image.copy()
        
        for color_name, ranges in AppleHSVParams.color_ranges.items():
            # 合并当前颜色的所有掩膜
            combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            
            # 遍历当前颜色的所有HSV范围
            for lower, upper in ranges:
                mask = cv2.inRange(hsv, lower, upper)
                combined_mask = cv2.bitwise_or(combined_mask, mask)
            
            # 可选：形态学操作去除噪声（根据实际效果调整）
            kernel = np.ones((3, 3), np.uint8)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            
            # 在原图上绘制
            for cnt in contours:
                # 过滤点数过少的轮廓（噪声）
                if cnt.shape[0] < 150:
                    continue
                
                # 绘制轮廓（绿色线条）
                cv2.drawContours(result, [cnt], -1, (0, 255, 0), 2)
                
                # 计算外接矩形，得到中心点
                x, y, w, h = cv2.boundingRect(cnt)
                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                cv2.circle(result, (center_x, center_y), 5, (0, 255, 0), -1)
                
                # 创建物体信息对象
                apple = AppleInfo(i, color_name, center_x, center_y)
                objects_list.append(apple)
                i += 1
        
        return objects_list, result