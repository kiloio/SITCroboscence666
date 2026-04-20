import cv2
import numpy as np
from hobot_vio import libsrcampy as srcampy
import time
from dataclasses import dataclass
from params import *

class Camera:
    def __init__(self):
        self.cam = srcampy.Camera()
        self.cam.open_cam(0, -1, 30, [1920], [1080])

    def get_bgr_photo(self):
        """
        获取BGR格式图像
        返回：BGR格式图像
        """
        img_nv12 = self.cam.get_img(2, 1920, 1080)
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
        检测图像中的指定颜色物体，绘制轮廓和中心点
        参数: image - BGR 格式图像
        返回: 识别内容物，绘制后的图像
        """
        objects_list = []
        i = 0
        # 转换为 HSV 颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        result = image.copy()
        for color_name, (lower, upper) in AppleHSVParams.threshold.items():
            # 生成掩膜
            mask = cv2.inRange(hsv, lower, upper)
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
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
                apple = AppleInfo(i,color_name,center_x,center_y)
                objects_list.append(apple)
                i += 1

        return objects_list, result

    def close(self):
        self.cam.close_cam()
