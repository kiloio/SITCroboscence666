import time
from base import *
from lidar import *
from calibration import *
robot = RobotController()
lidar = YDLidarScanner("TMini")
calibrator = WallCalibration(robot, lidar)

def main():
    calibrator.full_calibration(30)
    pass

if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        robot.close()
        print("程序被用户中断")

    finally:
        robot.close()
        print("程序结束")