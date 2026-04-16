import time
from base import *
from lidar import *
from calibration import *
from button import *
robot = RobotController()
lidar = YDLidarScanner("X3")
calibrator = WallCalibration(robot, lidar)
button = Button()

def main():
    button.wait_press_start_button()
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