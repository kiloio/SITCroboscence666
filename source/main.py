import time
from base import *
from lidar import *
from calibration import *
from button import *
from camera import *
robot = RobotController()
lidar = YDLidarScanner("X3")
calibrator = WallCalibration(robot, lidar)
button = Button()
cam = Camera()

a = 0.1

def main():
    robot.navigate_waypoints([[0,0,0],[0*a,-1*a],[1*a,-1*a],[2*a,-2*a],[-1*a,-1*a],[0,0,0]])
    time.sleep(2)
    pass

if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        robot.close()
        cam.close()
        print("程序被用户中断")

    finally:
        robot.close()
        cam.close()
        print("程序结束")