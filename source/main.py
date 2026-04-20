import time
from base import *
from lidar import *
from calibration import *
from button import *
from camera import *
from params import *
robot = RobotController()
lidar = YDLidarScanner("X3")
calibrator = WallCalibration(robot, lidar)
button = Button()
cam = Camera()

a = 0.6

def main():
    robot.navigate_waypoints([[0,0,0],[0*a,1*a,0]])
    img = cam.save_bgr_photo("/home/sunrise/out.jpg")
    apcolors,rpicture = cam.detect_color_objects(img)
    cv2.imwrite("/home/sunrise/rout.jpg", rpicture)
    if len(apcolors) > 0:
        apple = apcolors[0]
        apcolor = apple.color
    else:
        apcolor = None
    if apcolor == "Red":
        robot._rotate_angle(2*3.14,0.5)
    else:
        robot._rotate_angle(-2*3.14,0.5)

    robot.navigate_waypoints([[0*a,1*a,0],[0,0,0]])

if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        robot.close()
        print("程序被用户中断")

    finally:
        robot.close()
        print("程序结束")
