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
    code, result = cam.detect_color_objects(cam.save_bgr_photo("/home/sunrise/qcode.png"))
    if len(result) > 0:
        cv2.imwrite("/home/sunrise/qcode_result.jpg", result)
    # if len(code) > 0:
    #     qr_info = code[0]
    #     print("🤣🤣🤣🤣🤣🤣")
    #     print(f"识别结果: {qr_info.qr_data}, 位置: x={qr_info.x}, y={qr_info.y}, w={qr_info.w}, h={qr_info.h}")        
    # button.enableLED()
    # time.sleep(2)
    # button.disableLED()
    # button.wait_press_start_button()
    # button.enableLED()
    # robot.navigate_waypoints([[0,0,0],[0*a,0.1,0]])
    # calibrator.full_calibration(30)

    # robot.navigate_waypoints([[0,0,0],[0*a,1*a,0]])
    # # robot.navigate_waypoints([[0*a,1*a,0],[0*a,1*a,180],[0,2*a,0]])
    # img = cam.save_bgr_photo("/home/sunrise/out.jpg")
    # apcolors,rpicture = cam.detect_color_objects(img)
    # cv2.imwrite("/home/sunrise/rout.jpg", rpicture)
    # if len(apcolors) > 0:
    #     apple = apcolors[0]
    #     apcolor = apple.color
    # else:
    #     apcolor = None
    # if apcolor == "Red":
    #     robot.navigate_waypoints([[0*a,1*a,0],[0*a,1*a,180],[0,0,180]])
    #     # robot._rotate_angle(2*3.14,0.5)
    # else:
    #     robot.navigate_waypoints([[0*a,1*a,-180],[0*a,1*a,-180],[0,0,-180]])
    #     # robot._rotate_angle(-2*3.14,0.5)

    # robot.navigate_waypoints([[0*a,1*a,0],[0,0,0]])

if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        robot.close()
        button.close()
        print("程序被用户中断")

    finally:
        robot.close()
        button.close()
        print("程序结束")
