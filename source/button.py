import time
from params import *
import Hobot.GPIO as GPIO

class Button:
    def __init__(self):
        GPIO.setmode(GPIOParams.MODE)
        GPIO.setup(GPIOParams.StartButtonNum, GPIO.IN)

    def wait_press_start_button(self):
        while(True):
            if(GPIO.input(GPIOParams.StartButtonNum) == GPIOParams.StartButtonTriggerMode):
                time.sleep(20/1000)
                if(GPIO.input(GPIOParams.StartButtonNum) == GPIOParams.StartButtonTriggerMode):
                    break