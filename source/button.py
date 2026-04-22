import time
from params import *
import Hobot.GPIO as GPIO

class Button:
    def __init__(self):
        GPIO.setmode(GPIOParams.MODE)
        GPIO.setup(GPIOParams.StartButtonNum, GPIO.IN)
        GPIO.setup(GPIOParams.StartLED, GPIO.OUT, initial=GPIO.LOW)

    def wait_press_start_button(self):
        while(True):
            if(GPIO.input(GPIOParams.StartButtonNum) == GPIOParams.StartButtonTriggerMode):
                time.sleep(20/1000)
                if(GPIO.input(GPIOParams.StartButtonNum) == GPIOParams.StartButtonTriggerMode):
                    break

    def enableLED(self):
        GPIO.output(GPIOParams.StartLED, GPIO.HIGH)

    def disableLED(self):
        GPIO.output(GPIOParams.StartLED, GPIO.LOW)

    def close(self):
        self.disableLED()
        GPIO.cleanup()