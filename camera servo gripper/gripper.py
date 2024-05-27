from gpiozero import DigitalOutputDevice
from gpiozero import Device
import RPi.GPIO as GPIO
from time import sleep

class Gripper:
    pin1=17
    pin2=27
    def __init__(self):
        self.gripped=False
        GPIO.setmode(GPIO.BCM)        

    def grip(self):
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW) 
        self.gripped=True
        
    
    def no_grip(self):
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)
        self.gripped=False
    
    def test(self):
        while(1):
            self.grip()
            self.no_grip()