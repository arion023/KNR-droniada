from gpiozero import AngularServo
from time import sleep
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory
Device.pin_factory = PiGPIOFactory()


#zakres katow, <90, -25>
#maksymalnie do dolu = 90 stopni
#maksymalnie do przodu = -25 stopni
#prostopadle do podloza, okolo 70,65 stopni, jesli bedzie potrzebne to jeszcze ten kat do znalezienia eksperymentalnie
#wtedy zmienic funckje look_down_perpendicular

class CameraServo:
    def __init__(self):
        self.servo = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.min_angle = -25
        self.max_angle = 90

    def go_up(self):
        if self.servo.angle - 0.5 >= self.min_angle:
            self.servo.angle -= 0.5
        else:
            self.servo.angle = self.min_angle
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle

    def go_down(self): 
        if self.servo.angle + 0.5 <= self.max_angle:
            self.servo.angle += 0.5
        else:
            self.servo.angle = self.max_angle
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle

    def look_down(self): 
        self.servo.angle = 90
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle
    
    def look_down_perpendicular(self): 
        self.servo.angle = 65
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle
    
    def look_up(self): 
        self.servo.angle = -25
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle

    def set_angle(self, x):
        if x < self.min_angle:
            self.servo.angle = self.min_angle
        elif x > self.max_angle:
            self.servo.angle = self.max_angle
        else:
            self.servo.angle = x
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle
    
    def max_positions_test(self):
        while (True):
            servo.set_angle(90)
            sleep(2)
            servo.set_angle(-25)
            sleep(2)


servo = CameraServo()
servo.max_positions_test()