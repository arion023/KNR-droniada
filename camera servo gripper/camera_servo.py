from gpiozero import AngularServo
from time import sleep
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory
Device.pin_factory = PiGPIOFactory()

class camera_servo:
    servo =AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)

    def go_up(self, x): #x - aktualny kąt
        self.servo.angle = x
        self.servo.angle = x - 0.5
        return self.servo.angle

    def go_down(self, x): #x - aktualny kąt
        self.servo.angle = x
        self.servo.angle = x + 0.5
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle

    def look_down(self): 
        self.servo.angle = 90
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle

    def set_angle(self, x):
        servo.angle = x
        print("aktualny kąt:" + str(self.servo.angle))
        return self.servo.angle
    
    def test(self):
        while (True):
            self.set_angle(90)
            sleep(2)
            self.set_angle(0)
            sleep(2)
            self.look_down()
            sleep(2)




 

##min_pulse_width=0.0005, max_pulse_width=0.0025
#while (True):
 #   servo.angle = a
  #  sleep(2)
   # while a < 30:
    #   a+=0.5
     #  go_down(a)
    #sleep(2)
    #while a > -20:
     #  a-=0.5
     #  go_down(a) 
    #sleep(2)  
    #servo.angle = 90
    #sleep(2)
    