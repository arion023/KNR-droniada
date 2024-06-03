import sys
import os
import RPi.GPIO as GPIO
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
#from DFRobot_TMF8x01 import DFRobot_TMF8801 as tof
from lib.DFRobot_TMF8x01 import DFRobot_TMF8701 as tof

tof = tof(enPin = -1, intPin = -1, bus_id = 1)

# # Ustawienia pinów
# pin_pwm = 12  # pin PWM (można zmienić na dowolny obsługujący PWM)
# pin_in1 = 23  # pin IN1 (można zmienić)
# pin_in2 = 24  # pin IN2 (można zmienić)

# # Ustawienia GPIO
# GPIO.setmode(GPIO.BCM)  # Używamy numeracji BCM
# GPIO.setup(pin_pwm, GPIO.OUT)
# GPIO.setup(pin_in1, GPIO.OUT)
# GPIO.setup(pin_in2, GPIO.OUT)

# # Ustawienia PWM
# pwm_freq = 1000  # Częstotliwość PWM w Hz
# pwm = GPIO.PWM(pin_pwm, pwm_freq)
# pwm.start(0)  # Uruchom PWM z wypełnieniem 0%

class GripperController:
    def __init__(self):
        pass

    # def open_claw(self):
    #     """
    #     Obrót silnika DC o pół obrotu w jednym kierunku.
    #     """
    #     GPIO.output(pin_in1, GPIO.HIGH)
    #     GPIO.output(pin_in2, GPIO.LOW)
    #     pwm.ChangeDutyCycle(50)  # 50% prędkości naprzód
    #     time.sleep(1)  # Czas na wykonanie pół obrotu
    #     pwm.ChangeDutyCycle(0)  # Stop

    # def close_claw(self):
    #     """
    #     Obrót silnika DC o pół obrotu w drugim kierunku.
    #     """
    #     GPIO.output(pin_in1, GPIO.LOW)
    #     GPIO.output(pin_in2, GPIO.HIGH)
    #     pwm.ChangeDutyCycle(50)  # 50% prędkości wstecz
    #     time.sleep(1)  # Czas na wykonanie pół obrotu
    #     pwm.ChangeDutyCycle(0)  # Stop

    def get_distance_claw_sensor(self):
        print("Initialization ranging sensor TMF8x01......", end=" ")
        tof.begin()
        #time.sleep(1)
        # while tof.begin() != 0:
        #     print("Initialization failed")
        # print("Initialization done.")
        
        # print("Software Version: ", tof.get_software_version())
        # print("Unique ID: %X" % tof.get_unique_id())
        # print("Model: ", tof.get_sensor_model())
        
        tof.start_measurement(calib_m=tof.eMODE_CALIB, mode=tof.ePROXIMITY)
        
        #while True:
        #if tof.is_data_ready():
        tof.is_data_ready() == True
        print("Distance = %d mm" % tof.get_distance_mm())

    def test_importu(self):
        print("test importu")


if __name__ == "__main__":
    try:
        gripper = GripperController()
        gripper.get_distance_claw_sensor()
    finally:
        GPIO.cleanup()
