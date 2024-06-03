from GripperController import GripperController
from CameraController import CameraController
from CV import BallFinder
#import FlightControllerInterface
import subprocess
import time
from time import sleep

def run_mission():
    
    #take off
    #poleć do miejsca gdzie zrobi fote

    #zrób fote 
    #przetwórz fote otrzymaj pola piłek

    #poleć do piłki 1
    #poleć do kubła

    #poleć do piłki 1
    #poleć do kubła

    #poleć do piłki 1
    #poleć do kubła
    
    pass

def find_balls_img(img):
    pass



def test_camera_controller():
    # inicjalizacja obiektu
    camera_controller_obj = CameraController()
    
    # patrzy w dół
    camera_controller_obj.set_angle(33)
    # sleep żeby miało czas się ruszyć
    sleep(0.2)
    # strzel fote
    camera_controller_obj.take_picture()

    #patrzy do przodu pod kątem lekko w dół
    camera_controller_obj.set_angle(-25)
    sleep(0.2)
    camera_controller_obj.take_picture()

def test_gripper_controller():
    gripper_controller = GripperController()
    gripper_controller.get_distance_claw_sensor()
    pass

if __name__ == "__main__":
    test_camera_controller()
    # test_camera_pic()