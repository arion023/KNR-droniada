from GripperController import GripperController
from CameraController import CameraController
from CV import BallFinder
#import FlightControllerInterface
import subprocess
import time
from time import sleep

def run_mission():
    camera_controller_obj = CameraController()
    gripper_controller_obj = GripperController()
    b
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
    print("kamera patrzy w dół")
    camera_controller_obj.set_angle(33)
    # sleep żeby miało czas się ruszyć
    sleep(0.2)
    # strzel fote
    print("zrobienie zdjęcia")
    camera_controller_obj.take_picture()

    #patrzy do przodu pod kątem lekko w dół
    print("kamera patrzy na skos")
    camera_controller_obj.set_angle(-25)
    sleep(0.2)
    print("zrobienie zdjęcia")
    camera_controller_obj.take_picture()

def test_gripper_controller():
    print("czytanie z czujnika odbiciowego")
    gripper_controller_obj = GripperController()
    distance_from_sensor = gripper_controller_obj.get_distance_claw_sensor()
    print(distance_from_sensor)
    

if __name__ == "__main__":
    #działa
    #test ruchu kamery i zrobienia zdjęcia
    test_camera_controller()

    test_gripper_controller()


    