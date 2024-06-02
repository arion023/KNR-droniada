from GripperController import GripperController
from CameraController import CameraController
from CV import BallFinder
#import FlightControllerInterface
import subprocess
import time

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
    camera_controller_obj = CameraController()
    #patrzy w dół
    camera_controller_obj.set_angle(33)
    #patrzy do przodu pod kątem lekko w dół
    camera_controller_obj.set_angle(-25)
    

    # try:
    #     # Run the CameraController.py script
    #     subprocess.run(['python3', 'CameraController.py'], check=True)
    # except subprocess.CalledProcessError as e:
    #     print(f"Error running CameraController.py: {e}")

def test_gripper_controller():
    gripper_controller = GripperController()
    gripper_controller.get_distance_claw_sensor()
    pass
# if __name__ == "__main__":
#     while True:
#         test_importu()
#         time.sleep(1)  # Add a delay if needed to prevent rapid execution
#         test_rotate_camera()
#         time.sleep(1)  # Add a delay if needed to prevent rapid execution

# gripper_controller.test_importu()


# test_grippercontroller()
test_camera_controller()