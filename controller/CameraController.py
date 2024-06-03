from gpiozero import AngularServo
from time import sleep
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory
#import ffmpeg
import numpy as np
from PIL import Image
import io
import os
import ffmpeg

'''
Servo kable:
 niebieski GND
 czerwony 5V 
 zielony na GPIO 18

 Kamera: USB 
'''

Device.pin_factory = PiGPIOFactory()

class CameraController:
    def __init__(self):
        self.camera_controller_obj = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.min_angle = -25
        self.max_angle = 90
        self.current_angle = 0  # Assume starting at 0 degrees
        self.image_folder = 'images'
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

    def go_up(self):
        if self.current_angle - 0.5 >= self.min_angle:
            self.current_angle -= 0.5
        else:
            self.current_angle = self.min_angle
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        self.take_picture()
        return self.current_angle

    def go_down(self): 
        if self.current_angle + 0.5 <= self.max_angle:
            self.current_angle += 0.5
        else:
            self.current_angle = self.max_angle
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        self.take_picture()
        return self.current_angle

    def look_down(self): 
        self.current_angle = 90
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        self.take_picture()
        return self.current_angle
    
    def look_down_perpendicular(self): 
        self.current_angle = 65
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        self.take_picture()
        return self.current_angle
    
    def look_up(self): 
        self.current_angle = -25
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        self.take_picture()
        return self.current_angle

    def set_angle(self, x):
        if x < self.min_angle:
            self.current_angle = self.min_angle
        elif x > self.max_angle:
            self.current_angle = self.max_angle
        else:
            self.current_angle = x
        self.camera_controller_obj.angle = self.current_angle
        print("aktualny kąt:" + str(self.current_angle))
        return self.current_angle
    
    def max_positions_test(self):
        #while True:
        self.set_angle(33)
        sleep(1)
        self.set_angle(-25)
        #sleep(2)

    def take_picture(self):
        print(f"Taking picture at angle: {self.current_angle}")
        try:
            stream_url = 'http://localhost:8080/?action=stream'
            out, _ = (
                ffmpeg
                .input(stream_url)
                .output('pipe:', vframes=1, format='image2', vcodec='mjpeg')
                .run(capture_stdout=True, capture_stderr=True)
            )
            image_np = np.frombuffer(out, np.uint8)
            image = Image.open(io.BytesIO(image_np))
            image.save(os.path.join(self.image_folder, f'output_{self.current_angle}.jpg'))
        except ffmpeg.Error as e:
            print("ffmpeg error:", e.stderr.decode('utf8'))

if __name__ == "__main__":
    camera_controller_obj = CameraController()
    camera_controller_obj.max_positions_test()
    camera_controller_obj.take_picture()
    
