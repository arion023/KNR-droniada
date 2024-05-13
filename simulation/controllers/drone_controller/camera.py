from controller import Robot, Camera
import numpy as np
import os
from PIL import Image

class DroneCamera:
    def __init__(self, drone_node, fps, timestep, folder_path):
        self.drone_node = drone_node
        self.timestep = timestep

        #to use gyro first initialize drone, because it enables gyro
        self.gyro = self.drone_node.getDevice('gyro')

        self.camera = self.drone_node.getDevice('camera')
        self.camera_rate = 1000 // fps
        self.camera.enable(self.camera_rate)

        self.start_time = self.drone_node.getTime()
        self.image_counter = 0

        #initalize folder for images
        self.folder_path = folder_path
        os.makedirs(self.folder_path, exist_ok=True)  # Create folder if it does not exist

        # Gimbal
        self.camera_roll_motor = self.drone_node.getDevice("camera roll")
        self.camera_pitch_motor = self.drone_node.getDevice("camera pitch")

        #camera sensors
        self.pitch_sensor = drone_node.getDevice('camera pitch sensor')
        self.pitch_sensor.enable(timestep)
        self.roll_sensor = drone_node.getDevice('camera roll sensor')
        self.roll_sensor.enable(timestep)




    def gimbal_stabilize(self):
        """Stabilize camera (gimbal)."""
        acceleration = self.gyro.getValues()
        self.camera_roll_motor.setPosition(-0.115 * acceleration[0])
        self.camera_pitch_motor.setPosition(-0.1 * acceleration[1])


    def move_camera(self):
        #TODO get instructions from que and assign them to delta_camera_position
        delta_camera_position = [0.1, 0] # delta_pitch, delta_roll
        delta_pitch, delta_roll = delta_camera_position

        camera_pitch = self.pitch_sensor.getValue()
        camera_roll = self.roll_sensor.getValue()

        self.camera_roll_motor.setPosition(camera_roll + delta_roll)
        self.camera_pitch_motor.setPosition(camera_pitch + delta_pitch)

        return True

    def run(self):
        self.gimbal_stabilize()
        self.move_camera()

    def capture_image(self):
        current_time = self.drone_node.getTime()
        elapsed_time = (current_time - self.start_time) * 1000  # Elapsed time in milliseconds
        image_bytes = self.camera.getImage()
        self.image_counter += 1

        if self.image_counter % 2 == 0:
            try:
                image_array = np.frombuffer(image_bytes, dtype=np.uint8).reshape((240, 400, 4))
                img = Image.fromarray(image_array[..., [2, 1, 0, 3]], 'RGBA')
                img = img.convert('RGB')

                file_path = os.path.join(self.folder_path, f"image_{self.image_counter}.jpg")
                img.save(file_path, 'JPEG', quality=90)

                print(f"Time since start: {int(elapsed_time)} ms")
            except ValueError as e:
                print(f"Failed to handle image data: {e}")

# Użycie klasy Kamera
if __name__ == "__main__":
    drone = Robot()
    timestep = int(drone.getBasicTimeStep())
    folder_path = "path_to_save_images"

    kamera = Camera(drone, timestep, folder_path)

    while drone.step(timestep) != -1:
        kamera.capture_image()