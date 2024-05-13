from controller import Robot, Motor, Camera, GPS, InertialUnit, Gyro, LED, Keyboard, Compass
import math
import numpy as np
import time
import os
from PIL import Image
# Helper functions
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def sign(x):
    return (x > 0) - (x < 0)

# Create the Robot instance and initialize devices
robot = Robot()
timestep = int(robot.getBasicTimeStep())
folder_path = "images_from_camera"
os.makedirs(folder_path, exist_ok=True)  # Utwórz folder jeśli nie istnieje

image_counter = 0

camera = robot.getDevice('camera')
camera.enable(timestep)

front_left_led = robot.getDevice('front left led')
front_right_led = robot.getDevice('front right led')

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

gyro = robot.getDevice('gyro')
gyro.enable(timestep)

keyboard = Keyboard()
keyboard.enable(timestep)
camera.enable(64)
camera_roll_motor = robot.getDevice('camera roll')
camera_pitch_motor = robot.getDevice('camera pitch')
# camera_yaw_motor = robot.getDevice('camera yaw')  # Not used in this example
# Pobierz dostęp do kamery i stawów obrotowych kamery
camera_yaw = robot.getDevice('camera yaw')
camera_pitch = robot.getDevice('camera pitch')
camera_roll = robot.getDevice('camera roll')
yaw_sensor = robot.getDevice('camera yaw sensor')
pitch_sensor = robot.getDevice('camera pitch sensor')
roll_sensor = robot.getDevice('camera roll sensor')

# Ustaw prędkości obrotowe dla poszczególnych stawów
max_speed = 1.0
yaw_sensor.enable(64)
pitch_sensor.enable(64)
roll_sensor.enable(64)

motors = []
for name in ['front left propeller', 'front right propeller', 'rear left propeller', 'rear right propeller']:
    motor = robot.getDevice(name)
    motor.setPosition(float('+inf'))
    motor.setVelocity(1.0)
    motors.append(motor)

# Constants, empirically found
K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts
K_VERTICAL_OFFSET = 0.6   # Vertical offset where the robot actually targets to stabilize itself
K_VERTICAL_P = 3.0        # P constant of the vertical PID
K_ROLL_P = 50.0           # P constant of the roll PID
K_PITCH_P = 30.0          # P constant of the pitch PID

# Variables
target_altitude = 0.5  # The target altitude. Can be changed by the user
sampling_interval = 10
last_sampling_time = time.time()

current_pitch = 0
current_roll = 0

if __name__ == '__main__':
    # Main loop
    while robot.step(timestep) != -1:

        time1 = robot.getTime()

        # Capture image from the camera
        image_bytes = camera.getImage()
        image_counter += 1

        if image_counter % 8 == 0:
            try:
                # Assuming the dimensions are known and correct (400x240)
                # Notice the change in indexing the channels from BGR to RGB
                image_array = np.frombuffer(image_bytes, dtype=np.uint8).reshape((420, 784, 4))
                img = Image.fromarray(image_array[..., [2, 1, 0, 3]], 'RGBA')  # Swap BGR to RGB
                img = img.convert('RGB')  # Convert to RGB if alpha channel is not needed

                # Construct the file path
                file_path = os.path.join(folder_path, f"image_{image_counter}.jpg")

                # Save the image as JPEG with specified quality
                try:
                    img.save(file_path, 'JPEG', quality=90)
                    print(f"Image saved as {file_path}")
                except IOError as e:
                    print(f"Failed to save image: {e}")

            except ValueError as e:
                print(f"Failed to handle image data: {e}")

        #
        roll, pitch, yaw = imu.getRollPitchYaw()
        print(roll, pitch, yaw)

        altitude = gps.getValues()[2]
        roll_velocity, pitch_velocity, _ = gyro.getValues()

        # Blink the front LEDs alternatively with a 1 second rate
        led_state = int(time1) % 2
        front_left_led.set(led_state)
        front_right_led.set(not led_state)

        # Stabilize the Camera by actuating the camera motors according to the gyro feedback
        camera_roll_motor.setPosition(current_roll - 0.115 * roll_velocity)
        camera_pitch_motor.setPosition(current_pitch - 0.1 * pitch_velocity)

        # Transform the keyboard input to disturbances on the stabilization algorithm
        roll_disturbance = pitch_disturbance = yaw_disturbance = 0.0
        # Zdefiniuj początkowe pozycje kamery
        current_pitch = pitch_sensor.getValue()
        current_roll = roll_sensor.getValue()

        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                pitch_disturbance = -1.0
            elif key == Keyboard.DOWN:
                pitch_disturbance = 1.0
            elif key == Keyboard.RIGHT:
                yaw_disturbance = -1.0
            elif key == Keyboard.LEFT:
                yaw_disturbance = 1.0
            elif key == Keyboard.SHIFT + Keyboard.RIGHT:
                roll_disturbance = -0.8
            elif key == Keyboard.SHIFT + Keyboard.LEFT:
                roll_disturbance = 0.8
            elif key == Keyboard.SHIFT + Keyboard.UP:
                target_altitude += 0.05
                print("target altitude: {:.2f} m".format(target_altitude))
            elif key == Keyboard.SHIFT + Keyboard.DOWN:
                target_altitude -= 0.05
                print("target altitude: {:.2f} m".format(target_altitude))

            # Obsługa klawiatury dla kamery
            if key == ord('1'):
                current_pitch += 0.1
                camera_pitch.setPosition(current_pitch)
            elif key == ord('2'):
                current_pitch -= 0.1
                camera_pitch.setPosition(current_pitch)
            elif key == ord('A'):
                current_roll += 0.1
                camera_roll.setPosition(current_roll)
            elif key == ord('D'):
                current_roll -= 0.1
                camera_roll.setPosition(current_roll)

        # Pobierz kolejny klawisz
        key = keyboard.getKey()

        #Compute the roll, pitch, yaw and vertical inputs
        roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_disturbance
        pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = clamp(target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)

        vertical_input = K_VERTICAL_P * math.pow(clamped_difference_altitude, 3.0)

        # Actuate the motors taking into consideration all the computed inputs
        front_left_motor_input = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input
        # print(front_left_motor_input)
        # print(-front_right_motor_input)
        # print(rear_left_motor_input)
        # print(rear_right_motor_input)
        time.sleep(0.1)
        motors[0].setVelocity(front_left_motor_input)
        motors[1].setVelocity(-front_right_motor_input)  # Inverted to match the propeller's direction
        motors[2].setVelocity(-rear_left_motor_input)  # Inverted to match the propeller's direction
        motors[3].setVelocity(rear_right_motor_input)

    # Cleanup code (if any) would go here. In Webots, the cleanup is mostly handled by Webots itself.