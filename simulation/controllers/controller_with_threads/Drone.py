from controller import Robot, Motor, Camera, GPS, InertialUnit, Gyro, LED, Keyboard, Compass
import time, math

# Helper functions
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def sign(x):
    return (x > 0) - (x < 0)


class Drone(Robot):
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts
    K_VERTICAL_OFFSET = 0.6   # Vertical offset where the robot actually targets to stabilize itself
    K_VERTICAL_P = 3.0        # P constant of the vertical PID
    K_ROLL_P = 50.0           # P constant of the roll PID
    K_PITCH_P = 30.0          # P constant of the pitch PID


    def __init__(self, timestep=None):
        super().__init__()
        if not timestep:
            self.timestep = self.getBasicTimeStep()
        else:
            self.timestep = timestep

        #setting up camera
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timestep)

        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)

        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)

        self.compass = self.getDevice('compass')
        self.compass.enable(self.timestep)

        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.timestep)

        self.camera_roll_motor = self.getDevice('camera roll')
        self.camera_pitch_motor = self.getDevice('camera pitch')

        self.camera_yaw = self.getDevice('camera yaw')
        self.camera_pitch = self.getDevice('camera pitch')
        self.camera_roll = self.getDevice('camera roll')
        self.yaw_sensor = self.getDevice('camera yaw sensor')
        self.pitch_sensor = self.getDevice('camera pitch sensor')
        self.roll_sensor = self.getDevice('camera roll sensor')

        self.yaw_sensor.enable(self.timestep)
        self.pitch_sensor.enable(self.timestep)
        self.roll_sensor.enable(self.timestep)

        self.motors = []
        for name in ['front left propeller', 'front right propeller', 'rear left propeller', 'rear right propeller']:
            motor = self.getDevice(name)
            motor.setPosition(float('+inf'))
            motor.setVelocity(1.0)
            self.motors.append(motor)

        # Variables
        self.target_altitude = 0.5  # The target altitude. Can be changed by the user
        self.sampling_interval = 10
        self.last_sampling_time = time.time()

        self.current_pitch = 0
        self.current_roll = 0

        self.pitch_disturbance = 0.0
        self.roll_disturbance = 0.0
        self.yaw_disturbance = 0.0


    def _go_up(self):
        self.target_altitude -= 0.05

    def _go_down(self):
        self.target_altitude += 0.05

    def _go_rear(self):
        pass

    def _go_back(self):
        pass

    def move(self):


        roll, pitch, yaw = self.imu.getRollPitchYaw()
        print(roll, pitch, yaw)

        altitude = self.gps.getValues()[2]
        roll_velocity, pitch_velocity, _ = self.gyro.getValues()

        #Compute the roll, pitch, yaw and vertical inputs
        roll_input = Drone.K_ROLL_P * clamp(roll, -1.0, 1.0) + self.roll_disturbance
        pitch_input = Drone.K_PITCH_P * clamp(pitch, -1.0, 1.0) + self.pitch_disturbance
        yaw_input = self.yaw_disturbance
        clamped_difference_altitude = clamp(self.target_altitude - altitude + Drone.K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = Drone.K_VERTICAL_P * math.pow(clamped_difference_altitude, 3.0)


        # Actuate the motors taking into consideration all the computed inputs
        front_left_motor_input = Drone.K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = Drone.K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = Drone.K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = Drone.K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

        # print('new')
        # print(front_left_motor_input)
        # print(-front_right_motor_input)
        # print(rear_left_motor_input)
        # print(rear_right_motor_input)
        time.sleep(0.1)
        self.motors[0].setVelocity(front_left_motor_input)
        self.motors[1].setVelocity(-front_right_motor_input)  # Inverted to match the propeller's direction
        self.motors[2].setVelocity(-rear_left_motor_input)  # Inverted to match the propeller's direction
        self.motors[3].setVelocity(rear_right_motor_input)