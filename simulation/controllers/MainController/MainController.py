#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 29 23:16:03 2020

@author: Angel Ayala <angel4ayala [at] gmail.com>
"""
import numpy as np
import pickle
from controller import Robot
from simple_pid import PID

import os
import sys
import struct, fcntl

from drone import Drone
from camera import DroneCamera
from webots_drone.utils import encode_image

from enum import Enum





class RESPONSE(Enum):
    REACHED = "REACHED"
    OK = "OK"



class COMMAND_TYPE(Enum):
    NONE = 0
    VELOCITY = 1
    DESTINATION = 2


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

# Drone Robot
class DroneController(Robot):
    """DroneController is the main class to manage the drone's action.

    This class manage the Emitter and Receiver nodes to send and get the states
    and actions of the drone to the Remote Control.
    This use the FlightControl to controlate the motors velocity of the drone.
    """
    ACCURACY = 0.1
    MAX_YAW_DISTURBANCE = 2
    MAX_PITCH_DISTURBANCE = 2
    # Precision between the target position and the robot position in meters
    target_precision = 0.1

    def __init__(self, read_from_path, write_to_path, camera_read_from_path, camera_write_to_path, command_type):
        super(DroneController, self).__init__()
        # local variables
        self.timestep = int(self.getBasicTimeStep())
        self.read_path = read_from_path
        self.write_path = write_to_path
        self.command_type = command_type
        self.last_set_velocities = np.array([0., 0., 0., 0.])
        self.start_position = [.0, .0, .0]
        self.target_position = 3 * [.0,]
        self.current_pose = 6 * [.0]

        #initialize fifo objects
        self.read_fifo = None
        self.write_fifo = None
        
        self.open_write()
        self.open_read()


        # Initialize Flight Control
        print('Initializing Drone Control...', end=' ')
        self.__drone = Drone(self)
        self.__camera = DroneCamera(self, read_from_path=camera_read_from_path, write_to_path=camera_write_to_path, fps=FPS, timestep=self.timestep, folder_path=IMAGE_FOLDER, if_camera_save=False)
        self.__time_delta = self.timestep / 1000
        self.__motor_controllers(self.__time_delta)

        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)

        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)

        #set destination point
        self.last_set_destination = 4*[.0]
        self.if_reached = False

        compass = self.getDevice('compass')
        compass.enable(self.timestep)

        print('OK')

    def __motor_controllers(self, time_delta=1):

        # Propeller PID control params tunned with Ziegler–Nichols PID
        K_u = 150.
        T_u = 342.857 / 1000.  # ms
        # no overshoot
        params_roll = {'P': K_u / 5.,
                       'I': (2. / 5.) * K_u / T_u,
                       'D': K_u * T_u / 15.,
                       'sp': 0.}
        self.roll = PID(params_roll['P'],
                        params_roll['I'],
                        params_roll['D'],
                        setpoint=params_roll['sp'],
                        output_limits=(-2., 2.), sample_time=time_delta)

        K_u = 150.
        T_u = 682.66 / 1000.  # ms
        # no overshoot
        params_pitch = {'P': K_u/5.,
                        'I': (2. / 5.) * K_u / T_u,
                        'D': K_u*T_u/15.,
                        'sp': 0.}
        self.pitch = PID(params_pitch['P'],
                         params_pitch['I'],
                         params_pitch['D'],
                         setpoint=params_pitch['sp'],
                         output_limits=(-2., 2.), sample_time=time_delta)
        K_u = 20.
        T_u = 1621.33 / 1000.  # ms
        # PD
        params_yaw = {'P': 0.8 * K_u,
                      'I': 0.,
                      'D': K_u * T_u / 10.,
                      'sp': 0}
        self.yaw = PID(params_yaw['P'],
                       params_yaw['I'],
                       params_yaw['D'],
                       setpoint=params_yaw['sp'],
                       output_limits=(-2., 2.), sample_time=time_delta)

        K_u = 20.
        T_u = 2668.8 / 1000.  # ms
        # PD
        params_vert = {'P': 0.8 * K_u,
                       'I': 0.,
                       'D': K_u * T_u / 10.,
                       'sp': 0.}
        self.vert = PID(params_vert['P'],
                        params_vert['I'],
                        params_vert['D'],
                        setpoint=params_vert['sp'],
                        output_limits=(-5., 5.), sample_time=time_delta)

    def __stabilize_pose(self, pose_angles, pose_vel):
        phi, theta, psi = pose_angles
        p, q, r = pose_vel
        roll = self.roll(phi, dt=self.__time_delta) - p
        pitch = self.pitch(theta, dt=self.__time_delta) - q
        yaw = self.yaw(r, dt=self.__time_delta)
        return [roll, pitch, yaw]

    def __compute_disturbances(self, disturbances):
        # current state
        orientation, ang_velocity, position, _, _ = self.__drone.get_odometry()
        # compute velocities to stabilize momentum
        self.roll.setpoint = disturbances[0]
        self.pitch.setpoint = disturbances[1]
        zero_momentum = self.__stabilize_pose(orientation, ang_velocity)
        if disturbances[2] != 0.:
            yaw_perturbance = disturbances[2]
        else:
            yaw_perturbance = zero_momentum[2]
        # check altitude
        _, _, curr_alt = position
        thrust_level = self.vert(curr_alt, dt=self.__time_delta)
        if disturbances[3] != 0:
            self.vert.setpoint = curr_alt
        # apply disturbances velocities
        pose_disturbance = [zero_momentum[0],
                            zero_momentum[1],
                            yaw_perturbance,
                            thrust_level + disturbances[3]]
        return pose_disturbance


    def open_read(self):
        self.read_fifo = open(self.read_path, 'rb')
        # Set the file descriptor to non-blocking mode
        fd = self.read_fifo.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    

    def close_read(self):
        self.read_fifo.close()


    def open_write(self):
        self.write_fifo = open(self.write_path, 'w')

    def close_write(self):
        self.write_fifo.close()


    def read_command(self):
        length = 4
        if self.command_type==COMMAND_TYPE.DESTINATION:
            length=3
        data_format =length*'f'
        try:
            data = self.read_fifo.read(length*4)
            if data:
                unpacked = struct.unpack(data_format, data)
                print(f'Received: { unpacked }')
                self.send(RESPONSE.OK.value)
                return np.array(unpacked)
            else:
                # No data available at the moment
                # print('Pipe is empty.\nWaiting...')
                pass
        except BlockingIOError:
            # No data was available for reading
            pass
        
        return None

    def read_velocities(self):
        command = self.read_command()

        if self.command_type == COMMAND_TYPE.DESTINATION:
            if isinstance(command, np.ndarray):
                self.last_set_destination[0:3] = self.start_position + command
                self.if_reached=False
                print('command: ', command)
                # print('move to target: ', self.move_to_target())
            
            
            if self.if_reached:
                return [.0, .0, .0, .0]

            # This will be in ]-pi;pi]
            target_angle = np.arctan2(
                self.last_set_destination[1] - self.current_pose[1], self.last_set_destination[0] - self.current_pose[0])
            # This is now in ]-2pi;2pi[
            angle_left =  target_angle - self.current_pose[5]

            angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
            if (angle_left > np.pi):
                angle_left -= 2 * np.pi
                # print('al ', angle_left)


            rotation_matrix = np.array([[np.cos(angle_left), np.sin(angle_left), 0], [-np.sin(angle_left), np.cos(angle_left), 0], [0, 0, 1]], dtype=float)
            # print('rm ', rotation_matrix)
            # print(angle_left)
            # print('direction_vector ', direction_vector)

            direction_vector = np.array(self.last_set_destination[0:3]) - np.array(self.current_pose[0:3])

            direction_vector =  np.matmul(direction_vector, rotation_matrix)
            
            # print('direction_vector ', direction_vector)
            
            direction_vector = np.insert(direction_vector, 2, angle_left)
            
            velocities = np.array([.0, .0, .0, .0], dtype=float)

            for i in range(len(direction_vector)):
                if abs(direction_vector[i]) > self.ACCURACY:
                    velocities[i] = direction_vector[i]
                else:
                    velocities[i] = .0
            
            if(velocities[2] != 0 or velocities[3] != 0):
                velocities[0] = 0.
                velocities[1] = 0. 
                velocities[2] = np.sign(velocities[2]) * self.MAX_YAW_DISTURBANCE
                velocities[3] = np.sign(velocities[3]) * 4
            else:
                max_velo = np.max(abs(velocities[0:2]))
                if(max_velo != 0):
                    velocities[0] = 0. #velocities[1] / max_velo * 0.1
                    velocities[1] =  clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)# velocities[0] / max_velo * 0.1
                    velocities[2] = 0.
                    velocities[3] = 0.
            
            if not np.any(velocities):
                self.if_reached = True
                self.send(RESPONSE.REACHED.value)

            
            return velocities

        elif self.command_type == COMMAND_TYPE.VELOCITY:
            return command


    def send(self, message):
        self.write_fifo.write(message + '\n')
        self.write_fifo.flush()
        print(message)



    def __compute_velocity(self):

        velocities = self.read_velocities() # velocities in roll, pitch, yaw, z
        
        if isinstance(velocities, np.ndarray):
            self.last_set_velocities = velocities

        
        # apply disturbances velocities
        pose_disturbance = self.__compute_disturbances(self.last_set_velocities)
        roll_d, pitch_d, yaw_d, thrust_d = pose_disturbance

        fl_motor = thrust_d + roll_d - pitch_d - yaw_d  # front L
        fr_motor = thrust_d - roll_d - pitch_d + yaw_d  # front R
        rl_motor = thrust_d + roll_d + pitch_d + yaw_d  # rear L
        rr_motor = thrust_d - roll_d + pitch_d - yaw_d  # rear R

        return fl_motor, fr_motor, rl_motor, rr_motor

    def update_postition(self):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        x_pos, y_pos, altitude = self.gps.getValues()
        self.current_pose = [x_pos, y_pos, altitude, roll, pitch, yaw]

    def run(self):
        """Run controller's main loop.

        Send the variations of the altitude, the roll, pitch, and yaw angles
        to the drone. Send the current image captured by the drone's
        camera and get the actions from the Remote Control, once the action
        (variations of the angles an altitude) is received, the Drone
        calculates the velocity, and apply it to the 3 different angles and
        altitude.
        """
        # control loop
        print('Drone control is active')
        #init step
        self.step(self.timestep)
        self.start_position = np.array(self.gps.getValues())
        print('Start position: ', self.start_position)
        iter = 0
        while self.step(self.timestep) != -1:
            if(iter == 1000//self.timestep):
                iter=0
                print("gps: ", np.array(self.gps.getValues()) - self.start_position)
            # actuates over devices and motors
            self.update_postition()
            propellers_vel = self.__compute_velocity()
            self.__drone.set_motors_velocity(*propellers_vel)
            self.__drone.blink_leds()
            self.__camera.run()
            iter+=1
            # comms
            # self.__send_state()


#SETTINGS
FPS = 1
IMAGE_FOLDER = "./images/"

FIFO_CONTROLLER_READER_PATH = "./pipes/main_to_controller"
FIFO_CONTROLLER_WRITER_PATH = "./pipes/controller_to_main"

FIFO_CAMERA_READER_PATH = "./pipes/main_to_camera"
FIFO_CAMERA_WRITER_PATH = "./pipes/camera_to_main"

TYPE_OF_COMMAND = COMMAND_TYPE.DESTINATION

"""Usage of pipes.

DESTINATION TYPE
set destination, write to pipe flaot vector [x, y, z], wait for reciving communicate from drone OK. After reaching destination drone will response with REACHED.

VELOCITY TYPE
set velocity in 4 axis, write to pipe float vector [y_velocity, x_velocity, rotatation_velocity, altitude_velocity]. After geting communicate, drone will response with OK.
drone will
"""

if __name__ == '__main__':
    # run controller

    controller = DroneController(read_from_path=FIFO_CONTROLLER_READER_PATH,
        write_to_path=FIFO_CONTROLLER_WRITER_PATH,
        camera_read_from_path=FIFO_CAMERA_READER_PATH,
        camera_write_to_path=FIFO_CAMERA_WRITER_PATH,
        command_type = TYPE_OF_COMMAND
        )

    controller.run()

    del controller
