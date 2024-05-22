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




# Drone Robot
class DroneController(Robot):
    """DroneController is the main class to manage the drone's action.

    This class manage the Emitter and Receiver nodes to send and get the states
    and actions of the drone to the Remote Control.
    This use the FlightControl to controlate the motors velocity of the drone.
    """

    def __init__(self, fifo_from_main_path, fifo_to_main_path):
        super(DroneController, self).__init__()
        # local variables
        self.timestep = int(self.getBasicTimeStep())
        self.fifo_read_path = fifo_from_main_path
        self.last_set_velocities = [0., 0., 0., 0.]
        
        self.read_fifo = None
        self.open_read_fifo()

        # Initialize Flight Control
        print('Initializing Drone Control...', end=' ')
        self.__drone = Drone(self)
        self.__camera = DroneCamera(self, fifo_to_main_path, fps=1, timestep=self.timestep, folder_path="./images/", if_camera_save=False)
        self.__time_delta = self.timestep / 1000
        self.__motor_controllers(self.__time_delta)

        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)

        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)
        self.start_position = self.gps.getValues()
        self.last_set_destination = self.start_position


        compass = self.getDevice('compass')
        compass.enable(self.timestep)

        # Initialize comms
        # self.state = self.getDevice('StateEmitter')  # channel 4
        # self.action = self.getDevice('ActionReceiver')  # channel 6
        # self.action.enable(self.timestep)
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

    def open_read_fifo(self):
        self.read_fifo = open(self.fifo_read_path, 'r', buffering=1)
        # Set the file descriptor to non-blocking mode
        fd = self.read_fifo.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    

    def close_read_fifo(self):
        self.read_fifo.close()


    def read_from_fifo(self):

        try:
            data = self.read_fifo.readline()
            if data:
                unpacked = struct.unpack('ffff', eval(data))
                print(f'Received: { unpacked }')
                return unpacked
            else:
                # No data available at the moment
                print('Pipe is empty.\nWaiting...')
        except BlockingIOError:
            # No data was available for reading
            pass
        
        return None



    def __compute_velocity(self):
        command = self.read_from_fifo() # velocities in roll, pitch, yaw, z

        if command:
            self.last_set_velocities = command

        velocities = self.last_set_velocities

        # apply disturbances velocities
        pose_disturbance = self.__compute_disturbances(velocities)
        roll_d, pitch_d, yaw_d, thrust_d = pose_disturbance

        fl_motor = thrust_d + roll_d - pitch_d - yaw_d  # front L
        fr_motor = thrust_d - roll_d - pitch_d + yaw_d  # front R
        rl_motor = thrust_d + roll_d + pitch_d + yaw_d  # rear L
        rr_motor = thrust_d - roll_d + pitch_d - yaw_d  # rear R

        return fl_motor, fr_motor, rl_motor, rr_motor

    def __send_state(self):
        # get current state
        uav_orientation, uav_angular_velocity,\
            uav_position, uav_speed, uav_north_rad =\
            self.__drone.get_odometry()
        uav_distance_sensors = self.__drone.get_dist_sensors()
        uav_image = self.__drone.get_image()

        motors_vel = [m.getVelocity() for m in self.__drone.motors]
        # encode data
        msg_data = dict(timestamp=np.round(self.getTime(), 3),
                        orientation=uav_orientation,
                        angular_velocity=uav_angular_velocity,
                        position=uav_position,
                        speed=uav_speed,
                        north=uav_north_rad,
                        dist_sensors=uav_distance_sensors,
                        motors_vel=motors_vel)
        enc_img = "NoImage" if uav_image is None else encode_image(uav_image)
        msg_data['image'] = enc_img
        # send data
        # emitter_send_json(self.state, msg_data)


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
        while self.step(self.timestep) != -1:
            print("gps: ", self.gps.getValues())
            # actuates over devices and motors
            propellers_vel = self.__compute_velocity()
            self.__drone.set_motors_velocity(*propellers_vel)
            self.__drone.blink_leds()
            self.__camera.run()
            # comms
            # self.__send_state()


FIFO_CONTROLLER_READER_PATH = "./pipes/main_to_controller"
FIFO_CAMERA_WRITER_PATH = "./pipes/camera_to_main"

if __name__ == '__main__':
    # run controller

    main_to_drone_pipe, drone_to_main_pipe = os.pipe()
    main_to_camera_pipe, camera_to_main_pipe = os.pipe()

    controller = DroneController(fifo_from_main_path=FIFO_CONTROLLER_READER_PATH, fifo_to_main_path=FIFO_CAMERA_WRITER_PATH)

    controller.run()

    del controller
