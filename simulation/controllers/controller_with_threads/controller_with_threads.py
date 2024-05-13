import threading

from controller import Robot, Motor, Camera, GPS, InertialUnit, Gyro, LED, Keyboard, Compass
import math
import numpy as np
import time
import os
from PIL import Image

from Drone import Drone

IMAGES_PATH = "images_from_camers"
TIMESTEP = 64


if __name__ == "__main__":
    drone = Drone(TIMESTEP)

    while drone.step(TIMESTEP) != -1:
        drone.move()
