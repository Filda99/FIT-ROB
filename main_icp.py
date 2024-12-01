#!/bin/python3
# coding: utf-8

from tp_icp.simulator import Simulator
from robot.robot import Robot
from robot.lidar import LiDAR
from robot.pose import Pose3D
from environment.seg_environment import SegEnv
from parameters.parameters import Parameters
import copy as copy

"""main.py: main file of the program
"""

import sys as sys

if sys.version_info[0] < 3 or sys.version_info[0] == 3 and sys.version_info[1] < 6:
    print("You need Python 3.6 or higher to run this script")
elif __name__ == "__main__":
    initial_pose = Pose3D(5, 5, -3.1419/2.0)
    robot = Robot()  # the simulated robot
    robot.init_robot(initial_pose)

    sensor = LiDAR()  # the simulated sensor of the robot
    sensor.init_sensor()

    environment = SegEnv()  # the simulated environment
    environment.init_environment_icp()

    # setting up all the parameters
    parameters = Parameters()

    setattr(parameters, "robot", robot)
    setattr(parameters, "sensor", sensor)
    setattr(parameters, "environment", environment)
    setattr(parameters, "initial_pose", copy.copy(initial_pose))

    # we start the simulator
    sim = Simulator(parameters)
else:
    print("This file should be executed as __main__")
