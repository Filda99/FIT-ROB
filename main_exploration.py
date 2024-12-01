#!/bin/python3
# coding: utf-8
__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the main function for the exploration workshop program
"""

from tp_exploration.simulator import Simulator
from tp_exploration.exploration_map import ExplorationGridMap
from robot.robot import Robot
from robot.lidar import LiDAR
from robot.pose import Pose3D
from environment.seg_environment import SegEnv
from parameters.parameters import Parameters

import sys as sys

if sys.version_info[0] < 3 or sys.version_info[0] == 3 and sys.version_info[1] < 6:
    print("You need Python 3.6 or higher to run this script")
elif __name__ == "__main__":
    robot = Robot()  # the simulated robot
    robot.init_robot(Pose3D(2.01, 2.01, 0))

    sensor = LiDAR()  # the simulated sensor of the robot
    sensor.init_sensor()

    environment = SegEnv()  # the simulated environment
    environment.init_environment()

    exploration_map = ExplorationGridMap()  # the known grid map of the environment
    exploration_map.init_map()

    # setting up all the parameters
    parameters = Parameters()

    setattr(parameters, "robot", robot)
    setattr(parameters, "sensor", sensor)
    setattr(parameters, "environment", environment)
    setattr(parameters, "map", exploration_map)
    setattr(parameters, "fps", 20)
    setattr(parameters, "rk_step", 10)

    # we start the simulator
    sim = Simulator(parameters)
else:
    print("This file should be executed as __main__")
