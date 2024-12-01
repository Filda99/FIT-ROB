#!/bin/python3
# coding: utf-8
__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the main function for the RRT workshop program
"""

from tp_rrt.simulator import Simulator
from tp_rrt.rrt_star import RRTStar
from robot.robot import Robot
from robot.pose import Pose3D
from environment.seg_environment import SegEnv
from environment.seg_map import SegMap
from parameters.parameters import Parameters

import sys as sys

if sys.version_info[0] < 3 or sys.version_info[0] == 3 and sys.version_info[1] < 6:
    print("You need Python 3.6 or higher to run this script")
elif __name__ == "__main__":
    robot = Robot()  # the simulated robot
    robot.init_robot(Pose3D(2.01, 2.01, 0))

    environment = SegEnv()  # the simulated environment
    environment.init_environment()

    seg_map = SegMap()  # the known map of the environment (list of segments)
    seg_map.init_map()
    seg_map.compute_map(environment)  # this does not actually compute the map because the map is hard-coded

    rrt_star = RRTStar()

    # setting up all the parameters
    parameters = Parameters()
    setattr(parameters, "robot", robot)
    setattr(parameters, "environment", environment)
    setattr(parameters, "map", seg_map)
    setattr(parameters, "rrt_star", rrt_star)
    setattr(parameters, "fps", 20)
    setattr(parameters, "rk_step", 10)

    # start the simulator
    sim = Simulator(parameters)
else:
    print("This file should be executed as __main__")
