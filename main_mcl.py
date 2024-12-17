#!/bin/python3
# coding: utf-8
__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the main function for the MCL workshop program
"""

from environment.seg_environment import SegEnv
from environment.grid_map import GridMap
from parameters.parameters import Parameters
from robot.lidar import LiDAR
from tp_mcl.monte_carlo import Robot
from robot.pose import Pose3D
from tp_mcl.cost_map import CostMap
# from tp_mcl.monte_carlo import MonteCarloLocalization
from tp_mcl.simulator import Simulator

import sys as sys

if sys.version_info[0] < 3 or sys.version_info[0] == 3 and sys.version_info[1] < 6:
    print("You need Python 3.6 or higher to run this script")
elif __name__ == "__main__":
    robot = Robot(Pose3D(2.01, 2.01, 0))  # main robot

    # environment = SegEnv()  # the simulated environment
    # environment.init_environment()
    landmarks = [[], []]  # TODO(filip): define landmarks

    grid_map = GridMap()  # the known grid map of the environment
    grid_map.init_map()
    grid_map.compute_map(environment)

    # mcl = MonteCarloLocalization()  # the Monte Carlo Localization Algorithm

    parameters = Parameters()

    setattr(parameters, "robot", robot)  # TODO(filip): fix
    setattr(parameters, "environment", environment)
    setattr(parameters, "map", grid_map)
    setattr(parameters, "number_of_particles", 300)
    setattr(parameters, "percent_random_particles", 10)
    setattr(parameters, "fps", 20)
    setattr(parameters, "rk_step", 10)
    # setattr(parameters, "mcl", mcl)

    # we start the simulator
    sim = Simulator(parameters)
else:
    print("This file should be executed as __main__")
