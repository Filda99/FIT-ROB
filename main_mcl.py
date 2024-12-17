#!/bin/python3
# coding: utf-8
__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the main function for the MCL workshop program
"""

from parameters.parameters import Parameters
from tp_mcl.monte_carlo import Robot
from robot.pose import Pose3D
from tp_mcl.simulator import Simulator


if __name__ == "__main__":
    # robot = Robot(Pose3D(15, 30, 0))  # main robot
    robot = Robot(Pose3D(40, 40, 0))  # main robot

    # environment = SegEnv()  # the simulated environment
    # environment.init_environment()

    # grid_map = GridMap()  # the known grid map of the environment
    # grid_map.init_map()
    # grid_map.compute_map(environment)

    # mcl = MonteCarloLocalization()  # the Monte Carlo Localization Algorithm

    parameters = Parameters()

    setattr(parameters, "robot", robot)  # TODO(filip): fix
    # setattr(parameters, "environment", environment)
    # setattr(parameters, "map", grid_map)
    setattr(parameters, "number_of_particles", 1000)
    setattr(parameters, "percent_random_particles", 10)
    setattr(parameters, "fps", 20)
    setattr(parameters, "rk_step", 10)
    # setattr(parameters, "mcl", mcl)

    sim = Simulator(parameters)
