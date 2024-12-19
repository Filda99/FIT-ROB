"""
    Project: ROBa project
    File: main_mcl.py
    Description: This file provides the main function for the software project.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
    NOTE: Some parts of the code was borowed from the following source: https://perso-laris.univ-angers.fr/~r.guyonneau/students/robotique_mobile/
"""

from environment.grid_map import GridMap
from parameters.parameters import Parameters
from mcl.monte_carlo import Robot
from mcl.pose import Pose3D
from mcl.simulator import Simulator

if __name__ == "__main__":
    robot = Robot(Pose3D(40, 40, 0))  # main robot
    predicted_robot = Robot(Pose3D(40, 40, 0))  # predicted robot

    grid_map = GridMap()  # the known grid map of the environment
    grid_map.init_map()

    parameters = Parameters()

    setattr(parameters, "robot", robot)
    setattr(parameters, "predicted_robot", predicted_robot)
    setattr(parameters, "map", grid_map)
    setattr(parameters, "number_of_particles", 1000)
    setattr(parameters, "percent_random_particles", 10)
    setattr(parameters, "fps", 20)
    setattr(parameters, "rk_step", 10)

    sim = Simulator(parameters)
