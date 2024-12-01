# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
""""
    This file contains function to implement
"""
from typing import List, Tuple  # for python3.8 compatibility

from math import atan2
from math import sqrt
from math import pi

from robot.robot import Robot
from tp_rrt.rrt_star import Node


def follow_path(robot:Robot, path:List[Node]) -> Tuple[List[Node], List[float]]:
    """ function that allows the robot to follow the path
        it returns a command u to reach the next node, and update the path by
            removing the nodes that have been reached by the robot
        PARAMETERS:
            robot: (robot.robot.Robot) the robot (to get the current position)
            path: (list of tp_rrt.rrt_star.Node) the path to follow
        RETURNS:
            updated path: (list of tp_rrt.rrt_star.Node)
            wheel command: (list of 2 numbers: [left wheel speed, right wheel speed]
    """
    dst_min = 0.05
    u = [0, 0]
    # TODO
    return path, u
