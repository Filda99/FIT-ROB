# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
""""
    This file contains function to implement
"""

from math import atan2
from math import pi
from geometry.point import Point2D


def set_target(robot, exploration_map):
    """ function that provides the next target for the robot, according to its position and the exploration map
        it returns a command u to reach the next node, and update the path by
            removing the nodes that have been reached by the robot
    PARAMETERS:
        robot: (robot.robot.Robot)
        exploration_map: (tp_exploration.exploration_map.ExplorationGridMap)
    RETURN:
        the next target for the robot (geometry.point.Point2D)
    """
    target = None
    # TODO
    return target


def go_2_target(robot, target):
    """ function that allows the robot to go to the target point
        it returns a command u to reach the target
        PARAMETERS:
            robot: (robot.robot.Robot) the robot (to get the current position)
            target: (geometry.point.Point2D) the target point to reach
        RETURNS:
            wheel command: (list of 2 numbers: [left wheel speed, right wheel speed]
    """
    linear_speed = 10
    angular_speed = 5
    # TODO
    return [0, 0]
