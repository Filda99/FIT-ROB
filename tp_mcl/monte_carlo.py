"""
    Project: ROBa project
    File: monte_carlo.py
    Description: This file contains the implementation of the Monte Carlo localization algorithm for the robotics project.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""

from functools import reduce
from math import cos
from math import pi
from math import sin
import math
from random import gauss
from random import uniform
from scipy.stats import norm

from geometry.point import Point2D
from tp_mcl.pose import Pose3D
import copy as copy

from .global_vars import WORLD_SIZE


class Noise:
    turn_noise: float
    forward_noise: float
    sense_noise: float

    def __init__(self, noise: tuple[float, float, float]) -> None:
        self.forward_noise = noise[0]
        self.turn_noise = noise[1]
        self.sense_noise = noise[2]


class Robot:
    """ class to handle the particle of the monte carlo localization algorithm
    Attributes:
        self.pose: (robot.pose.Pose3D)  pose of the particle (x, z, theta)
        self.weight: (number) weight of the particle
    """
    pose: Pose3D
    weight: float
    robot_width = 2

    def __init__(self, pose: Pose3D|None = None, weight: float = 0.0, noise: tuple[float, float, float] = (0.2, 0.05, 2.0)):
        """ constructor of the class
            :param pose: (robot.pose.Pose3D)  pose of the particle (x, z, theta)
            :param weight: (number) weight of the particle
        """
        if pose is None:
            pose = Pose3D()
        self.pose = copy.copy(pose)
        self.weight = weight
        self.noise = Noise(noise)

    def set_pose(self, pose: Pose3D):
        self.pose = pose

    def __str__(self):
        """ to display a Particle as a string"""
        return f"{self.pose} : {self.weight}"

    def __eq__(self, other):
        """
        Overwrite the == operator
        :param other: (Particle) the Particle to be compared with
        """
        if other is None:
            return False
        else:
            return self.pose == other.pose

    def __lt__(self, other):
        return self.weight < other.weight

    def __le__(self, other):
        return self.weight <= other.weight

    def __gt__(self, other):
        return self.weight > other.weight

    def __ge__(self, other):
        return self.weight >= other.weight

    def move(self, forward: float , turn: float):
        if forward < 0:
            raise Exception("can't move backwards")

        if (turn != 0):
            self.pose.theta = self.pose.theta + turn + gauss(0.0, self.noise.turn_noise)
            self.pose.theta = self.pose.theta % (2 * math.pi)

        dist = 0.0
        if (forward > 0):
            dist = forward + gauss(0.0, self.noise.forward_noise)
        self.pose.x = self.pose.x + (cos(self.pose.theta) * dist)
        self.pose.y = self.pose.y + (sin(self.pose.theta) * dist)

        self.pose.x = self.pose.x % WORLD_SIZE[0]
        self.pose.y = self.pose.y % WORLD_SIZE[1]

    def get_measurements(self, landmarks: list[Point2D]) -> list[float]:
        pos = Point2D(self.pose.x, self.pose.y)
        return [Point2D.distance(pos, l) + gauss(0.0, self.noise.sense_noise) for l in landmarks]

    def get_measurement_prob(self, measurements: list[float], landmarks: list[Point2D]) -> float:
        pos = Point2D(self.pose.x, self.pose.y)
        probs = [norm.pdf(x=measurements[i], loc=Point2D.distance(pos, l), scale=self.noise.sense_noise) for i,l in enumerate(landmarks)]
        w: float = reduce(lambda x, y: x*y, probs, 1.0) # type: ignore
        self.weight = w
        return w

