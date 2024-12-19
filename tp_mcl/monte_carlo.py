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
    """
        Class to handle noise parameters for the robot's movements.
        Attributes:
            forward_noise (float): The noise in the forward movement.
            turn_noise (float): The noise in the turn movement.
            sense_noise (float): The noise in the sensing measurements.
    """
    turn_noise: float
    forward_noise: float
    sense_noise: float

    def __init__(self, noise: tuple[float, float, float]) -> None:
        """
            Constructor initializes the noise parameters for the robot's movements.
            Parameters:
                noise (tuple[float, float, float]): A tuple containing the forward noise, turn noise, and sense noise.
        """
        
        self.forward_noise = noise[0]
        self.turn_noise = noise[1]
        self.sense_noise = noise[2]


class Robot:
    """
        Class to handle the particle of the Monte Carlo localization algorithm.
        Attributes:
            pose (Pose3D): The pose of the particle (x, y, theta).
            weight (float): The weight of the particle.
            robot_width (float): The width of the robot.
    """
    pose: Pose3D
    weight: float
    robot_width = 2

    def __init__(self, pose: Pose3D|None = None, weight: float = 0.0, noise: tuple[float, float, float] = (0.2, 0.05, 2.0)):
        """ 
            Constructor initializes the robot's pose, weight, and noise parameters.
            Parameters:
                pose (Pose3D|None): The pose of the robot (x, y, theta). Defaults to None.
                weight (float): The weight of the robot. Defaults to 0.0.
                noise (tuple[float, float, float]): A tuple containing the forward noise, turn noise, and sense noise. Defaults to (0.2, 0.05, 2.0).
        """
        if pose is None:
            pose = Pose3D()
        self.pose = copy.copy(pose)
        self.weight = weight
        self.noise = Noise(noise)


    def set_pose(self, pose: Pose3D):
        """
        Method sets the pose of the robot to the given pose.
        Parameters:
            pose (Pose3D): The new pose of the robot (x, y, theta).
        """
        self.pose = pose


    def move(self, forward: float , turn: float):
        """
        Method moves the robot based on the given forward and turn values, incorporating noise.
        Parameters:
            forward (float): The forward movement distance.
            turn (float): The turn angle.
        Raises:
            Exception: If the forward movement is negative.
        """
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
        """
            Method returns the distances from the robot to each landmark, incorporating sensing noise.
            Parameters:
                landmarks (list[Point2D]): A list of landmarks represented by their coordinates.
            Returns:
                list[float]: A list of distances from the robot to each landmark.
        """
        pos = Point2D(self.pose.x, self.pose.y)
        return [Point2D.distance(pos, l) + gauss(0.0, self.noise.sense_noise) for l in landmarks]


    def get_measurement_prob(self, measurements: list[float], landmarks: list[Point2D]) -> float:
        """
            Method calculates the probability of the given measurements based on the landmarks and the robot's sensing noise.
            Parameters:
                measurements (list[float]): A list of observed measurements.
                landmarks (list[Point2D]): A list of landmarks represented by their coordinates.
            Returns:
                float: The calculated measurement probability.
        """
        pos = Point2D(self.pose.x, self.pose.y)
        probs = [norm.pdf(x=measurements[i], loc=Point2D.distance(pos, l), scale=self.noise.sense_noise) for i,l in enumerate(landmarks)]
        w: float = reduce(lambda x, y: x*y, probs, 1.0) # type: ignore
        self.weight = w
        return w