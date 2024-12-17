# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a Particle class and a MonteCarloLocalization Class
"""

from math import cos
from math import pi
from math import sin
import math
from random import gauss
from random import uniform
from scipy.stats import norm

from typing import List  # for python 3.8 compatibility

from tp_mcl.pose import Pose3D
import copy as copy

class Robot:
    """ class to handle the particle of the monte carlo localization algorithm
    ATTRIBUTES:
        self.pose: (robot.pose.Pose3D)  pose of the particle (x, z, theta)
        self.weight: (number) weight of the particle
    """
    pose: Pose3D
    weight: float
    robot_width = 0.3

    turn_noise: float
    forward_noise: float

    def __init__(self, pose: Pose3D|None = None, weight: float = 0.0):
        """ constructor of the class
            :param pose: (robot.pose.Pose3D)  pose of the particle (x, z, theta)
            :param weight: (number) weight of the particle
        """
        if pose is None:
            pose = Pose3D()
        self.pose = copy.copy(pose)  # (robot.pose.Pose3D)  pose of the particle (x, z, theta) in meter
        self.weight = weight         # (number)   weight of the particle
        self.turn_noise = 0.05
        self.forward_noise = 0.05

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

    def move(self, forward: float , turn: float ):
        if forward < 0:
            raise Exception("can't move backwards")

        if (turn != 0):
            self.pose.theta = self.pose.theta + turn + gauss(0.0, self.turn_noise)
            self.pose.theta = self.pose.theta % (2 * math.pi)

        dist = 0.0
        if (forward > 0):
            dist = forward + gauss(0.0, self.forward_noise)
        
        self.pose.x = self.pose.x + (cos(self.pose.theta) * dist)
        self.pose.y = self.pose.y + (sin(self.pose.theta) * dist)

        self.pose.x = self.pose.x % 800
        self.pose.y = self.pose.y % 800


# class MonteCarloLocalization:{{{
#     """ class to handle the Monte Carlo Localization
#     ATTRIBUTES:
#         self.particles: (list of Particle) the particles
#         self.nb_particles: (int) number of particles
#         self.max_weight: (number) current maximal weight of a particle
#         self.id_best_particle: (int) the index of the best particle
#     """
#     particles: List[Particle]
#     nb_particles: int
#     max_weight: float
#     id_best_particle: int
#
#
#     def __init__(self):
#         """ constructor of the class
#         """
#         self.particles = []  # List of the particles
#         self.nb_particles = 0  # number of particles
#         self.max_weight = 0.0  # current maximal weight of a particle
#         self.id_best_particle = None  # the current best particle (should be an index in the particles list)
#
#     def init_particles(self, cost_map:CostMap, number_of_particles:int):
#         """ function that initialises the particles
#             the cost map is needed because we do not want to put particles in
#             a non obstacle free cell
#             the particles are uniformly spread over the map
#             :param cost_map: (environment.cost_map.CostMap) the cost map
#             :param number_of_particles: (int) the number of particles
#         """
#         self.nb_particles = number_of_particles
#         self.particles = []
#         self.max_weight = 0
#         # TODO
#
#     def __str__(self):
#         """
#         To display a MonteCarloLocalization as a string (debug propose)
#         """
#         msg = ""
#         for p in self.particles:
#             msg += str(p) + "\n"
#         return msg
#
#     def evaluate_particles(self, cost_map:CostMap, measurements:List[LiDARMeasurement]):
#         """ function that update the particles weight according to the measurements
#             and the cost map
#             note that the max weight and the best particle need to be computed and
#             the weight needs to be normalized (between 0 and 1)
#             :param cost_map: (environment.cost_map.CostMap) the cost map
#             :param measurements: (list of robot.lidar.LiDARMeasurement) the measurements
#         """
#         # TODO
#         pass
#
#     def re_sampling(self, sigma_xz:float=0.05, sigma_theta:float=1*pi/180):
#         """ function that re-sample the particles around the best ones
#             note: use a gaussian distribution around the best particles with
#             sigma for position : 0.05
#             sigma for orientation : 1 deg
#         """
#         # TODO
#         pass
#
#     def estimate_from_odometry(self, odo_delta_dst:float, odo_delta_theta:float):
#         """ function that update the position and orientation of all the particles
#             according to the odometry data
#             PARAMETERS:
#                 odo_delta_dst: (number) the distance delta
#                 odo_delta_theta: (number) the orientation delta
#         """
#         # TODO
#         pass
#
#     def add_random_particles(self, cost_map:CostMap, percent:float):
#         """ function that modifies "percent" percent of particles by initializing them
#             randomly
#             percent is a value between 0 and 100
#             PARAMETERS:
#                 cost_map: (CostMap) the cost map
#                 percent: (int) the percent (from 0 to 100)
#         """
#         # TODO
#         pass}}}
