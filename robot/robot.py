# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a Robot class to simulate a two wheeled mobile robot dynamic.
"""

from math import cos
from math import sin
from robot.pose import Pose3D


class Robot:
    """ A class to simulate a two wheeled mobile robot
        the input corresponds to the angular wheel speed (rad/s)
        ATTRIBUTES:
            self.wheel_radius: (number in m) radius of the left wheel
            self.wheel_distance: (number in m) distance between the left and the right wheel
            self.pose: (robot.pose.Pose3D) the pose of the robot (position and orientation)
            self.d_width_wheel: (number in m) width of the wheels
    """
    wheel_radius: float
    wheel_distance: float
    pose: Pose3D
    d_width_wheel: float

    def __init__(self):
        """ Constructor of the class
        """
        # variables for the model
        self.wheel_radius = None  # m  radius of the left wheel
        self.wheel_distance = None  # m  distance between the left and the right wheel

        self.pose = Pose3D()

        # variable for the display
        self.d_width_wheel = None  # m  width of the wheels

    def __str__(self):
        """ to be able to print a Robot
        """
        return f"{self.pose}"

    def init_robot(self, pose):
        """ Function to initialize the robot parameters
        PARAMETERS:
            pose: (robot.pose.Pose3D) the pose for the robot (position and orientation)
        """
        # variables for the model
        self.wheel_radius = 0.1  # m  radius of the left wheel
        self.wheel_distance = 0.2  # m  distance between the left and the right wheel

        self.pose = pose  # pose of the robot (x, z, theta)

        # variable for the display
        self.d_width_wheel = 0.05  # m  width of the wheels

    def f(self, _unused_x, _unused_z, theta, delta_t, u):
        """ Differential equation that models the robot
        PARAMETERS:
            _unused_x: (number in m) the x position (not used)
            _unused_z: (number in m) the z position (not used)
            theta: (number in rad) the orientation
            delta_t: (number in s) the step for the discreet time
            u: (list of two number in rad/s) the wheel speed commands [left, right]
        RETURNS:
            the x' value (derivative of the x position according to the differential equation) (number)
            the z' value (derivative of the z position according to the differential equation) (number)
            the theta' value (derivative of the orientation according to the differential equation) (number)
        """

        # variable to ease the reading
        r = self.wheel_radius  # m      radius of the left wheel
        dst = self.wheel_distance  # m      distance between the left and the right wheel
        ul = u[0]  # rad/s  angular speed of the left wheel
        ur = u[1]  # rad/s  angular speed of the right wheel

        xp = (r / 2.0) * (ul + ur) * cos(theta)
        zp = (r / 2.0) * (ul + ur) * sin(theta)
        theta_p = (r / dst) * (ul - ur)

        return xp * delta_t, zp * delta_t, theta_p * delta_t

    def runge_kutta(self, delta_t, u):
        """ Function that implements the runge kutta approach for estimating differential equation
        it updates the x, z and theta attribute of the object
        PARAMETERS:
            delta_t: (number in s) time step for the runge kutta
            u: (list of two number in rad/s) the wheel speed commands [left, right]
        """
        # to estimate the derivative equation
        k1xp, k1zp, k1_theta_p = self.f(self.pose.x, self.pose.z, self.pose.theta, delta_t, u)
        k2xp, k2zp, k2_theta_p = self.f(self.pose.x + k1xp / 2, self.pose.z + k1zp / 2,
                                        self.pose.theta + k1_theta_p / 2, delta_t, u)
        k3xp, k3zp, k3_theta_p = self.f(self.pose.x + k2xp / 2, self.pose.z + k2zp / 2,
                                        self.pose.theta + k2_theta_p / 2, delta_t, u)
        k4xp, k4zp, k4_theta_p = self.f(self.pose.x + k3xp, self.pose.z + k3zp,
                                        self.pose.theta + k3_theta_p, delta_t, u)

        # update of the robot's state (position and orientation)
        self.pose.x += (1 / 6.0) * (k1xp + 2 * k2xp + 2 * k3xp + k4xp)
        self.pose.z += (1 / 6.0) * (k1zp + 2 * k2zp + 2 * k3zp + k4zp)
        self.pose.theta += (1 / 6.0) * (k1_theta_p + 2 * k2_theta_p + 2 * k3_theta_p + k4_theta_p)

    def dynamics(self, delta_t, u):
        """ Function to provide an interface between the model and the display
            It updates the robot's position considering that to the command U was provided for delta t seconds
        PARAMETERS:
            delta_t: (number in s) time step for the runge kutta
            u: (list of two number in rad/s) the wheel speed commands [left, right]
        """
        self.runge_kutta(delta_t, u)
