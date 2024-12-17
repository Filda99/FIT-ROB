# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
 This file provides a LiDAR and a LiDARMeasurement class
"""

from math import cos
from math import pi
from math import sin

from typing import List  # for python 3.8 compatibility

from geometry.point import Point2D
from geometry.segment import Segment2D
from robot.pose import Pose3D
from environment.seg_environment import SegEnv


class LiDARMeasurement:
    """ class to handle LiDAR measurement
    """
    distance: float # (float in m) the distance of the LiDAR measurement
    angle: float    # (float in rad) the angle of the LiDAR measurement

    def __init__(self, distance:float=0.0, angle:float=0.0):
        """ constructor of the class
        PARAMETERS:
            distance : (number in m) the distance of the LiDAR measurement
            angle : (number in rad) the angle of the LiDAR measurement
        """
        self.distance = distance
        self.angle = angle


class LiDAR:
    """ class to handle a LiDAR sensor
    """
    range: float # (number in m) the distance of the LiDAR measurement

    def __init__(self, sensor_range:float=0):
        """ Constructor of the class
        PARAMETERS:
            sensor_range: (number in m) the range of the sensor
        """
        self.range = sensor_range  # m  maximal distance of a measurement

    def init_sensor(self):
        """ Function that initialize the LiDAR
        """
        # variables for the model
        self.range = 5

    def get_measurements(self, pose:Pose3D, environment:SegEnv) -> List[LiDARMeasurement]:
        """ Function that returns a measurement set, according to the position
            of the sensor (x, z), its orientation (theta) and the environment
            PARAMETERS:
                pose: (robot.pose.Pose3D) the pose of the sensor in the environment frame (x, z, theta)
                environment: (environment.seg_environment SegEnv) the environment the sensor is in
            RETURNS:
                list of LiDARMeasurement
        """
        measurements = []
        for step in range(-90, 91, 10):
            point_ext = Point2D(cos(pose.theta + step * pi / 180) * self.range + pose.x,
                                sin(pose.theta + step * pi / 180) * self.range + pose.y)
            distance = Point2D.distance(point_ext, Point2D(pose.x, pose.y))

            segment_sensor = Segment2D(Point2D(pose.x, pose.y), point_ext)
            for seg in environment.segments:
                result, new_inter = Segment2D.intersect(segment_sensor, seg)
                if result is True:
                    new_distance = Point2D.distance(new_inter, Point2D(pose.x, pose.y))
                    if new_distance < distance:
                        distance = new_distance
            if distance < self.range - 0.001:
                measurements.append(LiDARMeasurement(distance, step * pi / 180))
        return measurements
