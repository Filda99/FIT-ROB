# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file defines an ICP class that handle the ICP algorithm
"""

from math import cos, sin, sqrt
from typing import List  # for python 3.8 compatibility

from geometry.point import Point2D
from tp_icp.association import Association
from tp_icp.simplex import Simplex
from robot.lidar import LiDARMeasurement
from robot.pose import Pose3D
import copy as copy


class ICP:
    """ class to handle the Iterative Closest point algorithm
    Note that all the methods of the class are static methods...
    """

    def __init__(self):
        """ constructor of the class
        """
        pass

    @staticmethod
    def associate_by_index(model:List[Point2D], scene:List[Point2D]) -> List[Association]:
        """
        This function returns a list of associations based on the indexes of the points
            => first point of the model is associated to the first point of the scene and so on...
        It is assumed that the scene and the model have the same number of points
        :param model: (list of geometry.point.Point2D) the points of the model
        :param scene: (list of geometry.point.Point2D) the points of the scene
        :return: (list of tp_icp.Association) the list of associations
        """
        associations = []
        # TODO
        return associations

    @staticmethod
    def associate_by_closest(model:List[Point2D], scene:List[Point2D]) -> List[Association]:
        """
        This function returns a list of associations based on the euclidean distances of the points
            => the points of the scene are associated to the closest points of the model
        It is assumed that the scene and the model have the same number of points
        Note that a point of the model can be associate to several points of the scene (and so, some points pf the
        model may not be associated at all)
        :param model: (list of geometry.point.Point2D) the points of the model
        :param scene: (list of geometry.point.Point2D) the points of the scene
        :return: (list of tp_icp.Association) the list of associations
        """
        associations = []
        # TODO
        return associations

    @staticmethod
    def associate(model:List[Point2D], scene:List[Point2D], algorithm:int=1) -> List[Association]:
        """
        This functions uses the associations algorithms to return an association list
        :param model: (list of geometry.point.Point2D) the points of the model
        :param scene: (list of geometry.point.Point2D) the points of the scene
        :param algorithm: (int) the algorithm we want to use to process the association
        :return: (list of tp_icp.Association) the list of associations
        """
        if algorithm == 1:
            # the points are associated according to their indexes
            return ICP.associate_by_index(model, scene)
        elif algorithm == 2:
            # the points are associated according to their euclidean distances
            return ICP.associate_by_closest(model, scene)
        else:
            # should not be here...
            return []

    @staticmethod
    def convert_measurements_to_points(measurements:List[LiDARMeasurement], pose:Pose3D) -> List[Point2D]:
        """
        Function that convert a LiDAR measurement set to a set of Point2D according to a given pose
        :param measurements: (robot.lidar.LiDARMeasurement) the LiDAR measurements we want to convert to point cloud
        :param pose: (robot.pose.Pose3D) the pose we want to process the LiDAR measurements from
        :return: (list of geometry.point.Point2D) the set of 2D points
        """
        points = []
        for msr in measurements:
            mx = cos(msr.angle + pose.theta) * msr.distance + pose.x
            mz = sin(msr.angle + pose.theta) * msr.distance + pose.z
            points.append(Point2D(mx, mz))
        return points

    @staticmethod
    def nelder_and_mead(initial_simplex:Simplex, model:List[Point2D], scene:List[Point2D], association:List[Association]) -> Pose3D:
        """
        The Nelder and Mead optimization algorithms
        :param initial_simplex: (tp_icp.simplex.Simplex) the initial simplex, with the cost not initialized
        :param model: (list of geometry.point.Point2D) the model
        :param scene: (list of geometry.point.Point2D) the scene
        :param association: (list of tp_icp.association.Association) the associations model/scene
        :returns: (robot.pose.Pose3D) the pose solution of the optimization (the pose that minimize the distance
        between the model and the scene according to the association
        """
        # constants of the Nelder and Mead algorithm
        alpha = 1
        gamma = 2
        rho = 0.5
        sigma = 0.5
        # maximal number of iterations in order to not loop indefinitely
        nb_iteration_max = 500
        nb_iteration = 0  # current number of iterations
        # maximal number of iterations without updating the best vertex
        nb_no_change_max = 10
        nb_no_change = 0  # current number of iterations without updating the best vertex

        # Initialization of the algorithm simplex
        simplex = initial_simplex

        # TODO
        return simplex[0].pose
