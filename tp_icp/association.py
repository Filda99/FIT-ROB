# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file defines an Association class to deal with point association in the ICP algorithm
"""


class Association:
    """
    Class that handle an association between a model point and a scene point
    ATTRIBUTES:
        self.id_model : (int) index of the model point
        self.id_scene: (int) index of the scene point
        self.distance: (number) the euclidean distance between the model point and the scene point
    """
    id_model: int
    id_scene: int
    distance: float

    def __init__(self, id_model:int=0, id_scene:int=0, distance:float=0):
        """
        The constructor of the class
        :param id_model: (int) index of the model point
        :param id_scene: (int) index of the scene point
        :param distance: (number) euclidean distance between the model point and the scene point
        """
        self.id_model = id_model
        self.id_scene = id_scene
        self.distance = distance

    def __str__(self) -> str:
        """
        To convert an Association to a string
        :return: (string)
        """
        return f"{self.id_model} <=> {self.id_scene} : {self.distance}"
