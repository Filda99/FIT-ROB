"""
    Project: ROBa project
    File: glob_vars.py
    Description: This file contains global variables used throughout the robotics project.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""
from geometry.point import Point2D

import numpy as np

WORLD_SIZE = (80, 80)

# LANDMARKS: list[Point2D] = [Point2D(10, 10), Point2D(50, 70), Point2D(25, 30), Point2D(10, 65), Point2D(5, 40), Point2D(50, 10), Point2D(70, 50), Point2D(70, 12)]
# LANDMARKS_NP: np.ndarray = np.array([(10, 10), (50, 70), (25, 30), (10, 65), (5, 40), (50, 10), (70, 50), (70, 12)])

LANDMARKS: list[Point2D] = [Point2D(40, 40)]
LANDMARKS_NP: np.ndarray = np.array([(40, 40)])
