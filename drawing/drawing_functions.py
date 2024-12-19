"""
    Project: ROBa project
    File: drawing_functions.py
    Description: This file contains useful functions to draw on a Tkinter canvas.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""

from math import cos, sin
from mcl.global_vars import LANDMARKS


def x_real_2_draw(canvas, x_real, world_dimension):
    """
        Method to convert an x value from the world frame to the display frame
        Parameters:
            canvas: (tkinter.Canvas) the display
            x_real: (number in m) the x value in the world frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        Returns:
            (number) the corresponding x value in the canvas frame

    """
    return x_real * canvas.winfo_width() / world_dimension.width


def y_real_2_draw(canvas, y_real, world_dimension):
    """
        Method to convert a z value from the world frame to the display frame
        Parameters:
            canvas: (tkinter.Canvas) the display
            z_real: (number in m) the z value in the world frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        Returns:
            (number) the corresponding z value in the canvas frame
    """
    return y_real * canvas.winfo_height() / world_dimension.height


def x_draw_2_real(canvas, x_draw, world_dimension):
    """
        Method to convert an x value from the display frame to the world frame
        Parameters:
            canvas: (tkinter.Canvas) the display
            x_draw: (number) the x value in the display frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        Returns:
            (number) the corresponding x value in the world frame
    """
    return x_draw * world_dimension.width / canvas.winfo_width()


def z_draw_2_real(canvas, y_draw, world_dimension):
    """
        Method to convert a z value from the display frame to the world frame
        Parameters:
            canvas: (tkinter.Canvas) the display
            z_draw: (number) the z value in the display frame
            world_dimension: (should have width and height attributes) the world displayed int the canvas
        Returns:
            (number) the corresponding z value in the world frame
    """
    return y_draw * world_dimension.height / canvas.winfo_height()


def draw_robot(canvas, robot, world_dimension):
    """
        Method to draw a robot
        Parameters:
            canvas: (tkinter.Canvas) the display
            robot: (robot.Robot) the robot to draw
            world_dimension: (should have width and height attributes) the world the robot is in
    """

    """ ***** Drawing the arrow for the robot direction ***** """
    x_start = x_real_2_draw(canvas, robot.pose.x, world_dimension)
    y_start = y_real_2_draw(canvas, robot.pose.y, world_dimension)
    x_end = x_real_2_draw(canvas, robot.pose.x + cos(robot.pose.theta) * robot.robot_width, world_dimension)
    y_end = y_real_2_draw(canvas, robot.pose.y + sin(robot.pose.theta) * robot.robot_width, world_dimension)
    canvas.create_line(x_start, y_start, x_end, y_end, fill='black', arrow='last')


def draw_grid_map(canvas, grid_map):
    """
        Method to draw a grid map
        Parameters:
            canvas: (tkinter.Canvas) the display
            grid_map: (environment.GridMap) the grid map to draw
    """
    # loops over all the cells of the grid map
    for z in grid_map.cells:
        for c in z:
            color = '#%02x%02x%02x' % (int(255 - c.val * 255), int(255 - c.val * 255), int(255 - c.val * 255))
            canvas.create_rectangle(x_real_2_draw(canvas, c.x * grid_map.size_x, grid_map),
                                    y_real_2_draw(canvas, c.z * grid_map.size_z, grid_map),
                                    x_real_2_draw(canvas, c.x * grid_map.size_x + grid_map.size_x, grid_map),
                                    y_real_2_draw(canvas, c.z * grid_map.size_z + grid_map.size_z, grid_map),
                                    fill=color, outline="white")


def draw_point(canvas, point, size, color, world_dimension):
    """
        Method to draw a point with coordinates in the world frame
        Parameters:
            canvas: (tkinter.Canvas) the display
            point: (geometry.point Point2D) the point to draw
            size: (number) the size of the point to draw
            color: (string) the color to draw the point
            world_dimension: (should have width and height attributes) the world the point is defined in
    """
    x_left_top = x_real_2_draw(canvas, point.x - size, world_dimension)
    y_left_top = y_real_2_draw(canvas, point.y - size, world_dimension)
    x_right_bot = x_real_2_draw(canvas, point.x + size, world_dimension)
    y_right_bot = y_real_2_draw(canvas, point.y + size, world_dimension)
    canvas.create_oval(x_left_top, y_left_top, x_right_bot, y_right_bot, fill=color, outline=color)


def draw_particles(canvas, particles, world_dimension):
    """
        Method to draw a set particles with pose in the world frame
        Parameters:
            canvas: (tkinter.Canvas) the display
            robot: (robot.robot.Robot) the robot the particles are evaluated the pose of
            mcl: (mcl.monte_carlo MonteCarloLocalization) The MonteCarloLocalization object we want to draw
            the particle of
            world_dimension: (should have width and height attributes) the world the measurements are defined in
    """

    for particle in particles:
        draw_point(canvas, particle.pose, size=0.1, color="grey", world_dimension=world_dimension)


def draw_landmarks(canvas, world_dimension):
    """
        Method draws the landmarks on the given Tkinter canvas based on their positions in the world frame.
        Parameters:
            canvas (tkinter.Canvas): The canvas on which to draw the landmarks.
            landmarks (list): A list of landmarks, where each landmark is represented by its coordinates.
            world_dimension (object): An object representing the dimensions of the world, with width and height attributes.
    """
    for landmark in LANDMARKS:
        draw_point(canvas, landmark, size=0.3, color="red", world_dimension=world_dimension)