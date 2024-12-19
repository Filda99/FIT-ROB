"""
    Project: ROBa project
    File: simulator.py
    Description: This file contains the main simulator logic for the robotics project.

    Authors:
        - Author 1: xstolf00, xstolf00@stud.fit.vutbr.cz
        - Author 2: xjahnf00, xjahnf00@vutbr.cz

    Date of Creation: 2024-12-19
"""

import math
import tkinter as tk
from datetime import datetime
from math import pi
from math import sqrt
from random import sample, uniform, random
import copy as copy
from geometry.point import Point2D
from tp_mcl.pose import Pose3D
from landmarks.landmarks import get_landmarks

import drawing.drawing_functions as drawing
from tp_mcl.global_vars import WORLD_SIZE
from tp_mcl.monte_carlo import Robot



""" ***** The simulator class ***** """

class Simulator:
    """
    This class provides the simulator for the MCL workshop
    """

    def __init__(self, parameters):
        """
        constructor of the class
        Parameters:
            parameters: (parameters.parameters.Parameters) the parameters of the simulator
        """
        # INITIALIZATION OF THE USER INTERFACE
        self.screen = tk.Tk()  # the window
        self.screen.title("ROBa project - Monte Carlo Localization")  # define the window name
        self.screen.geometry("800x800+100+00")  # define the default window geometry (size and position)
        self.canvas = tk.Canvas(self.screen, background='white')  # the area to draw, the background color is white
        self.canvas.pack(fill=tk.BOTH, expand=tk.YES, side=tk.TOP)  # add the drawing area to the window

        """ ***** The checkboxes and radio-buttons ***** """

        # define the variables to get/set the values of the cb and rb
        self.map_to_display = tk.IntVar()
        self.display_env = tk.IntVar()
        self.display_particles = tk.IntVar()
        self.display_robot = tk.IntVar()

        """ ***** Handle some keyboard events ***** """

        self.screen.bind('<Left>', self.left_key)  # pad left arrow key
        self.screen.bind('<Right>', self.right_key)  # pad right key
        self.screen.bind('<Up>', self.up_key)  # pad up key
        self.screen.bind('<i>', self.i_key)  # pad i key
        self.screen.bind('<Escape>', self.close_window_event)  # pad escape key (to close the simulator)

        # to call close_window when closing the window
        self.screen.protocol("WM_DELETE_WINDOW", self.close_window)

        # INITIALIZATION OF THE Parameters
        try:
            self.world_size = (WORLD_SIZE[0], WORLD_SIZE[1])
            self.robot: Robot = getattr(parameters, "robot")
            self.map = getattr(parameters, "map")
            self.number_of_particles = getattr(parameters, "number_of_particles")
            self.particles: list[Robot] = self.init_particles()
            self.landmarks = get_landmarks()
            self.percent_random_particles = getattr(parameters, "percent_random_particles")
            self.fps = getattr(parameters, "fps")

        except AttributeError as ae:
            print(ae)
            exit(0)

        self.particle_to_draw = None

        self.update_simulator()

        self.screen.mainloop()


    def update_simulator(self):
        """
        Update the dynamic of the simulator.

        This function performs the following steps:
        1. Gets the measurements from the robot's sensors.
        2. Calculates the weights of the particles based on the measurements.
        3. Resamples the particles according to their weights.
        4. Randomizes a specified number of particles.
        5. Draws the updated state.
        6. Schedules the next update.
        """

        # sensor model
        z = self.robot.get_measurements(self.landmarks)
        w = self.calculate_weights(z)
        self.resample_particles(w)
        # self.robot.pose = self.estimate_location() # robot location estimate based on particles
        self.randomize_n_particles(100)  # TODO(filip): make optional
        self.draw()
        self.screen.after(int(500 / self.fps), self.update_simulator)


    def init_particles(self):
        """
        Initialize the particles.

        This function creates a list of particles with random positions and orientations.
        """
        return [Robot(Pose3D(random() * self.world_size[0], random() * self.world_size[1], random() * 2 * math.pi)) for _ in range(self.number_of_particles)]


    def calculate_weights(self, z: list[float]) -> list[float]:
        """
        Calculate the weights of the particles.

        This function calculates the weights of the particles based on the measurement probabilities.

        Parameters:
            z (list[float]): The observed measurements.

        Returns:
            list[float]: The weights of the particles.
        """
        return [p.get_measurement_prob(z, self.landmarks) for p in self.particles]


    def resample_particles(self, weights: list[float]):
        """
        Resample the particles according to the weights.

        This function resamples the particles based on their weights to focus on the more likely particles.

        Parameters:
            weights (list[float]): The weights of the particles.
        """
        tmp_particles: list[Robot] = []
        idx = int(random() * self.number_of_particles)
        beta = 0.0
        mw = max(weights)
        for _ in range(self.number_of_particles):
            beta += random() * 2.0 * mw
            while (beta > weights[idx]):
                beta -= weights[idx]
                idx = int((idx + 1) % self.number_of_particles)
            tmp_particles.append(copy.deepcopy(self.particles[idx]))  # NOTE: deepcopy is important -- otherwise particles will all become the same object
        self.particles = tmp_particles


    def randomize_n_particles(self, n: int):
        """
        Randomize n particles.

        This function randomizes the positions and orientations of a specified number of particles.

        Parameters:
            n (int): The number of particles to randomize.
        """
        rnd_particles = sample(self.particles, n)
        for p in rnd_particles:
            p.set_pose(Pose3D(random() * self.world_size[0], random() * self.world_size[1], random() * 2 * math.pi))


    def estimate_location(self) -> Pose3D:
        """
        Estimate the location of the robot.

        This function estimates the location of the robot based on the particle with the highest weight.

        Returns:
            Pose3D: The estimated pose of the robot.
        """
        mp = max(self.particles, key=lambda x: x.weight)

        return Pose3D(mp.pose.x, mp.pose.y, mp.pose.theta)


    def move_particles(self, forward: float, turn: float):
        """
        Move the particles.

        This function moves the particles based on the given forward and turn values.

        Parameters:
            forward (float): The forward movement.
            turn (float): The turn movement.
        """
        return [p.move(forward=forward, turn=turn) for p in self.particles]


    def draw(self):
        """
        Draw all the elements.

        This function draws the grid map, particles, robot, and landmarks on the canvas.
        """
        self.canvas.delete("all")  # we start by removing the old display

        drawing.draw_grid_map(self.canvas, self.map)
       
        drawing.draw_particles(self.canvas, self.particles, self.map)

        drawing.draw_robot(self.canvas, self.robot, self.map)

        drawing.draw_landmarks(self.canvas, self.map)


    def left_key(self, _):
        """ function called when pressing the left key
        Parameters:
            _: (event) the event that is not used here
        """
        forward = 0.0
        turn = -pi / 50
        self.move_particles(forward=forward, turn=turn)
        self.robot.move(forward=forward, turn=turn)


    def right_key(self, _):
        """ function called when pressing the right key
        Parameters:
            _: (event) the event that is not used here
        """
        forward = 0.0
        turn = pi / 50
        self.move_particles(forward=forward, turn=turn)
        self.robot.move(forward=forward, turn=turn)


    def up_key(self, _):
        """ function called when pressing the up key
        Parameters:
            _: (event) the event that is not used here
        """
        forward = 0.1
        turn = 0.0
        self.move_particles(forward=forward, turn=turn)
        self.robot.move(forward=forward, turn=turn)


    def i_key(self, _):
        """ function called when pressing the 'i' key
        Parameters:
            _: (event) the event that is not used here
        """
        flag = False
        while not flag:
            self.robot.pose.x = uniform(0, self.map.width)
            self.robot.pose.y = uniform(0, self.map.width)
            self.robot.pose.theta = uniform(0, 2 * pi)
            # test if the new position is OK for the robot
            if 2 <= self.robot.pose.x <= 10 and 2 <= self.robot.pose.y <= 10:
                flag = True
        print("The robot has been kidnapped!")


    def close_window(self):
        self.close_window_event(None)


    def close_window_event(self, _):
        self.screen.destroy()


if __name__ == "__main__":
    print("This file should not be run as main file...")