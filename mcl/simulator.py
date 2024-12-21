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
from math import pi
from random import choices, sample, random
import copy as copy
from mcl.pose import Pose3D
from .global_vars import LANDMARKS
import numpy as np

import drawing.drawing_functions as drawing
from mcl.global_vars import WORLD_SIZE
from mcl.monte_carlo import Robot

NUM_EXTRA_MCL_ITERATIONS = 5


class Simulator:
    """
        Class provides the simulator for the MCL workshop
    """

    def __init__(self, parameters):
        """
            Constructor of the class
            Attributes:
                parameters: (parameters.parameters.Parameters) the parameters of the simulator
        """

        """ ***** Initialization of the user interface ***** """
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

        # checkbox for randomizing the particles
        self.randomize = tk.IntVar() # randomize the particles
        self.cb_env = tk.Checkbutton(self.screen, text="Randomize",
                                     variable=self.randomize)
        self.cb_env.pack(side='left')

        """ ***** Handle some keyboard events ***** """
        self.screen.bind('<Left>', self.left_key)  # pad left arrow key
        self.screen.bind('<Right>', self.right_key)  # pad right key
        self.screen.bind('<Up>', self.up_key)  # pad up key
        self.screen.bind('<Escape>', self.close_window_event)  # pad escape key (to close the simulator)

        # to call close_window when closing the window
        self.screen.protocol("WM_DELETE_WINDOW", self.close_window)

        """ ***** Bind mouse click event ***** """
        self.canvas.bind("<Button-1>", self.kidnap_robot)

        """ ***** Initialization of the parameters ***** """
        try:
            self.world_size = (WORLD_SIZE[0], WORLD_SIZE[1])
            self.robot: Robot = getattr(parameters, "robot")
            self.predicted_robot: Robot = getattr(parameters, "predicted_robot")
            self.map = getattr(parameters, "map")
            self.number_of_particles = getattr(parameters, "number_of_particles")
            self.particles: list[Robot] = self.init_particles()
            self.landmarks = LANDMARKS
            self.percent_random_particles = getattr(parameters, "percent_random_particles")
            self.fps = getattr(parameters, "fps")

        except AttributeError as ae:
            print(ae)
            exit(0)

        self.should_resample_mcl = 0;
        self.resample_frame = 5;

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

        self.resample_frame -= 1
        if self.should_resample_mcl > 0 and self.resample_frame <= 0:
            self.resample_frame = 5
            self.should_resample_mcl -= 1

            # sensor model
            z = self.robot.get_measurements(self.landmarks)
            w = self.calculate_weights(z)
            self.resample_particles(w) # type: ignore
            self.predicted_robot.pose = self.estimate_location() # robot location estimate based on particles
            if self.randomize.get():
                self.randomize_n_particles(100)  # TODO(filip): make optional

        self.draw()
        self.screen.after(int(1000 / self.fps), self.update_simulator)


    def init_particles(self):
        """
            Method creates a list of particles with random positions and orientations.
        """
        return [Robot(Pose3D(random() * self.world_size[0], random() * self.world_size[1], random() * 2 * math.pi), weight=1/self.number_of_particles) for _ in range(self.number_of_particles)]


    def calculate_weights(self, z: list[float]) -> list[float]:
        """
            Method calculates the weights of the particles based on the measurement probabilities.
            Parameters:
                z (list[float]): The observed measurements.
            Returns:
                list[float]: The weights of the particles.
        """
        ws = np.array([p.get_measurement_prob(z, self.landmarks) for p in self.particles])
        return ws / np.sum(ws)


    def resample_particles(self, weights: np.ndarray):
        """
            Method resamples the particles based on their weights to focus on the more likely particles.
            Parameters:
                weights (list[float]): The weights of the particles.
        """
        particles = np.array([np.array([p.pose.x, p.pose.y, p.pose.theta]) for p in self.particles])
        sampled_rows = np.random.choice(particles.shape[0], size=self.number_of_particles, p=weights, replace=True)

        sampled_particles = particles[sampled_rows]

        self.particles = [Robot(Pose3D(p[0], p[1], p[2])) for p in sampled_particles]
        return


    def randomize_n_particles(self, n: int):
        """
            Method randomizes the positions and orientations of a specified number of particles.
            Parameters:
                n (int): The number of particles to randomize.
        """
        rnd_particles = sample(self.particles, n)
        for p in rnd_particles:
            p.set_pose(Pose3D(random() * self.world_size[0], random() * self.world_size[1], random() * 2 * math.pi))


    def estimate_location(self) -> Pose3D:
        """
            Method estimates the location of the robot based on the particle with the highest weight.
            Returns:
                Pose3D: The estimated pose of the robot.
        """
        mp = max(self.particles, key=lambda x: x.weight)
        return Pose3D(mp.pose.x, mp.pose.y, mp.pose.theta)


    def move_particles(self, forward: float, turn: float):
        """
            Method moves the particles based on the given forward and turn values.
            Parameters:
                forward (float): The forward movement.
                turn (float): The turn movement.
        """
        return [p.move(forward=forward, turn=turn) for p in self.particles]


    def draw(self):
        """
            Method draws the grid map, particles, robot, and landmarks on the canvas.
        """
        self.canvas.delete("all")  # we start by removing the old display

        drawing.draw_grid_map(self.canvas, self.map)
       
        drawing.draw_particles(self.canvas, self.particles, self.map)
        drawing.draw_predicted_robot(self.canvas, self.predicted_robot, self.map)

        drawing.draw_robot(self.canvas, self.robot, self.map)

        drawing.draw_landmarks(self.canvas, self.map)


    def left_key(self, _):
        """ 
            Method called when pressing the left key
            Parameters:
                _: (event) the event that is not used here
        """
        forward = 0.0
        turn = -pi / 50
        self.should_resample_mcl = NUM_EXTRA_MCL_ITERATIONS
        self.move_particles(forward=forward, turn=turn)
        self.robot.move(forward=forward, turn=turn)


    def right_key(self, _):
        """ 
            Method called when pressing the right key
            Parameters:
                _: (event) the event that is not used here
        """
        forward = 0.0
        turn = pi / 50
        self.should_resample_mcl = NUM_EXTRA_MCL_ITERATIONS
        self.move_particles(forward=forward, turn=turn)
        self.robot.move(forward=forward, turn=turn)


    def up_key(self, _):
        """ 
            Method called when pressing the up key
            Parameters:
                _: (event) the event that is not used here
        """
        forward = 0.1
        turn = 0.0
        self.should_resample_mcl = NUM_EXTRA_MCL_ITERATIONS
        self.move_particles(forward=forward, turn=turn)
        self.robot.move(forward=forward, turn=turn)


    def kidnap_robot(self, event: tk.Event):
        """
            Method updates the robot's position based on the click coordinates.
            Parameters:
                event (tkinter.Event): The event object containing the click coordinates.
        """
        x_click = event.x * self.world_size[0] / self.canvas.winfo_width()
        y_click = event.y * self.world_size[1] / self.canvas.winfo_height()
        self.robot.set_pose(Pose3D(x_click, y_click, self.robot.pose.theta))
        self.draw()


    def close_window(self):
        self.close_window_event(None)


    def close_window_event(self, _):
        self.screen.destroy()


if __name__ == "__main__":
    print("This file should not be run as main file...")
