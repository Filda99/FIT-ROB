# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the simulator for the MCL workshop
"""

import tkinter as tk
from datetime import datetime
from math import pi
from math import sqrt
from random import uniform
import copy as copy
from tp_mcl.pose import Pose3D

import drawing.drawing_functions as drawing


""" ***** The simulator class ***** """


class Simulator:
    """
    This class provides the simulator for the MCL workshop
    """

    # Initialization of the class and the User Interface
    def __init__(self, parameters):
        """
        constructor of the class
        PARAMETERS:
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

        # INITIALIZATION OF THE PARAMETERS
        try:
            self.robot = getattr(parameters, "robot")
            self.map = getattr(parameters, "map")
            self.number_of_particles = getattr(parameters, "number_of_particles")
            self.percent_random_particles = getattr(parameters, "percent_random_particles")
            self.fps = getattr(parameters, "fps")
            self.rk_step = getattr(parameters, "rk_step")

        except AttributeError as ae:
            print(ae)
            exit(0)
        self.process_mcl = False

        self.particle_to_draw = None

        # to update the simulator : dynamics value and redraw. This function loops over itself with the after() function
        self.update_simulator()

        # main loop of the display window
        self.screen.mainloop()

    # function to update the simulator (dynamical value and display)
    def update_simulator(self):
        """ Function to update the dynamic of the simulator """

        # FPS expressed in ms between 2 consecutive frame
        delta_t = (1.0 / self.fps) / self.rk_step  # the time step for the computation of the robot state

        old_pose = copy.copy(self.robot.pose)
        # at each redisplay (new display frame)
        #for _ in range(0, self.rk_step):
            # we want the computation of the robot state to be faster than the
            # display to limit the computation errors
            # display : new frame at each 100ms
            # delta_t : 1ms for the differential equation evaluation
            # self.robot.move(delta_t, self.cmd)
# TODO(filip): based on arrow -> move position of robot

        # we update the measurement set according to the new robot's pose
        # self.measurements = self.sensor.get_measurements(self.robot.pose, self.environment)
        # processing the MCL algorithm
        # first we approximate the odometry value (translation and rotation)
        delta_dst = sqrt((self.robot.pose.x - old_pose.x) ** 2 + (self.robot.pose.y - old_pose.y) ** 2)
        delta_theta = self.robot.pose.theta - old_pose.theta
        # we update the particles according to the odometry value
        # self.mcl.estimate_from_odometry(delta_dst, delta_theta)

        # then we update the weight of the particles
        # self.mcl.evaluate_particles(self.cost_map, self.measurements)

        # we re-sample the particle around the best ones
        # self.mcl.re_sampling()

        # we re-evaluate the particles after re-sampling to display the colors correctly
        # this line could be avoided to be more efficient
        # self.mcl.evaluate_particles(self.cost_map, self.measurements)

        # we add random particle to recover from kidnapping
        # the number corresponds to the percentage of random particle among all the particles
        # self.mcl.add_random_particles(self.cost_map, self.percent_random_particles)

        # if needed, we save the best particle to draw the measurement set
        # if self.mcl.id_best_particle is not None:
        #     self.particle_to_draw = self.mcl.particles[self.mcl.id_best_particle]

        self.cmd = [0, 0]

        # call the function that handles the display of the simulator
        self.draw()

        # to call the update_simulator function again, after waiting a certain among of time
        self.screen.after(int(1000 / self.fps), self.update_simulator)

    def draw(self):
        """ function to draw all the stuff"""
        self.canvas.delete("all")  # we start by removing the old display

        drawing.draw_grid_map(self.canvas, self.map)
       
        # TODO: drawing.draw_particles(self.canvas, self.particles, self.map)

        drawing.draw_robot(self.canvas, self.robot, self.map)

        drawing.draw_landmarks(self.canvas, self.map)

    def left_key(self, _):
        """ function called when pressing the left key
        PARAMETERS:
            _: (event) the event that is not used here
        """
        self.robot.move(0.0, -0.1)

    def right_key(self, _):
        """ function called when pressing the right key
        PARAMETERS:
            _: (event) the event that is not used here
        """
        self.robot.move(0.0, 0.1)

    def up_key(self, _):
        """ function called when pressing the up key
        PARAMETERS:
            _: (event) the event that is not used here
        """
        self.robot.move(0.1, 0.0)

    def i_key(self, _):
        """ function called when pressing the 'i' key
        PARAMETERS:
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
