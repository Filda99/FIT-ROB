# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the simulator for the MCL workshop
"""

import math
import tkinter as tk
from datetime import datetime
from math import pi
from math import sqrt
from random import sample, uniform, random
import copy as copy
from matplotlib import pyplot as plt

from geometry.point import Point2D
from robot.pose import Pose3D

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
        PARAMETERS:
            parameters: (parameters.parameters.Parameters) the parameters of the simulator
        """
        # INITIALIZATION OF THE USER INTERFACE
        self.screen = tk.Tk()  # the window
        self.screen.title("Mobile robotics - Monte Carlo Localization")  # define the window name
        self.screen.geometry("800x800+100+00")  # define the default window geometry (size and position)
        self.canvas = tk.Canvas(self.screen, background='white')  # the area to draw, the background color is white
        self.canvas.pack(fill=tk.BOTH, expand=tk.YES, side=tk.TOP)  # add the drawing area to the window

        """ ***** The UI buttons ***** """

        # the start button
        self.btnStart = tk.Button(self.screen, text="Start")  # Creation of the button "start"
        self.btnStart.pack(fill=tk.X, side=tk.BOTTOM)  # adding the button to the window
        self.btnStart.bind('<Button-1>', self.btn_start_event)  # Define the function to call when the button is pressed

        self.btnTest = tk.Button(self.screen, text="Test")  # Creation of the button "Test"
        self.btnTest.pack(fill=tk.X, side=tk.BOTTOM)  # adding the button to the window
        # self.btnTest.bind('<Button-1>', self.btn_test_event)  # Define the function to call when the button is pressed

        """ ***** The checkboxes and radio-buttons ***** """

        # define the variables to get/set the values of the cb and rb
        self.map_to_display = tk.IntVar()
        self.display_env = tk.IntVar()
        self.display_particles = tk.IntVar()
        self.display_robot = tk.IntVar()
        self.display_lidar = tk.IntVar()

        # create the cb and rb
        self.rb_map = tk.Radiobutton(self.screen, text="Map",
                                     variable=self.map_to_display,
                                     value=1)
        # self.rb_cost_map = tk.Radiobutton(self.screen, text="Cost map",
                                          # variable=self.map_to_display,
                                          # value=2)
        self.cb_env = tk.Checkbutton(self.screen, text="Environment",
                                     variable=self.display_env)
        self.cb_particles = tk.Checkbutton(self.screen, text="Particles",
                                           variable=self.display_particles)
        self.cb_robot = tk.Checkbutton(self.screen, text="Robot",
                                       variable=self.display_robot)
        self.lb_lidar = tk.Label(self.screen, text=" | LiDAR:")
        self.rb_lidar_best = tk.Radiobutton(self.screen, text="Best particle",
                                            variable=self.display_lidar,
                                            value=1)
        self.rb_lidar_robot = tk.Radiobutton(self.screen, text="Robot",
                                             variable=self.display_lidar,
                                             value=2)
        self.rb_lidar_none = tk.Radiobutton(self.screen, text="None",
                                            variable=self.display_lidar,
                                            value=3)

        # add the cb and rb to the window
        self.rb_map.pack(side='left')
        # self.rb_cost_map.pack(side='left')
        self.cb_env.pack(side='left')
        self.cb_particles.pack(side='left')
        self.cb_robot.pack(side='left')
        self.lb_lidar.pack(side='left')
        self.rb_lidar_robot.pack(side='left')
        self.rb_lidar_best.pack(side='left')
        self.rb_lidar_none.pack(side='left')

        # init some cb and rb
        self.rb_map.select()
        self.cb_particles.select()
        self.cb_robot.select()
        self.rb_lidar_robot.select()

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
            self.world_size = (80, 80)
            self.robot: Robot = getattr(parameters, "robot")
            self.number_of_particles = getattr(parameters, "number_of_particles")
            self.particles: list[Robot] = self.init_particles()
            self.landmarks: list[Point2D] = [Point2D(10, 10), Point2D(20, 50), Point2D(25, 30), Point2D(10, 65), Point2D(5, 40), Point2D(50, 10), Point2D(70, 50), Point2D(70, 12)]
            # self.sensor = getattr(parameters, "sensor")
            # self.environment = getattr(parameters, "environment")
            # self.map = getattr(parameters, "map")
            # self.cost_map = getattr(parameters, "cost_map")
            self.percent_random_particles = getattr(parameters, "percent_random_particles")
            self.fps = getattr(parameters, "fps")
            self.rk_step = getattr(parameters, "rk_step")
            # self.mcl = getattr(parameters, "mcl")

        except AttributeError as ae:
            print(ae)
            exit(0)

        self.particle_to_draw = None
        self.cmd = [0, 0]

        # ===== tmp
        self.count = 0
        # ===== tmp

        self.update_simulator()

        self.screen.mainloop()


    def update_simulator(self):
        """ Function to update the dynamic of the simulator """

        # ===== old code -- do we need this?
        # FPS expressed in ms between 2 consecutive frame
        delta_t = (1.0 / self.fps) / self.rk_step  # the time step for the computation of the robot state
        old_pose = copy.copy(self.robot.pose)
        # at each redisplay (new display frame)
        for _ in range(0, self.rk_step):
            pass   # TODO(filip): how to handle this?
            # we want the computation of the robot state to be faster than the
            # display to limit the computation errors
            # display : new frame at each 100ms
            # delta_t : 1ms for the differential equation evaluation
            # self.robot.dynamics(delta_t, self.cmd)
        # ===== old code -- do we need this?


        forward, turn = self.get_movement()
        self.robot.move(forward=forward, turn=turn)
        self.move_particles(forward=forward, turn=turn)

        # sensor model
        z = self.robot.get_measurements(self.landmarks)
        w = self.calculate_weights(z)


        self.resample_particles(w)


        loc = self.estimate_location() # robot location estimate based on particles

        # ====== tmp{{{
        plt.clf()
        plt.scatter([p.pose.x for p in self.particles], [p.pose.y for p in self.particles], color='blue', s=5, label='Particles')
        plt.scatter(self.robot.pose.x, self.robot.pose.y, color='green', alpha=1, s=20, label='True Position')
        plt.scatter(loc.x, loc.z, color='orange', alpha=1, s=20, label='Estimated Position')
        plt.scatter([l.x for l in self.landmarks], [l.z for l in self.landmarks], color='red', s=70, label='landmarks')
        plt.xlim(0, self.world_size[0])
        plt.ylim(0, self.world_size[1])
        plt.legend()
        plt.pause(0.1)
        # ====== tmp}}}

        self.randomize_n_particles(100)  # TODO(filip): make optional

        self.draw()

        self.screen.after(int(1000 / self.fps), self.update_simulator)

        # ===== tmp
        if self.count >= 50:
            exit()
        elif self.count == 25:
            self.robot.pose.x = 40
            self.robot.pose.y = 40
        self.count += 1
        # ===== tmp

    def init_particles(self):
        return [Robot(Pose3D(random() * self.world_size[0], random() * self.world_size[1], random() * 2 * math.pi)) for _ in range(self.number_of_particles)]

    def calculate_weights(self, z: list[float]) -> list[float]:
        return [p.get_measurement_prob(z, self.landmarks) for p in self.particles]

    def resample_particles(self, weights: list[float]):
        tmp_particles: list[Robot] = []
        idx = int(random() * self.number_of_particles)
        beta = 0.0;
        mw = max(weights)
        for _ in range(self.number_of_particles):
            beta += random() * 2.0 * mw;
            while (beta > weights[idx]):
                beta -= weights[idx];
                idx = int((idx + 1) % self.number_of_particles)
            tmp_particles.append(copy.deepcopy(self.particles[idx]))  # NOTE: deepcopy is important -- otherwise particles will all become the same object
        self.particles = tmp_particles

    def randomize_n_particles(self, n: int):
        rnd_particles = sample(self.particles, n)
        for p in rnd_particles:
            p.set_pose(Pose3D(random() * self.world_size[0], random() * self.world_size[1], random() * 2 * math.pi))

    def estimate_location(self) -> Point2D:
        mp = max(self.particles, key=lambda x: x.weight)

        return Point2D(mp.pose.x, mp.pose.y)


    def get_movement(self):  # TODO(filip): implement
        forward = 2.5
        turn = 0.2
        return (forward, turn)

    def move_particles(self, forward: float, turn: float):
        return [p.move(forward=forward, turn=turn) for p in self.particles]

    def add_text(self):
        """ function to add some text to the simulator user interface
        """
        # 0, 0 : text position in the window (top left corner)
        # anchor : otherwise the text is centered into the coordinates (nw: north west)
        self.canvas.create_text(5, 0, anchor="nw", text=datetime.now().strftime("%H:%M:%S.%f"))
        self.canvas.create_text(5, 15, anchor="nw", text="Use the keyboard arrows to control the robot")
        self.canvas.create_text(5, 30, anchor="nw", text="Use the 'i' key to do a kidnapping")

    def draw(self):
        """ function to draw all the stuff"""
        self.canvas.delete("all")  # we start by removing the old display

        # to draw the map, it is either the cost map of the probability map (obstacles map)
        # if self.map_to_display.get() == 1:
        #     drawing.draw_grid_map(self.canvas, self.map)
        # elif self.map_to_display.get() == 2:
        #     drawing.draw_cost_map(self.canvas, self.cost_map)

        # to display for text over the user interface (time, how to...)
        self.add_text()

        # to draw the environment, note that the map is computed from the environment.
        # the environment is assumed to be a set of segments
        # if self.display_env.get() == 1:
        #     drawing.draw_seg_environment(self.canvas, self.environment)

        # to display the LiDAR sensor measurements
        if self.display_lidar.get() == 1:
            # the LiDAR data is displayed according to the best particle's pose
            # note that to be displayed according to the best particle, it has to be defined...
            # if self.mcl.id_best_particle is None:
            #     print("[Error !!] best particle not defined")
            #     self.rb_lidar_none.select()
            # else:
            #     drawing.draw_lidar_measurements(self.canvas, self.measurements, self.particle_to_draw.pose, self.map)
            pass
        elif self.display_lidar.get() == 2:
            # the LiDAR data is displayed according to the robot's pose
            # drawing.draw_lidar_measurements(self.canvas, self.measurements, self.robot.pose, self.map)
            pass

        # to draw the MCL particles
        # if self.display_particles.get() == 1:
        #     drawing.draw_particles(self.canvas, self.robot, self.mcl, self.map)

        # to draw the actual robot
        # if self.display_robot.get() == 1:
        #     drawing.draw_robot(self.canvas, self.robot, self.map)

    def btn_start_event(self, _):
        """ function called when pressing the start button
        PARAMETERS:
            _: (event) the button event that is not used here
        """
        # self.cost_map.compute_cost_map(self.map)
        # self.mcl.init_particles(self.cost_map, self.number_of_particles)
        # self.process_mcl = True

    # def btn_test_event(self, _):
    #     """ function called when pressing the test button
    #     PARAMETERS:
    #         _: (event) the button event that is not used here
    #     """
    #     # You can put some test code here!
    #     # for instance:
    #     self.cost_map.compute_cost_map(self.map)
    #     if self.cost_map is not None:
    #         print(self.cost_map.max_cost)  # must be 11
    #         print(self.cost_map.evaluate_cost(Pose3D(2, 2, 0), self.measurements), ": must be 0")
    #         print(self.cost_map.evaluate_cost(Pose3D(5, 2, 0), self.measurements), ": must be 4")
    #         print(self.cost_map.evaluate_cost(Pose3D(2, 2, -3.14), self.measurements), ": must be 29")
    #         print(self.cost_map.evaluate_cost(Pose3D(5, 4, 3.14 / 2), self.measurements), ": must be 75")

    def left_key(self, _):
        """ function called when pressing the left key
        PARAMETERS:
            _: (event) the event that is not used here
        """
        self.cmd = [-4, 4]

    def right_key(self, _):
        """ function called when pressing the right key
        PARAMETERS:
            _: (event) the event that is not used here
        """
        self.cmd = [4, -4]

    def up_key(self, _):
        """ function called when pressing the up key
        PARAMETERS:
            _: (event) the event that is not used here
        """
        self.cmd = [30, 30]

    def i_key(self, _):
        """ function called when pressing the 'i' key
        PARAMETERS:
            _: (event) the event that is not used here
        """
        flag = False
        while not flag:
            # self.robot.pose.x = uniform(0, self.map.width)
            # self.robot.pose.y = uniform(0, self.map.width)
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
