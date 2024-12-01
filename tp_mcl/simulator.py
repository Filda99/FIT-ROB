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
from robot.pose import Pose3D

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
        self.btnTest.bind('<Button-1>', self.btn_test_event)  # Define the function to call when the button is pressed

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
        self.rb_cost_map = tk.Radiobutton(self.screen, text="Cost map",
                                          variable=self.map_to_display,
                                          value=2)
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
        self.rb_cost_map.pack(side='left')
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
            self.robot = getattr(parameters, "robot")
            self.sensor = getattr(parameters, "sensor")
            self.environment = getattr(parameters, "environment")
            self.map = getattr(parameters, "map")
            self.cost_map = getattr(parameters, "cost_map")
            self.number_of_particles = getattr(parameters, "number_of_particles")
            self.percent_random_particles = getattr(parameters, "percent_random_particles")
            self.fps = getattr(parameters, "fps")
            self.rk_step = getattr(parameters, "rk_step")
            self.mcl = getattr(parameters, "mcl")

        except AttributeError as ae:
            print(ae)
            exit(0)
        self.process_mcl = False
        self.measurements = self.sensor.get_measurements(self.robot.pose, self.environment)

        self.particle_to_draw = None
        self.cmd = [0, 0]

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
        for _ in range(0, self.rk_step):
            # we want the computation of the robot state to be faster than the
            # display to limit the computation errors
            # display : new frame at each 100ms
            # delta_t : 1ms for the differential equation evaluation
            self.robot.dynamics(delta_t, self.cmd)

        # we update the measurement set according to the new robot's pose
        self.measurements = self.sensor.get_measurements(self.robot.pose, self.environment)
        if self.process_mcl:  # if we started to process the MCL (it is done by pressing the start button)
            # processing the MCL algorithm
            # first we approximate the odometry value (translation and rotation)
            delta_dst = sqrt((self.robot.pose.x - old_pose.x) ** 2 + (self.robot.pose.z - old_pose.z) ** 2)
            delta_theta = self.robot.pose.theta - old_pose.theta
            # we update the particles according to the odometry value
            self.mcl.estimate_from_odometry(delta_dst, delta_theta)
            # then we update the weight of the particles
            self.mcl.evaluate_particles(self.cost_map, self.measurements)
            # we re-sample the particle around the best ones
            self.mcl.re_sampling()
            # we re-evaluate the particles after re-sampling to display the colors correctly
            # this line could be avoided to be more efficient
            self.mcl.evaluate_particles(self.cost_map, self.measurements)
            # we add random particle to recover from kidnapping
            # the number corresponds to the percentage of random particle among all the particles
            self.mcl.add_random_particles(self.cost_map, self.percent_random_particles)
            # if needed, we save the best particle to draw the measurement set
            if self.mcl.id_best_particle is not None:
                self.particle_to_draw = self.mcl.particles[self.mcl.id_best_particle]
        self.cmd = [0, 0]

        # call the function that handles the display of the simulator
        self.draw()

        # to call the update_simulator function again, after waiting a certain among of time
        self.screen.after(int(1000 / self.fps), self.update_simulator)

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
        if self.map_to_display.get() == 1:
            drawing.draw_grid_map(self.canvas, self.map)
        elif self.map_to_display.get() == 2:
            drawing.draw_cost_map(self.canvas, self.cost_map)

        # to display for text over the user interface (time, how to...)
        self.add_text()

        # to draw the environment, note that the map is computed from the environment.
        # the environment is assumed to be a set of segments
        if self.display_env.get() == 1:
            drawing.draw_seg_environment(self.canvas, self.environment)

        # to display the LiDAR sensor measurements
        if self.display_lidar.get() == 1:
            # the LiDAR data is displayed according to the best particle's pose
            # note that to be displayed according to the best particle, it has to be defined...
            if self.mcl.id_best_particle is None:
                print("[Error !!] best particle not defined")
                self.rb_lidar_none.select()
            else:
                drawing.draw_lidar_measurements(self.canvas, self.measurements, self.particle_to_draw.pose, self.map)
        elif self.display_lidar.get() == 2:
            # the LiDAR data is displayed according to the robot's pose
            drawing.draw_lidar_measurements(self.canvas, self.measurements, self.robot.pose, self.map)

        # to draw the MCL particles
        if self.display_particles.get() == 1:
            drawing.draw_particles(self.canvas, self.robot, self.mcl, self.map)

        # to draw the actual robot
        if self.display_robot.get() == 1:
            drawing.draw_robot(self.canvas, self.robot, self.map)

    def btn_start_event(self, _):
        """ function called when pressing the start button
        PARAMETERS:
            _: (event) the button event that is not used here
        """
        self.cost_map.compute_cost_map(self.map)
        self.mcl.init_particles(self.cost_map, self.number_of_particles)
        self.process_mcl = True

    def btn_test_event(self, _):
        """ function called when pressing the test button
        PARAMETERS:
            _: (event) the button event that is not used here
        """
        # You can put some test code here!
        # for instance:
        self.cost_map.compute_cost_map(self.map)
        if self.cost_map is not None:
            print(self.cost_map.max_cost)  # must be 11
            print(self.cost_map.evaluate_cost(Pose3D(2, 2, 0), self.measurements), ": must be 0")
            print(self.cost_map.evaluate_cost(Pose3D(5, 2, 0), self.measurements), ": must be 4")
            print(self.cost_map.evaluate_cost(Pose3D(2, 2, -3.14), self.measurements), ": must be 29")
            print(self.cost_map.evaluate_cost(Pose3D(5, 4, 3.14 / 2), self.measurements), ": must be 75")

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
            self.robot.pose.x = uniform(0, self.map.width)
            self.robot.pose.z = uniform(0, self.map.width)
            self.robot.pose.theta = uniform(0, 2 * pi)
            # test if the new position is OK for the robot
            if 2 <= self.robot.pose.x <= 10 and 2 <= self.robot.pose.z <= 10:
                flag = True
        print("The robot has been kidnapped!")

    def close_window(self):
        self.close_window_event(None)

    def close_window_event(self, _):
        self.screen.destroy()


if __name__ == "__main__":
    print("This file should not be run as main file...")
