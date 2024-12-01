# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"

"""
    This file provides the User Interface for the ICP workshop
"""

import tkinter as tk
from math import pi, cos, sin
from random import uniform
from robot.pose import Pose3D

import drawing.drawing_functions as drawing_fct
from tp_icp.icp import ICP
from tp_icp.simplex import Simplex
from tp_icp.vertex import Vertex
from parameters.parameters import Parameters
import copy as copy


""" ***** The simulator class ***** """


class Simulator:
    """
    This class provides the simulator for the MCL workshop
    """
    def btn_test_event(self, _:tk.Event):
        """
        The function called when pressing the Test button, can be modifier to test the implementation as needed
        :param _: (tkinter.Event) the event, not used
        """
        print(f"Test button event, initial pose: = {self.initial_pose}")

    # Initialization of the class and the User Interface
    def __init__(self, parameters:Parameters):
        """
        constructor of the class
        :param parameters: (parameters.parameters.Parameters) the parameters of the simulator
        """
        # INITIALIZATION OF THE USER INTERFACE
        self.screen = tk.Tk()  # the window
        self.screen.title("Mobile robotics - ICP")  # define the window name
        self.screen.geometry("600x600+0+00")  # define the default window geometry (size and position)
        self.canvas = tk.Canvas(self.screen, background='white')  # the area to draw, the background color is white
        self.canvas.pack(fill=tk.BOTH, expand=tk.YES, side=tk.TOP)  # add the drawing area to the window

        self.canvas.bind('<Configure>', self.resize)

        self.screen.bind('<Escape>', self.close_window_event)  # pad escape key (to close the simulator)

        # to call close_window when closing the window
        self.screen.protocol("WM_DELETE_WINDOW", self.close_window)

        """ ***** The UI buttons ***** """
        self.btnTest = tk.Button(self.screen, text="Test")  # Creation of the button "Test"
        self.btnTest.pack(fill=tk.X, side=tk.BOTTOM)  # adding the button to the window
        self.btnTest.bind('<Button-1>', self.btn_test_event)  # Define the function to call when pressing the btn

        self.btnNAndM = tk.Button(self.screen, text="Nelder and Mead")  # Creation of the button "Nelder and Mead"
        self.btnNAndM.pack(fill=tk.X, side=tk.BOTTOM)  # adding the button to the window
        self.btnNAndM.bind('<Button-1>', self.btn_nelder_and_mead_event)  # Define the function processing the event

        self.btnAssociate = tk.Button(self.screen, text="Associate")  # Creation of the button "Associate"
        self.btnAssociate.pack(fill=tk.X, side=tk.BOTTOM)  # adding the button to the window
        self.btnAssociate.bind('<Button-1>',
                               self.btn_associate_event)  # Define the function to call when the button is pressed

        self.btnInit = tk.Button(self.screen, text="Init")  # Creation of the button "Init"
        self.btnInit.pack(fill=tk.X, side=tk.BOTTOM)  # adding the button to the window
        self.btnInit.bind('<Button-1>', self.btn_init_event)  # Define the function to call when the button is pressed

        # define the variables to get/set the values of the cb and rb
        self.display_model = tk.IntVar()
        self.display_scene = tk.IntVar()
        self.ass_algorithm = tk.IntVar()
        self.display_associations = tk.IntVar()
        self.display_estimated_pose = tk.IntVar()

        self.lb_association = tk.Label(self.screen, text=" | Association:")

        self.cb_model = tk.Checkbutton(self.screen, text="Model",
                                       variable=self.display_model, command=self.checkbox_event)
        self.cb_scene = tk.Checkbutton(self.screen, text="Scene",
                                       variable=self.display_scene, command=self.checkbox_event)
        self.cb_associations = tk.Checkbutton(self.screen, text="Associations",
                                              variable=self.display_associations, command=self.checkbox_event)

        self.rb_ass_index2index = tk.Radiobutton(self.screen, text="Index 2 index", variable=self.ass_algorithm,
                                                 value=1)
        self.rb_ass_closest = tk.Radiobutton(self.screen, text="Closest", variable=self.ass_algorithm, value=2)

        self.cb_model.pack(side='left')
        self.cb_scene.pack(side='left')
        self.cb_associations.pack(side='left')
        self.lb_association.pack(side='left')
        self.rb_ass_index2index.pack(side='left')
        self.rb_ass_closest.pack(side='left')

        self.model = []
        self.scene = []
        self.associations = []

        self.measurements = []

        # INITIALIZATION OF THE PARAMETERS

        try:
            self.robot = getattr(parameters, "robot")
            self.sensor = getattr(parameters, "sensor")
            self.environment = getattr(parameters, "environment")
            self.initial_pose = getattr(parameters, "initial_pose")
        except AttributeError as ae:
            print(f"ERROR: {ae}")
            exit(0)

        self.scene_pose = copy.deepcopy(self.initial_pose)

        # init some cb and rb
        self.rb_ass_index2index.select()
        self.cb_model.select()
        self.cb_scene.select()
        self.cb_associations.select()

        self.screen.mainloop()

    def btn_init_event(self, _:tk.Event):
        """
        Function called when pressing the init button
        It initializes the model and the scene randomly
        :param _: (tkinter.Event) the event, not used
        """
        self.robot.pose.theta = uniform(-pi / 10, pi / 10) + self.initial_pose.theta
        self.robot.pose.x = uniform(-1, 1) + self.initial_pose.x
        self.robot.pose.z = uniform(-1, 1) + self.initial_pose.z

        self.measurements = self.sensor.get_measurements(self.robot.pose, self.environment)

        self.model = ICP.convert_measurements_to_points(self.measurements, self.robot.pose)

        self.scene = ICP.convert_measurements_to_points(self.measurements, self.initial_pose)
        self.scene_pose = copy.deepcopy(self.initial_pose)

        self.associations.clear()

        self.draw()

    def btn_nelder_and_mead_event(self, _:tk.Event):
        """
        Function called when pressing the Nelder and Mead button
        It calls the ICP.nelder_and_mead() function to estimate the position of the scene
        Association must be done before calling this function
        It transforms the scene according to the found pose and update the display
        :param _: (tkinter.Event) the event, not used
        """
        if len(self.associations) == len(self.scene):
            # initialization of the first simplex randomly around (0,0,0)
            rand_x = 0.5
            rand_z = 0.5
            rand_theta = 0.1

            t_x = 0
            t_z = 0
            t_theta = 0

            simplex = Simplex()
            simplex[0] = Vertex(Pose3D(t_x, t_z, t_theta))
            simplex[1] = Vertex(Pose3D(uniform(t_x - rand_x, t_x + rand_x),
                                       uniform(t_z - rand_z, t_z + rand_z),
                                       uniform(t_theta - rand_theta, t_theta + rand_theta)))
            simplex[2] = Vertex(Pose3D(uniform(t_x - rand_x, t_x + rand_x),
                                       uniform(t_z - rand_z, t_z + rand_z),
                                       uniform(t_theta - rand_theta, t_theta + rand_theta)))
            simplex[3] = Vertex(Pose3D(uniform(t_x - rand_x, t_x + rand_x),
                                       uniform(t_z - rand_z, t_z + rand_z),
                                       uniform(t_theta - rand_theta, t_theta + rand_theta)))

            # calling the Nelder and Mead to find the best pose for the scene
            t_pose = ICP.nelder_and_mead(simplex, self.model, self.scene, self.associations)

            # updating the pose of the scene according to the estimated transform
            self.scene_pose = Pose3D(self.scene_pose.x * cos(t_pose.theta) - self.scene_pose.z * sin(t_pose.theta)
                                     + t_pose.x,
                                     self.scene_pose.x * sin(t_pose.theta) + self.scene_pose.z * cos(t_pose.theta)
                                     + t_pose.z,
                                     self.scene_pose.theta + t_pose.theta)

            print(f"robot pose:{self.robot.pose}, estimated pose: {self.scene_pose}")
            # update all the point of the scene according to the transform
            for point in self.scene:
                new_x = point.x * cos(t_pose.theta) - point.z * sin(t_pose.theta) + t_pose.x
                new_z = point.x * sin(t_pose.theta) + point.z * cos(t_pose.theta) + t_pose.z
                point.x = new_x
                point.z = new_z

            # redraw the display
            self.draw()
        else:
            print("Association must be done first!")

    def btn_associate_event(self, _:tk.Event):
        """
        Function called when pressing association button
        It calls the ICP.associate() function to associate the points of the scene to the model
        The used algorithm to process the associations depends on the radio button
        :param _: (tkinter.Event) the event, not used
        """
        self.associations = ICP.associate(self.model, self.scene, algorithm=self.ass_algorithm.get())
        self.draw()

    def resize(self, _:tk.Event):
        """
        Function called when resizing the window: update the display
        :param _: (tkinter.Event) the event, not used
        """
        self.draw()

    def draw(self):
        """ function to draw all the stuff"""
        self.canvas.delete("all")  # we start by removing the old display

        drawing_fct.draw_seg_environment(self.canvas, self.environment, color="grey")

        if self.display_model.get() == 1:
            drawing_fct.draw_points(self.canvas, self.model, 0.1, "red", self.environment)
        if self.display_scene.get() == 1:
            drawing_fct.draw_points(self.canvas, self.scene, 0.1, "blue", self.environment)
        if self.display_associations.get() == 1:
            drawing_fct.draw_associations(self.canvas, self.model, self.scene, self.associations, self.environment)

    def checkbox_event(self):
        """
        Function called when modifying the checkbox values: updates the display
        """
        self.draw()

    def close_window(self):
        self.close_window_event(None)

    def close_window_event(self, _:tk.Event):
        self.screen.destroy()


if __name__ == "__main__":
    print("This file should not be run as main file...")
