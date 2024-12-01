# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the simulator for the RRT workshop
"""

from geometry.point import Point2D
import tkinter as tk
from datetime import datetime
from tp_rrt.utils import follow_path
from parameters.parameters import Parameters
import drawing.drawing_functions as drawing_fct

""" ***** The simulator class ***** """


class Simulator:

    # Initialization of the class and the User Interface
    def __init__(self, parameters:Parameters):
        """
        constructor of the class
        PARAMETERS:
            parameters: (parameters.parameters.Parameters) the parameters of the simulator
        """
        self.screen = tk.Tk()  # the window
        self.screen.title("Mobile robotics - Rapidly Explored Random Tree")  # define the window name
        self.screen.geometry("800x800+100+00")  # define the default window geometry (size and position)
        self.canvas = tk.Canvas(self.screen, background='white')  # the area to draw, the background color is white
        self.canvas.pack(fill=tk.BOTH, expand=tk.YES, side=tk.TOP)  # add the drawing area to the window

        self.canvas.bind("<Button-1>", self.left_click)

        """ ***** The UI buttons ***** """

        # the start button
        self.btnStart = tk.Button(self.screen, text="Start following path")  # Creation of the button "start"
        self.btnStart.pack(fill=tk.X, side=tk.BOTTOM)           # adding the button to the window
        self.btnStart.bind('<Button-1>', self.btn_start_event)  # Define the function to call when the button is pressed
        # the "improve path" button
        self.btnImprovePath = tk.Button(self.screen, text="Improve Path")
        self.btnImprovePath.pack(fill=tk.X, side=tk.BOTTOM)
        self.btnImprovePath.bind('<Button-1>', self.btn_improve_path_event)
        # the build path button
        self.btnBuildPath = tk.Button(self.screen, text="Build Path")  # Creation of the button "start"
        self.btnBuildPath.pack(fill=tk.X, side=tk.BOTTOM)              # adding the button to the window
        self.btnBuildPath.bind('<Button-1>', self.btn_build_path_event)  # The function called when button is pressed
        # the build rrt button
        self.btnBuildRRt = tk.Button(self.screen, text="Build RRT")  # Creation of the button "start"
        self.btnBuildRRt.pack(fill=tk.X, side=tk.BOTTOM)             # adding the button to the window
        self.btnBuildRRt.bind('<Button-1>', self.btn_build_rrt_event)  # The function to call when the button is pressed

        """ ***** The checkboxes and radio-buttons ***** """

        # define the variables to get/set the values of the cb and rb
        self.map_to_display = tk.IntVar()
        self.display_env = tk.IntVar()
        self.display_robot = tk.IntVar()
        self.display_start_and_goal = tk.IntVar()
        self.display_rrt = tk.IntVar()
        self.display_path = tk.IntVar()

        # create the cb and rb
        self.cb_map = tk.Checkbutton(self.screen, text="Map",
                                     variable=self.map_to_display)
        self.cb_env = tk.Checkbutton(self.screen, text="Environment",
                                     variable=self.display_env)
        self.cb_robot = tk.Checkbutton(self.screen, text="Robot",
                                       variable=self.display_robot)
        self.cb_start_and_goal = tk.Checkbutton(self.screen, text="Start and Goal",
                                                variable=self.display_start_and_goal)
        self.cb_rrt = tk.Checkbutton(self.screen, text="RRT",
                                     variable=self.display_rrt)
        self.cb_path = tk.Checkbutton(self.screen, text="Path",
                                      variable=self.display_path)

        # add the cb and rb to the window
        self.cb_map.pack(side='left')
        self.cb_env.pack(side='left')
        self.cb_robot.pack(side='left')
        self.cb_start_and_goal.pack(side='left')
        self.cb_rrt.pack(side='left')
        self.cb_path.pack(side='left')

        # init some cb and rb
        self.cb_map.select()
        self.cb_robot.select()
        self.cb_start_and_goal.select()
        self.cb_rrt.select()
        self.cb_path.select()

        """ ***** Handle some keyboard events ***** """

        self.screen.bind('<Left>', self.left_key)              # pad left arrow key
        self.screen.bind('<Right>', self.right_key)            # pad right key
        self.screen.bind('<Up>', self.up_key)                  # pad up key
        self.screen.bind('<Escape>', self.close_window_event)  # pad escape key (to close the simulator)

        # to call close_window when closing the window
        self.screen.protocol("WM_DELETE_WINDOW", self.close_window)

        # INITIALIZATION OF THE PARAMETERS
        try:
            self.robot = getattr(parameters, "robot")
            self.environment = getattr(parameters, "environment")
            self.map = getattr(parameters, "map")
            self.rrt = getattr(parameters, "rrt_star")
            self.fps = getattr(parameters, "fps")
            self.rk_step = getattr(parameters, "rk_step")
        except AttributeError as ae:
            print(ae)
            exit(0)

        self.cmd = [0, 0]  # the wheel command
        self.start_position = None  # the starting position
        self.target_position = None  # the target position
        self.follow_path = False  # flag to notify when the robot should follow the path autonomously

        # to update the simulator : dynamics value and redraw. This function loops over itself with the after() function
        self.update_simulator()

        # main loop of the display window
        self.screen.mainloop()

    def left_click(self, event:tk.Event):
        """ function to handle the mouse left click event
        PARAMETERS:
            event: the tkinter event, needed to get the coordinates of the mouse when clicked
        """
        self.rrt.clear()
        self.start_position = Point2D(self.robot.pose.x, self.robot.pose.z)
        self.target_position = Point2D(drawing_fct.x_draw_2_real(self.canvas, event.x, self.environment),
                                       drawing_fct.z_draw_2_real(self.canvas, event.y, self.environment))

    # function to update the simulator (dynamical value and display)
    def update_simulator(self):
        """ function to update the dynamics of the simulation """

        # FPS expressed in ms between 2 consecutive frame
        delta_t = (1.0/self.fps)/self.rk_step  # the time step for the computation of the robot state

        # at each redisplay (new display frame)
        for _ in range(0, self.rk_step):
            # we want the computation of the robot state to be faster than the
            # display to limit the computation errors
            # display : new frame at each 100ms
            # delta_t : 1ms for the differential equation evaluation
            self.robot.dynamics(delta_t, self.cmd)

        if len(self.rrt.path) > 0 and self.follow_path is True:
            self.rrt.path, self.cmd = follow_path(self.robot, self.rrt.path)
        for _ in range(0, self.rk_step):
            # we want the computation of the robot state to be faster than the
            # display to limit the computation errors
            # display : new frame at each 100ms
            # delta_t : 1ms for the differential equation evaluation
            self.robot.dynamics(delta_t, self.cmd)
            # we also compute the new x and z position of the robot in the world

        self.cmd = [0, 0]

        # call the function that handles the display of the simulator
        self.draw()

        # to call the update_simulator function again, after waiting a certain among of time
        self.screen.after(int(1000/self.fps), self.update_simulator)

    """ ***** Drawing functions ***** """

    def update_time_canvas(self):
        """ function to display the current time in the top left corner of the window
            This allows to check that the simulation process the time correctly
        """
        # 0, 0 : text position in the window (top left corner)
        # anchor : otherwise the text is centered into the coordinates (nw: north west)
        self.canvas.create_text(0, 0, anchor="nw", text=datetime.now().strftime("%H:%M:%S.%f"))

    def draw(self):
        """ function to draw all the stuff"""
        self.canvas.delete("all")  # we start by removing the old display

        # to draw the map, it is either the cost map of the probability map (obstacles map)
        if self.map_to_display.get() == 1:
            drawing_fct.draw_seg_map(self.canvas, self.map)

        # to update the time display
        self.update_time_canvas()

        # to draw the environment, note that the map is computed from the environment.
        # the environment is assumed to be a set of segments
        if self.display_env.get() == 1:
            drawing_fct.draw_seg_environment(self.canvas, self.environment)

        if self.start_position is not None and self.target_position is not None and \
                self.display_start_and_goal.get() == 1:
            drawing_fct.draw_point(self.canvas, self.start_position, 0.1, "yellow", self.environment)
            drawing_fct.draw_point(self.canvas, self.target_position, 0.1, "red", self.environment)

        if self.display_rrt.get() == 1:
            drawing_fct.draw_rrt(self.canvas, self.rrt, "green", 0.05, self.environment)

        if self.display_robot.get() == 1:
            drawing_fct.draw_robot(self.canvas, self.robot, self.environment)

        if self.display_path.get() == 1:
            drawing_fct.draw_path(self.canvas, self.rrt, "orange", 0.05, self.environment)

    def btn_test_event(self, _:tk.Event):
        """ function called when pressing the test button
        PARAMETERS:
            _: the button event, not used
        """
        # You can put some test code here!
        # for instance:
        pass

    def btn_start_event(self, _:tk.Event):
        """ function called when pressing the start button, it changes the value of follow path flag
        PARAMETERS:
            _: the button event, not used
        """
        self.follow_path = not self.follow_path

    def btn_build_rrt_event(self, _:tk.Event):
        """ function called when pressing the build rrt button
        PARAMETERS:
            _: the button event, not used
        """
        if self.target_position is not None and self.start_position is not None:
            print("starting to build ... ")
            self.rrt.build_rrt(self.map, self.start_position, self.target_position)

            if self.rrt.succeed:
                print("... RRT built !")
            else:
                print("... RRT failed !")
        else:
            print("You need to define a GOAL first")

    def btn_build_path_event(self, _:tk.Event):
        """ function called when pressing the build path button
        PARAMETERS:
            _: the button event, not used
        """
        if self.rrt.succeed:
            print("starting to build path ... ")
            self.rrt.compute_path()
            print("... path built !")
        else:
            print("The RRT needs to be built first")

    def btn_improve_path_event(self, _:tk.Event):
        """ function called when pressing the "improve path" button
        PARAMETERS:
            _: the button event, not used
        """
        self.rrt.improve_path(self.map)

    def left_key(self, _:tk.Event):
        """ function called when pressing the left key
        PARAMETERS:
            _: the key event, not used
        """
        self.cmd = [-4, 4]

    def right_key(self, _:tk.Event):
        """ function called when pressing the right key
        PARAMETERS:
            _: the key event, not used
        """
        self.cmd = [4, -4]

    def up_key(self, _:tk.Event):
        """ function called when pressing the up key
        PARAMETERS:
            _: the key event, not used
        """
        self.cmd = [30, 30]

    def close_window(self):
        self.close_window_event(None)

    def close_window_event(self, _:tk.Event):
        self.screen.destroy()


if __name__ == "__main__":
    print("This file should not be run as main file...")
