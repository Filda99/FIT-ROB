# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides the simulator for the A* workshop
"""

import tkinter as tk
from datetime import datetime
from tp_a_star.utils import follow_path
from geometry.point import Point2D
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
        self.screen.title("Mobile robotics - A*")  # define the window name
        self.screen.geometry("800x800+100+50")  # define the default window geometry (size and position)
        self.canvas = tk.Canvas(self.screen, background='white')  # the area to draw, the background color is white
        self.canvas.pack(fill=tk.BOTH, expand=tk.YES, side=tk.TOP)  # add the drawing area to the window

        """ ***** The UI buttons ***** """

        # the test button
        self.btnTest = tk.Button(self.screen, text="Test")  # Creation of the button "Test"
        self.btnTest.pack(fill=tk.X, side=tk.BOTTOM)        # adding the button to the window
        self.btnTest.bind('<Button-1>', self.btn_test_event)   # Define the function to call when the button is pressed

        """ ***** The checkboxes and radio-buttons ***** """

        # define the variables to get/set the values of the cb and rb
        self.display_map = tk.IntVar()
        self.display_env = tk.IntVar()
        self.display_robot = tk.IntVar()
        self.display_path = tk.IntVar()
        self.auto_move = tk.IntVar()

        # create the cb and rb
        self.cb_map = tk.Checkbutton(self.screen, text="Map",
                                     variable=self.display_map)
        self.cb_env = tk.Checkbutton(self.screen, text="Environment",
                                     variable=self.display_env)
        self.cb_robot = tk.Checkbutton(self.screen, text="Robot",
                                       variable=self.display_robot)
        self.cb_path = tk.Checkbutton(self.screen, text="Path",
                                      variable=self.display_path)
        self.cb_move_auto = tk.Checkbutton(self.screen, text="Move autonomously",
                                           variable=self.auto_move)
        self.rb_map = tk.Radiobutton(self.screen, text="Grid map",
                                     variable=self.display_map, value=1)
        self.rb_astar = tk.Radiobutton(self.screen, text="A*",
                                       variable=self.display_map, value=2)

        # add the cb and rb to the window
        self.cb_robot.pack(side='left')
        self.cb_env.pack(side='left')
        self.cb_path.pack(side='left')
        self.rb_map.pack(side='left')
        self.rb_astar.pack(side='left')
        self.cb_move_auto.pack(side='left')

        # init some cb and rb
        self.rb_map.select()
        self.cb_robot.select()
        self.rb_astar.select()

        """ ***** Handle some keyboard events ***** """

        self.screen.bind('<Left>', self.left_key)              # pad left arrow key
        self.screen.bind('<Right>', self.right_key)            # pad right key
        self.screen.bind('<Up>', self.up_key)                  # pad up key
        self.screen.bind('<i>', self.i_key)                    # pad i key
        self.screen.bind('<Escape>', self.close_window_event)  # pad escape key (to close the simulator)

        # to call close_window when closing the window
        self.screen.protocol("WM_DELETE_WINDOW", self.close_window)

        self.screen.bind("<Button-1>", self.left_click)

        # INITIALIZATION OF THE PARAMETERS
        try:
            self.robot = getattr(parameters, "robot")
            self.environment = getattr(parameters, "environment")
            self.map = getattr(parameters, "map")
            self.a_star = getattr(parameters, "a_star")
            self.fps = getattr(parameters, "fps")
            self.rk_step = getattr(parameters, "rk_step")
        except AttributeError as ae:
            print(ae)
            exit(0)

        self.cmd = [0, 0]  # the wheel command
        self.start_position = None  # the starting position
        self.target_position = None  # the target position
        """self.follow_path = False  # flag to notify when the robot should follow the path autonomously"""

        # to update the simulator : dynamics value and redraw. This function loops over itself with the after() function
        self.update_simulator()

        # main loop of the display window
        self.screen.mainloop()

    def left_click(self, event:tk.Event):
        """ function to handle the mouse left click event
        PARAMETERS:
            event: the tkinter event, needed to get the coordinates of the mouse when clicked
        """
        if event.widget == self.canvas:  # if the click event occurs over the canvas

            x_node = self.x_draw_2_node(event.x)  # get the node corresponding to the click coordinates
            z_node = self.z_draw_2_node(event.y)

            self.start_position = Point2D(self.x_real_2_node(self.robot.pose.x), self.z_real_2_node(self.robot.pose.z))
            self.target_position = Point2D(x_node, z_node)

            self.a_star.find_path(self.start_position, self.target_position)

    # function to update the simulator (dynamical value and display)
    def update_simulator(self):
        """ function to update the dynamics of the simulation """

        # FPS expressed in ms between 2 consecutive frame
        delta_t = (1.0/self.fps)/self.rk_step  # the time step for the computation of the robot state

        # at each redisplay (new display frame)
        for i in range(0, self.rk_step):
            # we want the computation of the robot state to be faster than the
            # display to limit the computation errors
            # display : new frame at each 100ms
            # delta_t : 1ms for the differential equation evaluation
            self.robot.dynamics(delta_t, self.cmd)

        self.cmd = [0, 0]

        if self.auto_move.get() == 1:
            self.cmd = follow_path(self.robot, self.a_star)

        # call the function that handles the display of the simulator
        self.draw()

        # to call the update_simulator function again, after waiting a certain among of time
        self.screen.after(int(1000/self.fps), self.update_simulator)

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

        # to draw the map or the A*
        if self.display_map.get() == 1:
            drawing_fct.draw_grid_map(self.canvas, self.map)
        elif self.display_map.get() == 2:
            drawing_fct.draw_astar(self.canvas, self.a_star)

        # to update the time display
        self.update_time_canvas()

        # to draw the environment, note that the map is computed from the environment.
        # the environment is assumed to be a set of segments
        if self.display_env.get() == 1:
            drawing_fct.draw_seg_environment(self.canvas, self.environment)

        # to draw the actual robot
        if self.display_robot.get() == 1:
            drawing_fct.draw_robot(self.canvas, self.robot, self.environment)

        # to draw the path (where the robot is supposed to go)
        if self.display_path.get() == 1:
            drawing_fct.draw_path(self.canvas, self.a_star, "orange", 0.04, self.a_star)

    def x_draw_2_node(self, x_draw:int):
        """ function to convert an x value from the display to an x index in the map"""
        x_real = drawing_fct.x_draw_2_real(self.canvas, x_draw, self.environment)
        x_map = x_real / self.a_star.sizeX
        return int(x_map)

    def z_draw_2_node(self, z_draw:int):
        """ function to convert a z value from the display to a z index in the map"""
        z_real = drawing_fct.z_draw_2_real(self.canvas, z_draw, self.environment)
        z_map = z_real / self.a_star.sizeZ
        return int(z_map)

    def x_real_2_node(self, x_real:float):
        """ function to convert an x value from the display to an x index in the map"""
        x_map = x_real / self.a_star.sizeX
        return int(x_map)

    def z_real_2_node(self, z_real:float):
        """ function to convert a z value from the display to a z index in the map"""
        z_map = z_real / self.a_star.sizeZ
        return int(z_map)

    def btn_test_event(self, _:tk.Event):
        # You can put some test code here!
        # for instance:
        print(f"test button clicked, current wheel cmd {self.cmd}")
        pass

    def left_key(self, _:tk.Event):
        self.cmd = [-4, 4]

    def right_key(self, _:tk.Event):
        self.cmd = [4, -4]

    def up_key(self, _:tk.Event):
        self.cmd = [30, 30]

    def i_key(self, _:tk.Event):
        self.robot.x = 2.0
        self.robot.z = 2.0
        self.robot.theta = 0.0

    def rb_map_sel(self):
        # nothing to do here...
        pass

    def close_window(self):
        self.close_window_event(None)

    def close_window_event(self, _:tk.Event):
        self.screen.destroy()


if __name__ == "__main__":
    print("This file should not be run as main file...")
