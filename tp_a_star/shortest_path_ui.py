# coding: utf-8

__author__ = "Remy Guyonneau"
__license__ = "GPL"
__email__ = "remy.guyonneau@univ-angers.fr"
"""
    This file provides a user interface to test A*, weighted A* and Dijkstra algorithms
    This file is self sufficient, and relies on the following libraries:
        tkinter (for the display)
        math (for the computation)
        datetime (to save the file)
"""

import tkinter as tk
from tkinter import messagebox
from math import sqrt, pow
import datetime as dt


class Cell:
    """ A class to handle a grid cell
        ATTRIBUTES:
            self.s_cost: (number) the distance between the cell and the starting cell
            self.h_cost: (number) the distance between the cell and the target cell (heuristic)
            self.f_cost: (number) the sum of the two previous costs
            self.x: (int) the x index of the cell in the grid
            self.z: (int) the z index of the cell
            self.parent_x: (int) the x index of the parent cell in the grid
            self.parent_z: (int) the z index of the parent cell in the grid
            self.is_obstacle: (boolean) is the cell an obstacle
            self.is_target: (boolean) is the cell the target cell
            self.is_start: (boolean) is the cell the starting cell
            self.is_opened: (boolean) is the cell opened
            self.is_closed: (boolean) is the cell closed
            self.is_path: (boolean) is the cell part of the smallest path
    """
    s_cost: float
    h_cost: float
    f_cost: float
    x: int
    z: int
    parent_x: int
    parent_z: int
    is_obstacle: bool
    is_target: bool
    is_start: bool
    is_opened: bool
    is_closed: bool
    is_path: bool

    def __init__(self):
        """ constructor of the class
        """
        self.s_cost = float("inf")  # the distance between the cell and the starting cell
        self.h_cost = float("inf")  # the distance between the cell and the target cell (heuristic)
        self.f_cost = float("inf")  # the sum of the two previous costs

        # indexes of the cell in the grid
        self.x = 0
        self.z = 0

        # indexes of the parent cell (the one used to compute the costs)
        self.parent_x = -1
        self.parent_z = -1

        # properties of the cell
        self.is_obstacle = False  # is the cell an obstacle
        self.is_target = False  # is the cell the target cell
        self.is_start = False  # is the cell the starting cell

        self.is_opened = False  # is the cell opened
        self.is_closed = False  # is the cell closed
        self.is_path = False  # is the cell part of the smallest path


class ShortestPathUI:
    """ class to handle the user interface and everything actually...
    """

    def __init__(self):
        """ constructor of the class
        """
        self.nb_x = 11  # number of columns in the grid
        self.nb_z = 6  # number of rows in the grid

        # values used by the radio buttons
        self.dijkstra_id = 1
        self.a_star_id = 2
        self.weighted_a_star_id = 3

        # the weight considered of the weighted A*
        self.heuristic_weight = 10

        self.screen = tk.Tk()  # the main window
        self.screen.title("Mobile robotics - User interface to understand A*")  # define the window name
        self.screen.geometry("900x500+100+00")  # define the default window geometry (size and position)

        self.screen.columnconfigure(2, weight=1)
        self.screen.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(self.screen, background='white')  # the area to draw the grid
        self.canvas.grid(columnspan=3, column=0, row=0, sticky=tk.N+tk.S+tk.E+tk.W)  # add the canvas to the window

        # handling the radio buttons
        self.algorithm_var = tk.IntVar()
        self.rb_astar = tk.Radiobutton(self.screen, text="A*", variable=self.algorithm_var, value=self.a_star_id,
                                       command=self.rb_changed)
        self.rb_astar.grid(column=0, row=1, sticky=tk.W)
        self.rb_astar.select()
        self.rb_dijkstra = tk.Radiobutton(self.screen, text="Dijkstra", variable=self.algorithm_var,
                                          value=self.dijkstra_id, command=self.rb_changed)
        self.rb_dijkstra.grid(column=1, row=1, sticky=tk.W)
        self.rb_weighted_a_star = tk.Radiobutton(self.screen, text="Weighted A*", variable=self.algorithm_var,
                                                 value=self.weighted_a_star_id, command=self.rb_changed)
        self.rb_weighted_a_star.grid(column=2, row=1, sticky=tk.W)

        self.screen.bind("<Configure>", self.resize)  # event called when the window is resized
        # to handle the event over the mouse buttons
        self.screen.bind("<Button-1>", self.left_click)
        self.screen.bind("<Button-2>", self.middle_click)
        self.screen.bind("<Button-3>", self.right_click)
        # to handle the event over the mouse when pressing the ctrl key at the same time
        self.screen.bind("<Control-Button-3>", self.ctrl_right_click)
        self.screen.bind("<Control-Button-1>", self.ctrl_left_click)
        # to handle some events from the keyboard
        self.screen.bind('<i>', self.i_key)
        self.screen.bind('<s>', self.s_key)
        self.screen.bind('<F1>', self.f1_key)

        # to add a menu bar for the "help" and "about" commands
        self.menu_bar = tk.Menu(self.screen)
        self.menu_bar.add_command(label="Help", command=self.show_help_message)
        self.menu_bar.add_command(label="About", command=self.show_about_message)
        self.screen.config(menu=self.menu_bar)  # the menu is added to the main window

        # variables for the different colors
        self.start_color = "#008080"  # color of the staring cell
        self.end_color = "#008080"  # color of the target cell
        self.obstacle_color = "#010101"  # color the obstacle cells
        self.empty_color = "white"  # color of the non initialized cell
        self.opened_color = "#AFE9AF"  # color of the opened cells
        self.closed_color = "#DE8787"  # color of the closed cells
        self.path_color = "#008080"  # color of the cells that are part of the path
        self.grid_color = "black"  # color of the grid
        self.parent_color = "white"  # color of the dot that indicates the parent cell
        self.smallest_color = "red"  # color of the border of the smallest cell (the next one to test)

        # initialization of the grid
        self.grid = [[Cell()]]
        self.grid.clear()
        for z in range(0, self.nb_z):
            self.grid.append([])
            for x in range(0, self.nb_x):
                c = Cell()
                c.z = z
                c.x = x
                self.grid[z].append(c)

        # initialization of the target cell coordinates
        self.x_target = 4
        self.z_target = 1
        # initialization of the staring cell coordinates
        self.x_start = 7
        self.z_start = 4
        # initialization of the smallest cell index
        self.x_smallest = -1
        self.z_smallest = -1

        self.init()  # function that initializes all the cells of the grid

        # to draw the cell at the correct size according to the window size
        self.x_size = 0  # the x size of a cell
        self.z_size = 0  # the z size of a cell

        # defining some font size that will be used in the later
        self.font_size_1 = 0
        self.font_size_2 = 0

        self.point_size = 0  # the size of the point that indicates the parent cell
        self.border_width = 4  # border width of the cells (the size of the grid lines)

        self.draw()  # to draw the grid

        self.screen.mainloop()  # main loop of the main window

    def rb_changed(self):
        """ function called when a radio button is selected """
        self.init()  # the interface is reset
        self.draw()  # the display is updated

    def show_help_message(self):
        """ function to display the help window
            it creates the window and all the text/drawing inside
        """

        box_size = 70  # size of the boxes for the drawing

        # creating a new window to display the help
        help_window = tk.Toplevel()
        help_window.geometry("730x750+100+00")  # defining the geometry of the window (size and position)
        help_window.title("Help")  # define the window title
        i = 0  # the i index is used to easily modify the items, is allows to track the current line of the window

        # the detail of the mouse and keyboard commands

        tk.Label(help_window, text="Commands", font='Arial 12 bold').grid(columnspan=8, row=i, column=0)
        i += 1
        tk.Label(help_window, text="Mouse", font='Arial 11 bold').grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - Left click to evaluate a cell").grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - Right click to add/remove an obstacle").grid(columnspan=8, row=i, column=0,
                                                                                    sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - Middle click to generate the path when the target is reached"). \
            grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - Ctrl + left click to change the starting cell"). \
            grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - Ctrl + right click to change the target cell"). \
            grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text="Keyboard", font='Arial 11 bold').grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - 'i' key to reinitialize the grid").grid(columnspan=8, row=i,
                                                                               column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - 's' key to save the grid as a ps image").grid(columnspan=8, row=i,
                                                                                     column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - 'F1' key to show this message").grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1

        # the detail of the cell color meanings

        tk.Label(help_window, text="\nCell Colors", font='Arial 12 bold').grid(columnspan=8, row=i, column=0)
        i += 1
        canvas_obs = tk.Canvas(help_window, width=box_size, height=box_size, background=self.obstacle_color,
                               highlightbackground=self.grid_color, highlightthickness=4)
        canvas_obs.grid(row=i, column=0, sticky=tk.W)
        tk.Label(help_window, text="Obstacle cell").grid(row=i, column=1, sticky=tk.W)

        canvas_empty = tk.Canvas(help_window, width=box_size, height=box_size, background=self.empty_color,
                                 highlightbackground=self.grid_color, highlightthickness=4)
        canvas_empty.grid(row=i, column=2, sticky=tk.W)
        tk.Label(help_window, text="Empty cell").grid(row=i, column=3, sticky=tk.W)

        canvas_start = tk.Canvas(help_window, width=box_size, height=box_size, background=self.start_color,
                                 highlightbackground=self.grid_color, highlightthickness=4)
        canvas_start.grid(row=i, column=4, sticky=tk.W)
        canvas_start.create_text(int(box_size / 2), int(box_size / 2), text='A', font="Arial " + str(self.font_size_1),
                                 fill="white")
        tk.Label(help_window, text="Starting cell").grid(row=i, column=5, sticky=tk.W)

        canvas_end = tk.Canvas(help_window, width=box_size, height=box_size, background=self.end_color,
                               highlightbackground=self.grid_color, highlightthickness=4)
        canvas_end.grid(row=i, column=6, sticky=tk.W)
        canvas_end.create_text(int(box_size / 2), int(box_size / 2), text='B', font="Arial " + str(self.font_size_1),
                               fill="white")
        tk.Label(help_window, text="Target cell").grid(row=i, column=7, sticky=tk.W)

        i += 1

        canvas_opened = tk.Canvas(help_window, width=box_size, height=box_size, background=self.opened_color,
                                  highlightbackground=self.grid_color, highlightthickness=4)
        canvas_opened.grid(row=i, column=0, sticky=tk.W)
        tk.Label(help_window, text="Opened cell").grid(row=i, column=1, sticky=tk.W)

        canvas_closed = tk.Canvas(help_window, width=box_size, height=box_size, background=self.closed_color,
                                  highlightbackground=self.grid_color, highlightthickness=4)
        canvas_closed.grid(row=i, column=2, sticky=tk.W)
        tk.Label(help_window, text="Closed cell").grid(row=i, column=3, sticky=tk.W)

        canvas_closed = tk.Canvas(help_window, width=box_size, height=box_size, background=self.path_color,
                                  highlightbackground=self.grid_color, highlightthickness=4)
        canvas_closed.grid(row=i, column=4, sticky=tk.W)
        tk.Label(help_window, text="Path cell").grid(row=i, column=5, sticky=tk.W)

        canvas_smallest = tk.Canvas(help_window, width=box_size, height=box_size, background=self.opened_color,
                                    highlightbackground=self.smallest_color, highlightthickness=4)
        canvas_smallest.grid(row=i, column=6, sticky=tk.W)
        tk.Label(help_window, text="Smallest opened cell").grid(row=i, column=7, sticky=tk.W)

        i += 1

        # the detail of the numbers computed in the cells

        tk.Label(help_window, text="\nCell Numbers", font='Arial 12 bold').grid(columnspan=8, row=i, column=0)

        i += 1

        canvas_numbers = tk.Canvas(help_window, width=box_size, height=box_size, background=self.opened_color,
                                   highlightbackground=self.grid_color, highlightthickness=4)
        canvas_numbers.grid(rowspan=4, row=i, column=0, sticky=tk.W)
        tk.Label(help_window, text="Opened cell").grid(row=i, column=1, sticky=tk.W)

        f_color = "black"
        g_color = "blue"
        h_color = "red"
        text_x = int(box_size / 2)
        text_z = int(box_size / 2) + box_size / 5
        canvas_numbers.create_text(text_x, text_z, text=str(46),
                                   font="Arial " + str(int(box_size / 3)), fill=f_color)

        text_x = box_size / 5
        text_z = box_size / 5
        canvas_numbers.create_text(text_x, text_z, text=str(10),
                                   font="Arial " + str(int(box_size / 5)), fill=g_color)

        text_x = box_size - box_size / 5
        text_z = box_size / 5
        canvas_numbers.create_text(text_x, text_z, text=str(36),
                                   font="Arial " + str(int(box_size / 5)), fill=h_color)
        tk.Label(help_window, text="10: S cost, distance from starting node", fg=g_color, ). \
            grid(columnspan=5, row=i, column=1, sticky=tk.W)
        tk.Label(help_window, text="36: H cost (heuristic), depends on the considered algorithm", fg=h_color). \
            grid(columnspan=5, row=i + 1, column=1, sticky=tk.W)
        tk.Label(help_window, text="46: F cost = S cost + H cost", fg=f_color).grid(columnspan=5, row=i + 2, column=1,
                                                                                    sticky=tk.W)
        tk.Label(help_window, text="dot: parent cell direction", fg=self.parent_color).grid(columnspan=5, row=i + 3,
                                                                                            column=1, sticky=tk.W)

        canvas_numbers.create_oval(box_size - 10, box_size - 10, box_size, box_size, fill=self.parent_color,
                                   outline=self.parent_color)
        i += 4

        # The possible algorithms to test

        tk.Label(help_window, text="\nAlgorithms", font='Arial 12 bold').grid(columnspan=8, row=i, column=0)
        i += 1
        tk.Label(help_window, text="Optimal path", font='Arial 11 bold').grid(columnspan=8, row=i,
                                                                              column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - A* : the heuristic corresponds to the euclidean distance from the cell to the "
                                   "target").grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - Dijkstra: no heuristic considered (0 for H cost)"). \
            grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text="Non Optimal path", font='Arial 11 bold').grid(columnspan=8, row=i,
                                                                                  column=0, sticky=tk.W)
        i += 1
        tk.Label(help_window, text=" - Weighted A*: the heuristic (euclidean distance) is weighted (w = " + 
                                   str(self.heuristic_weight) + ")").grid(columnspan=8, row=i, column=0, sticky=tk.W)
        i += 1

    def show_about_message(self):
        """ function to show the "about" message box """
        about_message = "User interface to understand A*\n"
        about_detail = "by Remy Guyonneau\nVersion 0.1\nUses Python3 and Tkinter\n\nPolytech Angers" \
                       "\nremy.guyonneau@univ-angers.fr"
        messagebox.showinfo("About", message=about_message, detail=about_detail, parent=self.screen)

    def f1_key(self, _:tk.Event):
        """ function called when pressing the F1 button, it shows the help window
            PARAMETERS:
                _: the key event (not used)
        """
        self.show_help_message()

    def i_key(self, _:tk.Event):
        """ function called when pressing the "i" keyboard key
            PARAMETERS:
                _: the key event (not used)
        """
        self.init()  # initialization of all the cells
        self.draw()  # redraw the grid

    def s_key(self, _:tk.Event):
        """ function called when pressing the "s" keyboard key
            the canvas drawing is saved into a svg image file
            the name of the saved file is the current date
            the image is saved in the current folder
            PARAMETERS:
                _: the key event, not used
        """
        d = dt.datetime.now()
        t = d.timetuple()
        # the file name format : astar_YYYYMMDD_HHMMSS.ps
        name = "astar_" + str(t[0]) + str(t[1]) + str(t[2]) + "_" + str(t[3]) + str(t[4]) + str(t[5]) + ".ps"
        self.canvas.postscript(file=name, colormode='color')
        messagebox.showinfo("Image saved", message="Image saved in current folder", detail=name)

    def init(self):
        """ this function initializes the cells of the grid (to reset the interface for instance)
            the target, the start and the obstacles are not modified
        """
        for z in self.grid:
            for c in z:
                # all the cost are initialized to inf values
                c.s_cost = float("inf")
                c.h_cost = float("inf")
                c.f_cost = float("inf")
                # initialization of the cell parents
                c.parent_x = -1
                c.parent_z = -1
                # initialization of the cell properties
                c.is_opened = False
                c.is_closed = False
                c.is_path = False
        # reset the smallest cell
        self.x_smallest = -1
        self.z_smallest = -1

        # reset the target and the starting cells
        self.grid[self.z_target][self.x_target].is_target = True
        self.grid[self.z_start][self.x_start].is_start = True

        # reset the costs of the stating cell
        self.grid[self.z_start][self.x_start].s_cost = 0
        self.grid[self.z_start][self.x_start].h_cost = ShortestPathUI.distance_x_10(self.x_target, self.z_target,
                                                                                    self.x_start, self.z_start)
        s_cost = self.grid[self.z_start][self.x_start].s_cost
        h_cost = self.grid[self.z_start][self.x_start].h_cost
        self.grid[self.z_start][self.x_start].f_cost = s_cost + h_cost

    def ctrl_right_click(self, event:tk.Event) -> tk.Event:
        """ this function is called when pressing the ctrl keyboard button and while doing a mouse right click
            It changes the target point
            PARAMETERS:
                event: the click event
            RETURN:
                the click event if it was not processed
        """
        if event.widget != self.canvas:
            # if the click is not done over the canvas
            return event

        # get the indexes of the clicked cell
        x = int(event.x / self.x_size)
        z = int(event.y / self.z_size)

        # remove the current target
        self.grid[self.z_target][self.x_target].is_target = False

        # update the new target with the clicked coordinates
        self.x_target = x
        self.z_target = z
        self.grid[z][x].is_target = True

        # reset the grid and redraw it
        self.init()
        self.draw()
        return event

    def ctrl_left_click(self, event:tk.Event) -> tk.Event:
        """ this function is called when pressing the ctrl keyboard button and while doing a mouse left click
            It changes the starting point
            PARAMETERS:
                event: the click event
            RETURN:
                the click event if it was not processed
        """
        if event.widget != self.canvas:
            # if the click is not done over the canvas
            return event

        # get the indexes of the clicked cell
        x = int(event.x / self.x_size)
        z = int(event.y / self.z_size)

        # remove the current starting cell
        self.grid[self.z_start][self.x_start].is_start = False

        # update the new starting cell with the clicked coordinates
        self.x_start = x
        self.z_start = z
        self.grid[z][x].is_start = True

        # reset the grid cell and redraw the grid
        self.init()
        self.draw()
        return event

    def left_click(self, event:tk.Event) -> tk.Event:
        """ This function is called when doing a mouse left click (without pressing ctrl button)
            It updates the clicked cell
            PARAMETERS:
                 event: the click event
             RETURN:
                the click event if it was not processed
        """
        if event.widget != self.canvas:
            # if the click is not done over the canvas
            return event

        # get the coordinates of the clicked cell
        x = int(event.x / self.x_size)
        z = int(event.y / self.z_size)

        # when a cell is selected, it should be tagged as closed (A* algorithm)
        self.grid[z][x].is_closed = True
        self.grid[z][x].is_opened = False

        # update all the cells around the selected one
        self.update_cell(x, z, 1, 1)
        self.update_cell(x, z, 1, 0)
        self.update_cell(x, z, 1, -1)
        self.update_cell(x, z, 0, 1)
        self.update_cell(x, z, 0, -1)
        self.update_cell(x, z, -1, -1)
        self.update_cell(x, z, -1, 1)
        self.update_cell(x, z, -1, 0)

        # from those new computed cells, it is needed to look for the new smallest one
        self.update_smallest()

        # redraw the grid
        self.draw()
        return event

    def right_click(self, event:tk.Event) -> tk.Event:
        """ This function is called when doing a mouse right click (without pressing ctrl button)
            it adds/removes an obstacle
            PARAMETERS:
                event: the click event
            RETURN:
                the click event if it was not processed
        """
        if event.widget != self.canvas:
            # if the click is not done over the canvas
            return event

        # get the coordinates of the clicked cell
        x = int(event.x / self.x_size)
        z = int(event.y / self.z_size)

        # the target and the starting cells should not be obstacles
        if (z != self.z_target or x != self.x_target) and (x != self.x_start or z != self.z_start):
            # if the cell is an obstacle, remove the obstacle
            # otherwise, add the new obstacle
            self.grid[z][x].is_obstacle = not self.grid[z][x].is_obstacle
            self.init()
        # redraw the grid
        self.draw()
        return event

    def middle_click(self, event:tk.Event) -> tk.Event:
        """ this function is called when doing a mouse middle-click
            the idea is to update the path, if the target has been found
            PARAMETERS:
                event: the click event
            RETURN:
                the click event if it was not processed
        """
        if event.widget != self.canvas:
            # if the click is not done over the canvas
            return event

        if self.grid[self.z_target][self.x_target].parent_x >= 0 and \
                self.grid[self.z_target][self.x_target].parent_z >= 0:
            # if the target has been found (the target cell has a parent)
            # get the coordinates of the target parent
            curr_x = self.grid[self.z_target][self.x_target].parent_x
            curr_z = self.grid[self.z_target][self.x_target].parent_z
            # while the starting cell is not reached
            while self.grid[curr_z][curr_x].parent_x >= 0 and self.grid[curr_z][curr_x].parent_z >= 0:
                # update the current cell as part of the path
                self.grid[curr_z][curr_x].is_path = True
                # get the new parent coordinates
                old_x = curr_x
                old_z = curr_z
                curr_x = self.grid[old_z][old_x].parent_x
                curr_z = self.grid[old_z][old_x].parent_z
            # when going out of the while, the starting cell should have been reached
        # redraw the grid
        self.draw()
        return event

    def update_cell(self, x:int, z:int, dx:float, dz:float):
        """ function to update a cell
            The coordinates of the cell to update are (x+dx, z+dz)
            PARAMETERS:
                x: (int) the parent x coordinate (index in the grid)
                z: (int) the parent z coordinate (index in the grid)
                dx: (int {-1, 0, 1}) the x delta for the cell to update
                dz: (int {-1, 0, 1}) the z delta for the cell to update
        """
        if (0 <= x + dx < self.nb_x) and (0 <= z + dz < self.nb_z):
            # if the cell is in the grid
            if not self.grid[z + dz][x + dx].is_obstacle and not self.grid[z + dz][x + dx].is_start:
                # if the cell is not an obstacle, neither the starting cell
                dst = 14  # the diagonal distance
                if dx == 0 or dz == 0:  # if th parent is not at the diagonal
                    dst = 10  # the non diagonal distance
                # the s_cost: distance to the starting node
                s_cost = self.grid[z][x].s_cost + dst  # compute the s_cost according to the parent
                # compute the h_cost (distance to the target) according to the heuristic function
                # in this case the heuristic function is the euclidean distance
                h_cost = self.heuristic(self.grid[z+dz][x+dx])
                # the f_cost is the sum of the s_cost and the h_cost
                f_cost = s_cost + h_cost
                # tagged the cell as opened
                self.grid[z + dz][x + dx].is_opened = True
                # test if the new computed cost is better than the previous one (infinity if not opened yet...)
                if f_cost < self.grid[z + dz][x + dx].f_cost:
                    # update the cell with this new costs
                    self.grid[z + dz][x + dx].s_cost = s_cost
                    self.grid[z + dz][x + dx].h_cost = h_cost
                    self.grid[z + dz][x + dx].f_cost = f_cost
                    # update the cell parent
                    self.grid[z + dz][x + dx].parent_x = x
                    self.grid[z + dz][x + dx].parent_z = z

    def resize(self, _:tk.Event):
        """ function called when resizing the window
            it is then needed to adjust the drawing sizes to adapt to this new size
            PARAMETERS:
                _: the resize event, not used
        """

        # the sizes of the x and z values are updated
        self.x_size = self.canvas.winfo_width() / self.nb_x
        self.z_size = self.canvas.winfo_height() / self.nb_z

        # the font sizes are updated
        self.font_size_1 = int(min(self.x_size, self.z_size) / 3)
        self.font_size_2 = int(min(self.x_size, self.z_size) / 5)

        # the size of the dot to indicate the parent direction is updated
        self.point_size = int(min(self.x_size, self.z_size) / 15)

        # and finally the grid is redrawn
        self.draw()

    def draw(self):
        """ main function to draw all the stuff """
        self.canvas.delete("all")  # we start by removing the old display
        self.draw_grid()  # this function draw the cells
        # at the end the smallest cell is drawn
        # it has to be done at this end so the border will not be overlapped by previous cells
        if self.x_smallest >= 0 and self.z_smallest >= 0:
            self.draw_smallest_cell(self.grid[self.z_smallest][self.x_smallest])

    def draw_grid(self):
        """ function to draw all the cells of the grid
            depending on the cell properties, different cell drawing function are called
        """
        for z in self.grid:
            for c in z:
                if c.is_start or c.is_target:
                    # if the cell is the starting cell or the target
                    self.draw_start_target_cell(c)
                elif c.is_path:
                    # if the cell is part of the path
                    self.draw_path_cell(c)
                elif c.is_opened or c.is_closed:
                    # if the cell is opened or is closed
                    self.draw_opened_closed_cell(c)
                else:
                    # draw an empty cell or an obstacle
                    self.draw_cell(c)

    def draw_cell(self, c:Cell):
        """ function to draw an empty cell or an obstacle, only the color differ
            PARAMETERS:
                c: (Cell) the cell to draw
        """
        color_fill = self.empty_color
        if c.is_obstacle:
            color_fill = self.obstacle_color
        color_outline = self.grid_color
        # draw a rectangle according to the cell indexes
        self.canvas.create_rectangle(c.x * self.x_size,
                                     c.z * self.z_size,
                                     (c.x + 1) * self.x_size,
                                     (c.z + 1) * self.z_size,
                                     width=self.border_width,
                                     fill=color_fill, outline=color_outline)

    def create_point(self, x:int, y:int, size:int=1, color:str="#000000"):
        """ function to create a point, this is just overwriting the create_oval function
            PARAMETERS:
                x: (number) the x coordinate of the point in the canvas
                y: (number) the y coordinate of the point in the canvas
                size: (number) the size to draw the point with
                color: (string) the color to draw the point with
        """
        x1, y1 = (x - size), (y - size)
        x2, y2 = (x + size), (y + size)
        self.canvas.create_oval(x1, y1, x2, y2, fill=color, outline=color)

    def draw_opened_closed_cell(self, c:Cell):
        """ function to draw an open or closed cell (just the color differs)
            PARAMETERS:
                c: (Cell) the cell to draw
        """
        color_fill = self.opened_color
        if c.is_closed:
            color_fill = self.closed_color
        color_outline = self.grid_color
        # create the rectangle corresponding to the cell
        self.canvas.create_rectangle(c.x * self.x_size,
                                     c.z * self.z_size,
                                     (c.x + 1) * self.x_size,
                                     (c.z + 1) * self.z_size,
                                     width=self.border_width,
                                     fill=color_fill, outline=color_outline)

        # write the f_cost inside the cell
        text_x = int(self.x_size * (2 * c.x + 1) / 2)
        text_z = int(self.z_size * (2 * c.z + 1) / 2) + self.font_size_2
        self.canvas.create_text(text_x, text_z, text=str(c.f_cost),
                                font="Arial " + str(self.font_size_1), fill="black")

        # write the s_cost inside the cell
        text_x = int(self.x_size * c.x) + self.font_size_2 + self.point_size
        text_z = int(self.z_size * c.z) + self.font_size_2 + self.point_size + self.border_width
        self.canvas.create_text(text_x, text_z, text=str(c.s_cost),
                                font="Arial " + str(self.font_size_2), fill="black")

        # write the h_cost inside the cell
        text_x = int(self.x_size * (c.x + 1)) - self.font_size_2 - self.point_size
        text_z = int(self.z_size * c.z) + self.font_size_2 + self.point_size + self.border_width
        self.canvas.create_text(text_x, text_z, text=str(c.h_cost),
                                font="Arial " + str(self.font_size_2), fill="black")

        # draw the parent of the cell (the point indicating the parent direction)
        self.draw_parent(c, self.parent_color)

    def draw_smallest_cell(self, c:Cell):
        """ function to draw the smallest cell
            this function should be called after drawing all the other cell so the border will not be overlapped
            if corresponds to an open cell with a red border
            PARAMETERS:
                c: (Cell) the cell to draw
        """
        if c.is_target or c.is_start:
            # if the smallest is the starting cell or the target nothing to do
            return
        color_fill = self.opened_color
        color_outline = self.smallest_color
        # create the rectangle corresponding to the cell
        self.canvas.create_rectangle(c.x * self.x_size,
                                     c.z * self.z_size,
                                     (c.x + 1) * self.x_size,
                                     (c.z + 1) * self.z_size,
                                     width=self.border_width,
                                     fill=color_fill, outline=color_outline)
        # write the f cost of the cell
        text_x = int(self.x_size * (2 * c.x + 1) / 2)
        text_z = int(self.z_size * (2 * c.z + 1) / 2) + self.font_size_2
        self.canvas.create_text(text_x, text_z, text=str(c.f_cost),
                                font="Arial " + str(self.font_size_1), fill="black")
        # write the s cost of the cell
        text_x = int(self.x_size * c.x) + self.font_size_2 + self.point_size
        text_z = int(self.z_size * c.z) + self.font_size_2 + self.point_size + self.border_width
        self.canvas.create_text(text_x, text_z, text=str(c.s_cost),
                                font="Arial " + str(self.font_size_2), fill="black")
        # write the h cost of the cell
        text_x = int(self.x_size * (c.x + 1)) - self.font_size_2 - self.point_size
        text_z = int(self.z_size * c.z) + self.font_size_2 + self.point_size + self.border_width
        self.canvas.create_text(text_x, text_z, text=str(c.h_cost),
                                font="Arial " + str(self.font_size_2), fill="black")
        # draw the parent of the cell (the point indicating the parent)
        self.draw_parent(c, self.parent_color)

    def draw_parent(self, c:Cell, point_color:str="black"):
        """ this function draws a point inside a cell, indicating the parent solution
            PARAMETERS:
                c: (Cell) the cell to draw the parent of
                point_color: (string) the color to draw the point with
        """
        point_x = 0
        point_z = 0

        if c.parent_x < 0 or c.parent_z < 0:
            # if the parent of the cell has not yet be initialized, nothing to do
            return

        # test the position of the parent regarding the cell and draw the point to the correct position
        # 8 positions have to be tested
        # N, S, E, W, NW, NE, SW, SE
        if c.parent_x < c.x and c.parent_z < c.z:  # NW
            point_x = c.x * self.x_size + self.point_size + self.border_width
            point_z = c.z * self.z_size + self.point_size + self.border_width
        elif c.parent_x == c.x and c.parent_z < c.z:  # N
            point_x = int(self.x_size * (2 * c.x + 1) / 2)
            point_z = c.z * self.z_size + self.point_size + self.border_width
        elif c.parent_x > c.x and c.parent_z < c.z:  # NE
            point_x = (c.x + 1) * self.x_size - self.point_size - self.border_width
            point_z = c.z * self.z_size + self.point_size + self.border_width
        elif c.parent_x > c.x and c.parent_z == c.z:  # E
            point_x = (c.x + 1) * self.x_size - self.point_size - self.border_width
            point_z = int(self.z_size * (2 * c.z + 1) / 2)
        elif c.parent_x < c.x and c.parent_z == c.z:  # W
            point_x = c.x * self.x_size + self.point_size + self.border_width
            point_z = int(self.z_size * (2 * c.z + 1) / 2)
        elif c.parent_x == c.x and c.parent_z > c.z:  # S
            point_x = int(self.x_size * (2 * c.x + 1) / 2)
            point_z = (c.z + 1) * self.z_size - self.point_size - self.border_width
        elif c.parent_x > c.x and c.parent_z > c.z:  # SE
            point_x = (c.x + 1) * self.x_size - self.point_size - self.border_width
            point_z = (c.z + 1) * self.z_size - self.point_size - self.border_width
        elif c.parent_x < c.x and c.parent_z > c.z:  # SW
            point_x = c.x * self.x_size + self.point_size + self.border_width
            point_z = (c.z + 1) * self.z_size - self.point_size - self.border_width

        # draw the point at the computed position
        self.create_point(point_x, point_z, self.point_size, point_color)

    def draw_start_target_cell(self, c:Cell):
        """ function to draw the starting cell or the target cell
            PARAMETERS:
                 c: (Cell) the cell to draw
        """
        color_fill = self.start_color
        color_outline = self.grid_color
        text = 'A'  # the starting cell has the letter A
        if c.is_target:
            text = 'B'  # the target has the letter B
            color_fill = self.end_color
        # create the rectangle corresponding to the cell
        self.canvas.create_rectangle(c.x * self.x_size,
                                     c.z * self.z_size,
                                     (c.x + 1) * self.x_size,
                                     (c.z + 1) * self.z_size,
                                     width=4,
                                     fill=color_fill, outline=color_outline)
        # draw the letter inside the rectangle
        text_x = int(self.x_size * (2 * c.x + 1) / 2)
        text_z = int(self.z_size * (2 * c.z + 1) / 2)
        self.canvas.create_text(text_x, text_z, text=text,
                                font="Arial " + str(self.font_size_1), fill="white")
        # draw the parent of the cell (effective for the target only)
        self.draw_parent(c, self.parent_color)

    def draw_path_cell(self, c:Cell):
        """ function to draw all the cells that are part of the path
            it is the same as the draw_opened_closed_cell, should be modified is new version
            PARAMETERS:
                c: (Cell) the cell to draw
        """
        color_fill = self.path_color
        color_outline = self.grid_color
        self.canvas.create_rectangle(c.x * self.x_size,
                                     c.z * self.z_size,
                                     (c.x + 1) * self.x_size,
                                     (c.z + 1) * self.z_size,
                                     width=4,
                                     fill=color_fill, outline=color_outline)

        text_x = int(self.x_size * (2 * c.x + 1) / 2)
        text_z = int(self.z_size * (2 * c.z + 1) / 2) + self.font_size_2
        self.canvas.create_text(text_x, text_z, text=str(c.f_cost),
                                font="Arial " + str(self.font_size_1), fill="white")

        text_x = int(self.x_size * c.x) + self.font_size_2 + self.point_size
        text_z = int(self.z_size * c.z) + self.font_size_2 + self.point_size + self.border_width
        self.canvas.create_text(text_x, text_z, text=str(c.s_cost),
                                font="Arial " + str(self.font_size_2), fill="white")

        text_x = int(self.x_size * (c.x + 1)) - self.font_size_2 - self.point_size
        text_z = int(self.z_size * c.z) + self.font_size_2 + self.point_size + self.border_width
        self.canvas.create_text(text_x, text_z, text=str(c.h_cost),
                                font="Arial " + str(self.font_size_2), fill="white")

        self.draw_parent(c, self.parent_color)

    def update_smallest(self):
        """ function that updates the smallest opened cell, to be able to display it
        """
        smallest_f = float('inf')
        smallest_s = float('inf')
        self.x_smallest = -1
        self.z_smallest = -1
        # loop over all the cells and search for the smallest cell
        for z in self.grid:
            for c in z:
                if not c.is_closed and (c.f_cost == smallest_f and c.s_cost < smallest_s or c.f_cost < smallest_f):
                    smallest_f = c.f_cost
                    smallest_s = c.s_cost
                    self.z_smallest = c.z
                    self.x_smallest = c.x

    def heuristic(self, c:Cell):
        """ This functions allows to compute the heuristic (depending on the chosen algorithm) for a given cell
            PARAMETERS:
                c: (Cell) the cell to compute the heuristic
            RETURN:
                the considered heuristic (number, depending on the chosen algorithm)
        """
        if self.algorithm_var.get() == self.a_star_id:
            # A* : the heuristic is the euclidean distance between the cell and the target
            return ShortestPathUI.distance_x_10(c.x, c.z, self.x_target, self.z_target)
        elif self.algorithm_var.get() == self.dijkstra_id:
            # Dijkstra : the heuristic is 0 (no heuristic, only the distance to the starting cell)
            return 0
            # weighted A* : the A* heuristic is multiplied by a weight
        elif self.algorithm_var.get() == self.weighted_a_star_id:
            return ShortestPathUI.distance_x_10(c.x, c.z, self.x_target, self.z_target) * self.heuristic_weight

    @staticmethod
    def distance_x_10(x1:float, z1:float, x2:float, z2:float) -> float:
        """ This function returns the euclidean distance times 10:
            PARAMETERS:
                x1: (number) the x value of the point 1
                z1: (number) the z value of the point 1
                x2: (number) the x value of the point 2
                z2: (number) the z value of the point 2
            RETURN:
                the distance between the points 1 and 2, times 10 (number)
        """
        return int(sqrt(pow(x1 - x2, 2) + pow(z1 - z2, 2)) * 10)


# To start the user interface
if __name__ == "__main__":
    ui = ShortestPathUI()
