from matplotlib import pyplot as plt
from matplotlib import patches as mpatches
import numpy as np 
import parameters.MSDParam as P
# if you are having difficulty with the graphics, 
# try using one of the following backends  
# See https://matplotlib.org/stable/users/explain/backends.html
# import matplotlib
# matplotlib.use('qtagg')  # requires pyqt or pyside
# matplotlib.use('ipympl')  # requires ipympl
# matplotlib.use('gtk3agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk4agg')  # requires pyGObject and pycairo
# matplotlib.use('gtk3cairo')  # requires pyGObject and pycairo
# matplotlib.use('gtk4cairo')  # requires pyGObject and pycairo
# matplotlib.use('tkagg')  # requires TkInter
# matplotlib.use('wxagg')  # requires wxPython


class MSDAnimation:
    def __init__(self,limits,multi):
       # set up plot
        self.fig = plt.figure(1)
        if multi == True:
            self.ax = self.fig.add_subplot(1, 2, 1)
        else:
            self.ax = self.fig.add_subplot(1, 1, 1)
        self.ax.set_xlim(-limits, limits)
        # draw ground
        plt.plot([-limits, limits], [0, 0])     
        # for list of objects
        self.handle = []
        # init flag
        self.flag_init = True
        # set limits
        self.limits = limits

    def update(self, state):
        z = state[0][0]  # Horizontal position of cart, m
        # draw plot elements: cart, bob, rod
        self.draw_block(z)
        self.ax.axis('equal')
        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False

    def draw_block(self, z):
        # specify bottom left corner of rectangle
        x = z-P.w/2.0
        y = P.gap
        corner = (x, y)
        # create rectangle on first call, update on subsequent calls
        if self.flag_init is True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(
                mpatches.Rectangle(corner, P.w, P.h, fc='blue', ec='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[0])
        else:
            self.handle[0].set_xy(corner)  # Update patch
