from matplotlib import pyplot as plt
from matplotlib import patches as mpatches
import numpy as np 
import parameters.VTOLParam as P
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


class VTOLAnimation:
    def __init__(self, limits, multi):
        # set up plot
        self.fig = plt.figure(1)
        if multi == True:
            self.ax = self.fig.add_subplot(1, 2, 1)
        else:
            self.ax = self.fig.add_subplot(1, 1, 1)
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
        h = state[1][0]
        theta = state[2][0]
        # draw plot elements: body, Lprop, Rprop
        self.draw_body(z, h, theta)
        self.draw_Lprop(z, h, theta)
        self.draw_Rprop(z, h,theta)
        self.ax.set_aspect('equal')
        self.ax.set_ylim(bottom=-3,top=8)
        self.ax.set_xlim(left=-6,right=6)
        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False
    
    def draw_body(self, z, h, theta):
        # specify bottom left corner of rectangle
        x = z-P.wb/2.0
        y = h-P.hb/2
        corner = (x, y)
        # create rectangle on first call, update on subsequent calls
        if self.flag_init is True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.Rectangle(corner, P.wb, P.hb, fc='blue', ec='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[0])
        else:
            self.handle[0].set_xy(corner)  # Update patch

    def draw_Lprop(self, z, h, theta):
        # specify bottom left corner of rectangle
        x = [z-((P.wb/2)+(P.d*np.cos(-theta)))]
        y = [h+(P.d*np.sin(-theta))]
        center = (x, y)
        # create circle on first call, update on subsequent calls
        if self.flag_init is True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(
                mpatches.CirclePolygon(center, radius=0.05, resolution=15, fc='blue', ec='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[1])
        else:
            self.handle[1].xy=center  # Update patch
    
    def draw_Rprop(self, z, h, theta):
       # specify bottom left corner of rectangle
        x = [z+((P.wb/2)+(P.d*np.cos(-theta)))]
        y = [h-(P.d*np.sin(-theta))]
        center = (x, y)
        # create circle on first call, update on subsequent calls
        if self.flag_init is True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            self.handle.append(mpatches.CirclePolygon(center, radius=0.05, resolution=15, fc='blue', ec='black'))
            # Add the patch to the axes
            self.ax.add_patch(self.handle[2])
        else:
            self.handle[2].xy=center  # Update patch
   