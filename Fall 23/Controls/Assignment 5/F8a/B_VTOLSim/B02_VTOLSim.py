import sys
sys.path.append('.')
sys.path.append('C:/Users/spenc/Documents/School/Fall 23/Controls/Assignment 5/F8a')
import matplotlib.pyplot as plt
import numpy as np
import parameters.VTOLParam as P
from tools.signalGenerator import signalGenerator
from viewer.VTOLAnimation import VTOLAnimation
from viewer.dataPlotter import dataPlotter
from dynamics.VTOLDynamics import VTOLDynamics
from controller.VTOLcontroller import controller

# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
zRef = signalGenerator(amplitude=0.5, frequency=0.1)
fRef = P.f0

# instantiate the simulation plots and animation
vtol=VTOLDynamics(alpha=0.0)
# dataPlot = dataPlotter()
animation = VTOLAnimation(limits=10, multi=True)
control=controller()

# inital values
z=0
z_target=0
u=0

# plotting
zplt = animation.fig.add_subplot(2, 2, 2)
zplt.set_ylabel('z (m)')
fplt = animation.fig.add_subplot(2, 2, 4)
fplt.set_ylabel('Force (N)')

# initialize lists to plot
simtimes=[]
fs=[]
zs=[]
targets=[]

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    if t <= 1:
        z_target = 0.0
    elif t <= 25:
        z_target = 5
    elif t <= 45:
        z_target = 7
    elif t <= 70:
        z_target = 1
    elif t <= 90:
        z_target = 7
    else:
        z_target = 1

    # update animation
    f=control.update(z_target,vtol.state)
    y=vtol.update(f/2.,f/2.)
    t = t + P.Ts  
    animation.update(vtol.state)

    simtimes.append(t)
    fs.append(f)
    zs.append(y[1][0])
    targets.append(z_target)  

    # Plot
    zplt.clear()
    zplt.plot(simtimes,zs,label="Position")
    zplt.plot(simtimes,targets, label="Target")
    zplt.set_ylabel("Position (m)")
    zplt.legend(loc="upper left")
    zplt.grid()
    fplt.clear()
    fplt.plot(simtimes,fs)
    fplt.set_ylabel("Force (N)")
    fplt.grid()

    plt.pause(0.01)  # allow time for animation to draw
plt.close()
