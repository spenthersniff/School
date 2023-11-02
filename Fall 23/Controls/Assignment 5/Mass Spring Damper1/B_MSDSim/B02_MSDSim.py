import sys
sys.path.append('.')
sys.path.append('C:/Users/spenc/Documents/School/Fall 23/Controls/Assignment 5/Mass Spring Damper1')
import matplotlib.pyplot as plt
import numpy as np
import parameters.MSDParam as P
from tools.signalGenerator import signalGenerator
from viewer.MSDAnimation import MSDAnimation
from viewer.dataPlotter import dataPlotter
from dynamics.MSDDynamics import MSDDynamics
from Controller.MSDController import controller

# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
control=controller()

# instantiate the simulation plots and animation
msd=MSDDynamics(alpha=0.0)
animation = MSDAnimation(limits=2,multi=True)

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
    # set variables
    if t <= 1:
        z_target = 0
    elif t <= 15:
        z_target = 0.5
    elif t <= 30:
        z_target = 1.0
    elif t <= 45:
        z_target = 1.5
    elif t <= 60:
        z_target = 0.5
    else:
        z_target = 0
       
    
    # r = reference.square(t)
    u = control.update(z_target, msd.state)
    z=msd.update(u)

    simtimes.append(t)
    fs.append(u)
    zs.append(z[0])
    targets.append(z_target)

    # update animation
    t = t + P.Ts  
    animation.update(msd.state)

    # Plot
    zplt.clear()
    zplt.plot(simtimes,zs,label="Position")
    zplt.plot(simtimes,targets, label="Target")
    zplt.set_ylabel("Position (m)")
    zplt.legend(loc="upper right")
    zplt.grid()
    fplt.clear()
    fplt.plot(simtimes,fs)
    fplt.set_ylabel("Force (N)")
    fplt.grid()

    plt.pause(0.01)  # allow time for animation to draw

plt.close()
