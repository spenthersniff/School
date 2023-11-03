import sys
sys.path.append('.')
sys.path.append('C:/Users/spenc/Documents/School/Fall 23/Controls/Assignment 5/F8e')
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
# fRef = P.f0
z_signal = signalGenerator(amplitude=2.5, frequency=0.08)

# instantiate the simulation plots and animation
vtol=VTOLDynamics(alpha=0.0)
# dataPlot = dataPlotter()
animation = VTOLAnimation(limits=10, multi=True)
control=controller()

# inital values
z=0
z_target=3
u=0
h = 5.0


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
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        
        z_target = 3 + z_signal.square(t)
        h=5.0
    # update animation
        fr, fl=control.update(z_target, h, vtol.state)
        y=vtol.update(fr,fl)
        t = t + P.Ts  
    animation.update(vtol.state)

    simtimes.append(t)
    fs.append(fr)
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
