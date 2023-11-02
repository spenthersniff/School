import sys
sys.path.append('.')
sys.path.append('C:/Users/spenc/Documents/School/Fall 23/Controls/Assignment 5/Mass Spring Damper2')
import matplotlib.pyplot as plt
import numpy as np
import parameters.MSDParam as P
from tools.signalGenerator import signalGenerator
from viewer.MSDAnimation import MSDAnimation
from viewer.dataPlotter import dataPlotter
from dynamics.MSDDynamics import MSDDynamics
from Controller.MSDController import controller
from Controller.PIDController import PIDControl

# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
control=controller()
Pid = PIDControl(Fmax=6, sigma=0.05, flag=False)

# instantiate the simulation plots and animation
msd=MSDDynamics(alpha=0.0)
animation = MSDAnimation(limits=2,multi=True)

# inital values
z=0
z_target=1
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
z_target = 1

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    t_next_plot = t + P.t_plot
    while t < t_next_plot:    
    
        u = Pid.update(z_target, msd.h()[0])
        z=msd.update(u)
        t = t + P.Ts  
    simtimes.append(t)
    fs.append(u)
    zs.append(z[0])
    targets.append(z_target)

    # update animation 
    
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
