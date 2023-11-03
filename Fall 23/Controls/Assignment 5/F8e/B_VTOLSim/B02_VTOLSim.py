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
vtol=VTOLDynamics()
# dataPlot = dataPlotter()
animation = VTOLAnimation(limits=10, multi=True)
control=controller()

# inital values
z=0
z_target=3
u=0
h = 5.0


# plotting
zplt = animation.fig.add_subplot(4, 2, 2)
zplt.set_ylabel('z Location (m)')
fplt = animation.fig.add_subplot(4, 2, 4)
fplt.set_ylabel('Force (N)')
hplt = animation.fig.add_subplot(4, 2, 6)
hplt.set_ylabel('Height (m)')
thetaplt = animation.fig.add_subplot(4, 2, 8)
thetaplt.set_ylabel('Theta (deg)')

# initialize lists to plot
simtimes=[]
frs=[]
fls=[]
zs=[]
hs=[]
thetas=[]
z_targets=[]
h_targets=[]


t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        
        # z_target = 3 + z_signal.square(t)
        z_target = 3
        h_target=5.0
    
        fr, fl=control.update(z_target, h_target, vtol.state)
        y=vtol.update(fr,fl)
        t = t + P.Ts
    animation.update(vtol.state)

    simtimes.append(t)
    frs.append(fr)
    fls.append(fl)
    zs.append(y[0][0])
    hs.append(y[1][0])
    thetas.append(y[2][0])

    z_targets.append(z_target) 
    h_targets.append(h_target) 

    # Plot
    zplt.clear()
    zplt.plot(simtimes,zs,label="z Position")
    zplt.plot(simtimes,z_targets, label="z Target")
    zplt.set_ylabel("z Position (m)")
    zplt.legend(loc="upper left")
    zplt.grid()
    fplt.clear()
    fplt.plot(simtimes,frs,label="Fr")
    fplt.plot(simtimes,fls,label="Fl")
    fplt.legend(loc="upper left")
    fplt.set_ylabel("Force (N)")
    fplt.grid()
    hplt.clear()
    hplt.plot(simtimes,hs,label="Height")
    hplt.plot(simtimes,h_targets, label="h Target")
    hplt.set_ylabel("Height (m)")
    hplt.grid()
    thetaplt.clear()
    thetaplt.plot(simtimes,np.degrees(thetas))
    thetaplt.set_xlabel("Time (s)")
    thetaplt.set_ylabel("Theta (deg)")
    thetaplt.grid()

    plt.pause(0.01)  # allow time for animation to draw
plt.close()
