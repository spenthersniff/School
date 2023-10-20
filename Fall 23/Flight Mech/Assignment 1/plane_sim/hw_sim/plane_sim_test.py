
import sys
sys.path.append('C:/Users/spenc/OneDrive - University of Cincinnati/2023 Fall/Flight Mechanics/Python/plane_sim')# one directory up
import numpy as np
from math import cos, sin, tan
import scipy.linalg as linalg
import parameters.simulation_parameters as SIM
from viewers.plane_animation import plane_animation
from tools.signalGenerator import signalGenerator

state=np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
plane_anim=plane_animation(state, scale=5)

# create subplots
angles=plane_anim.fig.add_subplot(2,2,2)
translations=plane_anim.fig.add_subplot(2,2,4)

# initialize variables for append
simtime = []
phis = []
thetas = []
psis = []
pns = []
pes = []
pds = []
   
temp = signalGenerator(amplitude=np.radians(60), frequency=1)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
pn=state[0,0]
pe=state[1,0]
pd=state[2,0]
phi=state[6,0]
theta=state[7,0]
psi=state[8,0]
while sim_time < SIM.end_time:
    
    # perform each movement for 1 second
    if sim_time<=1:
        phi=temp.sin(sim_time)
    elif 1<sim_time<=2:
        theta=temp.sin(sim_time)
    elif 2<sim_time<=3:
        psi=temp.sin(sim_time)
    elif 3<sim_time<=4:
        pn=temp.sin(sim_time)
    elif 4<sim_time<=5:
        pe=temp.sin(sim_time)
    elif 5<sim_time<=6:
        pd=temp.sin(sim_time)

    # store data in lists from earlier
    simtime.append(sim_time)
    phis.append(phi)
    thetas.append(theta)
    psis.append(psi)
    pns.append(pn)
    pes.append(pe)
    pds.append(pd)

    # plot data on subplots
    angles.clear()
    translations.clear()
    angles.plot(simtime, np.rad2deg(phis), "r-", label="$\phi$")
    angles.plot(simtime, np.rad2deg(thetas), "g-", label="$\ttheta$")
    angles.plot(simtime, np.rad2deg(psis), "b-", label="$\psi$")
    translations.plot(simtime, pns, "m-", label="North")
    translations.plot(simtime, pes, "y-", label="East")
    translations.plot(simtime, pds, "k-", label="Down")
    angles.grid('major')
    angles.set_xlabel('Times (s)')
    angles.set_ylabel('Angle (degrees)')
    angles.legend(loc='upper right')
    translations.grid('major')
    translations.set_xlabel('Times (s)')
    translations.set_ylabel('Distance (m)')
    translations.legend(loc='upper right')

    plane_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height


    # -------increment time-------------
    sim_time += SIM.ts_simulation