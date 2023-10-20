import sys
sys.path.append('C:/Users/spenc/OneDrive - University of Cincinnati/2023 Fall/Flight Mechanics/Python/plane_sim2')# one directory up
import numpy as np
from math import cos, sin, tan
import scipy.linalg as linalg
import parameters.aerosonde_parameters as SIM
from viewers.plane_animation import plane_animation
from tools.signalGenerator import signalGenerator
from dynamics.PlaneDynamics import MAVDynamics

state=SIM.states0
plane_anim=plane_animation(state, scale=5)
uav=MAVDynamics(alpha=0.0)

# create subplots
angles=plane_anim.fig.add_subplot(5,2,9)
translations=plane_anim.fig.add_subplot(5,2,2)
anglerate=plane_anim.fig.add_subplot(5,2,10)
translationrate=plane_anim.fig.add_subplot(5,2,6)

# generate forces and moments
force = signalGenerator(amplitude=1250, frequency=2)
moment = signalGenerator(amplitude=10, frequency=2)


# initialize the simulation time
sim_time = SIM.start_time

# initialize variables for append
simtime = []
fxs = []
fys = []
fzs = []
ls = []
ms = []
ns = []
pns = []
pes = []
pds = []
us = []
vs = []
ws = []
phis = []
thetas = []
psis = []
ps = []
qs = []
rs = []
   

# main simulation loop
print("Press Command-Q to exit...")
pn = state[0,0]
pe = state[1,0]
pd = state[2,0]
u = state[3,0]
v = state[4,0]
w = state[5,0]
phi = state[6,0]
theta = state[7,0]
psi = state[8,0]
p = state[9,0]
q = state[10,0]
r = state[11,0]
    
while sim_time < SIM.end_time:
    
    # perform each movement for 1 second
    if sim_time<=0.5:
        fx=force.sin(sim_time)
        fy=0
        fz=0
        l=0
        m=0
        n=0
    elif sim_time<=1:
        fx=0
        fy=force.sin(sim_time)
    elif sim_time<=1.5:
        fy=0
        fz=force.sin(sim_time)
    elif sim_time<=2:
        fz=0
        l=moment.sin(sim_time)
    elif sim_time<=2.5:
        l=0
        m=moment.sin(sim_time)
    elif sim_time<=3:
        m=0
        n=moment.sin(sim_time)
    else:
        n=0
   
    y=uav.update(fx, fy, fz, l, m, n)
    plane_anim.update(y[0][0], y[1][0], y[2][0], y[6][0], y[7][0], y[8][0])
   
    # store data in lists from earlier
    simtime.append(sim_time)
    fxs.append(fx)
    fys.append(fy)
    fzs.append(fz)
    ls.append(l)
    ms.append(m)
    ns.append(n)

    pns.append(y[0][0])
    pes.append(y[1][0])
    pds.append(y[2][0])
    us.append(y[3][0])
    vs.append(y[4][0])
    ws.append(y[5][0])
    phis.append(y[6][0])
    thetas.append(y[7][0])
    psis.append(y[8][0])
    ps.append(y[9][0])
    qs.append(y[10][0])
    rs.append(y[11][0])

    # plot data on subplots
    angles.clear()
    anglerate.clear()
    translations.clear()
    translationrate.clear()
    angles.plot(simtime, np.rad2deg(phis), "r-", label="$\phi$")
    angles.plot(simtime, np.rad2deg(thetas), "g-", label="$\ttheta$")
    angles.plot(simtime, np.rad2deg(psis), "b-", label="$\psi$")
    translations.plot(simtime, pns, "m-", label="North")
    translations.plot(simtime, pes, "y-", label="East")
    translations.plot(simtime, pds, "k-", label="Down")
    anglerate.plot(simtime, np.rad2deg(ps), "r-", label="p")
    anglerate.plot(simtime, np.rad2deg(qs), "g-", label="q")
    anglerate.plot(simtime, np.rad2deg(rs), "b-", label="r")
    translationrate.plot(simtime, us, "r-", label="u")
    translationrate.plot(simtime, vs, "g-", label="v")
    translationrate.plot(simtime, ws, "b-", label="w")
    angles.grid('major')
    angles.set_xlabel('Times (s)')
    angles.set_ylabel('Angle (degrees)')
    angles.legend(loc='upper right')
    translations.grid('major')
    translations.set_xlabel('Times (s)')
    translations.set_ylabel('Distance (m)')
    translations.legend(loc='upper right')
    anglerate.grid('major')
    anglerate.set_xlabel('Times (s)')
    anglerate.set_ylabel('Angular Velocity (degrees/s)')
    anglerate.legend(loc='upper right')
    translationrate.grid('major')
    translationrate.set_xlabel('Times (s)')
    translationrate.set_ylabel('Velocity (m/s)')
    translationrate.legend(loc='upper right')

    

    # -------increment time-------------
    sim_time += SIM.ts_simulation