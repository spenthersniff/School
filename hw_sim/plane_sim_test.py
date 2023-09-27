import sys
sys.path.append('C:/Users/spenc/OneDrive - University of Cincinnati/2023 Fall/Flight Mechanics/Python/plane_sim3')# one directory up
import numpy as np
from math import cos, sin, tan
import scipy.linalg as linalg
import matplotlib.pyplot as plt
import parameters.aerosonde_parameters as SIM
from viewers.plane_animation import plane_animation
from tools.signalGenerator import signalGenerator
from dynamics.PlaneDynamics import MAVDynamics
from wind.wind import *
from aerodynamics.aerodynamics import *
import keyboard

state=SIM.states0
plane_anim=plane_animation(state, scale=5)
uav=MAVDynamics(alpha=0.0)
Vs=np.array([[5],[3],[2]])
wind=wind(Vs)
forces_moments=forces_moments()
da=0
de=0
dr=0
dt=1

# create subplots
# angles=plane_anim.fig.add_subplot(5,2,9)
# translations=plane_anim.fig.add_subplot(5,2,2)
# anglerate=plane_anim.fig.add_subplot(5,2,10)
# translationrate=plane_anim.fig.add_subplot(5,2,6)
# forces=plane_anim.fig.add_subplot()
# moments=plane_anim.fig.add_subplot()
# deflections=plane_anim.fig.add_subplot()

forces = plane_anim.fig.add_subplot(3,3,2)
forcep = forces.get_position(); forcep.x0 += 0.1; forcep.x1 += 0.075; forces.set_position(forcep)
moments = plane_anim.fig.add_subplot(3,3,5)
momentp = moments.get_position(); momentp.x0 += 0.1; momentp.x1 += 0.075; moments.set_position(momentp)
position = plane_anim.fig.add_subplot(3,3,8)
positionp = position.get_position(); positionp.x0 += 0.1; positionp.x1 += 0.075; position.set_position(positionp)
velocity = plane_anim.fig.add_subplot(3,3,3)
velocityp = velocity.get_position(); velocityp.x0 += 0.1; velocityp.x1 += 0.075; velocity.set_position(velocityp)
angles = plane_anim.fig.add_subplot(3,3,6)
anglep = angles.get_position(); anglep.x0 += 0.1; anglep.x1 += 0.075; angles.set_position(anglep)
rates = plane_anim.fig.add_subplot(3,3,9)
ratep = rates.get_position(); ratep.x0 += 0.1; ratep.x1 += 0.075; rates.set_position(ratep)
throttle = plt.figure(1).add_subplot(1, 20, 1); throttlep = throttle.get_position(); throttlep.x0-=0.075; throttlep.x1-=0.075
throttle.set_position(throttlep)



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
tht = []
   

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

Va=np.sqrt(u**2+v**2+w**2)

    
while sim_time < SIM.end_time:

    if keyboard.is_pressed("down arrow"): de -= np.deg2rad(2)
    if keyboard.is_pressed("up arrow"): de += np.deg2rad(2)
    if keyboard.is_pressed("right arrow"): da += np.deg2rad(0.5); dr -= np.deg2rad(0.25)
    if keyboard.is_pressed("left arrow"): da -= np.deg2rad(0.5); dr += np.deg2rad(0.25)
    if keyboard.is_pressed("space"): de = 0; da = 0; dr = 0
    if keyboard.is_pressed("shift"):
        if dt < 1: dt += 0.05
    if keyboard.is_pressed("left control"):
        if dt > 0: dt -= 0.05
   
    
    Va, alpha, beta = wind.wendy(state, Va, sim_time)
    fx, fy, fz = forces_moments.forces(state, alpha, beta, da, de, dr, dt, Va).flatten()
    l, m, n = forces_moments.moments(state, alpha, beta, da, de, dr, dt, Va).flatten()
    state = uav.update(fx, fy, fz, l, m, n)
    pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state.flatten()
    plane_anim.update(pn, pe, pd, phi, theta, psi)


    # store data in lists from earlier
    simtime.append(sim_time)
    fxs.append(fx)
    fys.append(fy)
    fzs.append(fz)
    ls.append(l)
    ms.append(m)
    ns.append(n)
    tht.append(dt)
    pns.append(pn)
    pes.append(pe)
    pds.append(pd)
    us.append(u)
    vs.append(v)
    ws.append(w)
    phis.append(phi)
    thetas.append(theta)
    psis.append(psi)
    ps.append(p)
    qs.append(q)
    rs.append(r)

    # plot data on subplots
    angles.clear()
    forces.clear()
    moments.clear()
    rates.clear()
    position.clear()
    velocity.clear() 
    throttle.clear()
    throttle.bar(0,dt)
    angles.plot(simtime, np.rad2deg(phis), "r-", label="$\phi$")
    angles.plot(simtime, np.rad2deg(thetas), "g-", label="$\ttheta$")
    angles.plot(simtime, np.rad2deg(psis), "b-", label="$\psi$")
    forces.plot(simtime, fxs, "r-", label="Fx")
    forces.plot(simtime, fys, "g-", label="Fy")
    forces.plot(simtime, fzs, "b-", label="Fz")
    moments.plot(simtime, ls, "r-", label="l")
    moments.plot(simtime, ms, "g-", label="m")
    moments.plot(simtime, ns, "b-", label="n")
    position.plot(simtime, pns, "m-", label="North")
    position.plot(simtime, pes, "y-", label="East")
    position.plot(simtime, pds, "k-", label="Down")
    rates.plot(simtime, np.rad2deg(ps), "r-", label="p")
    rates.plot(simtime, np.rad2deg(qs), "g-", label="q")
    rates.plot(simtime, np.rad2deg(rs), "b-", label="r")
    velocity.plot(simtime, us, "r-", label="u")
    velocity.plot(simtime, vs, "g-", label="v")
    velocity.plot(simtime, ws, "b-", label="w")
    throttle.set_ylabel('Throttle')
    throttle.set_ylim(0, 1)
    angles.grid('major')
    angles.set_xlabel('Times (s)')
    angles.set_ylabel('Angle (degrees)')
    angles.legend(loc='upper right')
    forces.grid('major')
    forces.set_xlabel('Times (s)')
    forces.set_ylabel('Force (N)')
    forces.legend(loc='upper right')
    moments.grid('major')
    moments.set_xlabel('Times (s)')
    moments.set_ylabel('Moments (Nm)')
    moments.legend(loc='upper right')
    position.grid('major')
    position.set_xlabel('Times (s)')
    position.set_ylabel('Position (m)')
    position.legend(loc='upper right')
    rates.grid('major')
    rates.set_xlabel('Times (s)')
    rates.set_ylabel('Angular Velocity (degrees/s)')
    rates.legend(loc='upper right')
    velocity.grid('major')
    velocity.set_xlabel('Times (s)')
    velocity.set_ylabel('Velocity (m/s)')
    velocity.legend(loc='upper right')

    

    # -------increment time-------------
    sim_time += SIM.ts_simulation