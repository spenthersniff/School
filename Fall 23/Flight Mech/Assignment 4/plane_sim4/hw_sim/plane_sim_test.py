import sys
sys.path.append('C:/Users/spenc/Documents/School/Fall 23/Flight Mech/Assignment 5/plane_sim5')# one directory up
import numpy as np
from math import cos, sin, tan
import scipy.linalg as linalg
import matplotlib.pyplot as plt
import parameters.aerosonde_parameters as SIM
from viewers.plane_animation import plane_animation
from tools.signalGenerator import signalGenerator
from dynamics.PlaneDynamics import MAVDynamics
from trim.compute_trim import ComputeTrim
from wind.wind import *
from aerodynamics.aerodynamics import *
from compute_gains.compute_gains import Compute_Gains
import keyboard

state=SIM.states0
plane_anim=plane_animation(state, scale=5)
uav=MAVDynamics()
Vs=np.array([[0.],[0.],[0.]])
trim=ComputeTrim()
wind=wind(Vs)
forces_moments=forces_moments()
gain=Compute_Gains()
# da=0
# de=0
# dr=0
# dt=0.5


forces = plane_anim.fig.add_subplot(3,3,2)
forcep = forces.get_position(); forcep.x0 += 0.1; forcep.x1 += 0.075; forces.set_position(forcep)
moments = plane_anim.fig.add_subplot(3,3,5)
momentp = moments.get_position(); momentp.x0 += 0.1; momentp.x1 += 0.075; moments.set_position(momentp)
position = plane_anim.fig.add_subplot(3,3,8)
positionp = position.get_position(); positionp.x0 += 0.1; positionp.x1 += 0.075; position.set_position(positionp)
velocity = plane_anim.fig.add_subplot(4,3,3)
velocityp = velocity.get_position(); velocityp.x0 += 0.1; velocityp.x1 += 0.075; velocity.set_position(velocityp)
angles = plane_anim.fig.add_subplot(4,3,6)
anglep = angles.get_position(); anglep.x0 += 0.1; anglep.x1 += 0.075; angles.set_position(anglep)
rates = plane_anim.fig.add_subplot(4,3,9)
ratep = rates.get_position(); ratep.x0 += 0.1; ratep.x1 += 0.075; rates.set_position(ratep)
defl = plane_anim.fig.add_subplot(4,3,12)
deflp = defl.get_position(); deflp.x0 += 0.1; deflp.x1 += 0.075; defl.set_position(deflp)
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
das = []
des = []
drs = []

   

# main simulation loop
print("Press Command-Q to exit...")




Va=35.0
Y=np.radians(0)
R=np.inf

x_trim, u_trim = trim.compute_trim(Va, Y, R)
d_e, d_t, d_a, d_r = u_trim.flatten()

pn = 0
pe = 0
pd = -100
u = x_trim.item(3)
v = x_trim.item(4)
w = x_trim.item(5)
phi = x_trim.item(6)
theta = x_trim.item(7)
psi = x_trim.item(8)
p = x_trim.item(9)
q = x_trim.item(10)
r = x_trim.item(11)

# # Phugoid Mode
# pn = 0
# pe = 0
# pd = -100
# u = x_trim.item(3) - 10
# v = x_trim.item(4)
# w = x_trim.item(5)
# phi = x_trim.item(6)
# theta = x_trim.item(7)*10
# psi = x_trim.item(8)
# p = x_trim.item(9)
# q = x_trim.item(10) + 10
# r = x_trim.item(11)

# # Short Period Mode
# pn = 0
# pe = 0
# pd = -100
# u = x_trim.item(3) + 10
# v = x_trim.item(4)
# w = x_trim.item(5)
# phi = x_trim.item(6)
# theta = x_trim.item(7)
# psi = x_trim.item(8)
# p = x_trim.item(9)
# q = x_trim.item(10) + 10
# r = x_trim.item(11)   

# # Roll Mode
# pn = 0
# pe = 0
# pd = -100
# u = x_trim.item(3) 
# v = x_trim.item(4)
# w = x_trim.item(5)
# phi = x_trim.item(6)
# theta = x_trim.item(7)
# psi = x_trim.item(8)
# p = x_trim.item(9) + 10
# q = x_trim.item(10)
# r = x_trim.item(11)

# # Dutch Roll Mode
# pn = 0
# pe = 0
# pd = -100
# u = x_trim.item(3) 
# v = x_trim.item(4)
# w = x_trim.item(5)
# phi = x_trim.item(6) + 10
# theta = x_trim.item(7)
# psi = x_trim.item(8)
# p = x_trim.item(9)*10
# q = x_trim.item(10)
# r = x_trim.item(11)*10

# # Spiral Mode
# pn = 0
# pe = 0
# pd = -100
# u = x_trim.item(3) 
# v = x_trim.item(4)
# w = x_trim.item(5)
# phi = x_trim.item(6)
# theta = x_trim.item(7)
# psi = x_trim.item(8)
# p = x_trim.item(9)
# q = x_trim.item(10)
# r = x_trim.item(11) + 40

T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_Va_theta, T_beta_delta_r = gain.transfer_functions(x_trim, u_trim)
A_lat, B_lat, elatvalue, elongvalue = gain.statespace(x_trim, u_trim)
 
print("Trim Conditions")
print(f"E: {np.degrees(d_e):.2f} deg")
print(f"T: {d_t*100:.2f} %")
print(f"A: {np.degrees(d_a):.2f} deg")
print(f"R: {np.rad2deg(d_r):.2f} deg")

state0 = np.array([[pn], [pe], [pd], [u], [v], [w], [phi], [theta], [psi], [p], [q], [r]])
uav.state = np.ndarray.copy(state0)


Va = np.sqrt(u**2 + v**2 + w**2)


while sim_time < SIM.end_time:
    t_next_plot = sim_time + P.ts_plotting
    while sim_time < t_next_plot:
        Va, alpha, beta = wind.wendy(uav.state, Va, sim_time)
        # print(np.degrees(alpha),np.degrees(beta))
        fx, fy, fz = forces_moments.forces(uav.state, alpha, beta, d_a, d_e, d_r, d_t, Va)
        l, m, n = forces_moments.moments(uav.state, alpha, beta, d_a, d_e, d_r, d_t, Va)
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
        tht.append(d_t)
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
        das.append(d_a)
        des.append(d_e)
        drs.append(d_r)

        # plot data on subplots
        angles.clear()
        forces.clear()
        moments.clear()
        rates.clear()
        defl.clear()
        position.clear() 
        velocity.clear() 
        throttle.clear()
        throttle.bar(0,d_t)
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
        defl.plot(simtime, des, "r-", label="e")
        defl.plot(simtime, das, "g-", label="a")
        defl.plot(simtime, drs, "b-", label="r")
        velocity.plot(simtime, us, "r-", label="u")
        velocity.plot(simtime, vs, "g-", label="v")
        velocity.plot(simtime, ws, "b-", label="w")
        throttle.set_ylabel('Throttle')
        throttle.set_ylim(0, 1)
        angles.grid('major')
        angles.set_ylabel('Angle (degrees)')
        angles.legend(loc='upper right')
        forces.grid('major')
        forces.set_ylabel('Force (N)')
        forces.legend(loc='upper right')
        moments.grid('major')
        moments.set_ylabel('Moments (Nm)')
        moments.legend(loc='upper right')
        position.grid('major')
        position.set_xlabel('Time (s)')
        position.set_ylabel('Position (m)')
        position.legend(loc='upper right')
        rates.grid('major')
        rates.set_ylabel('Ang Velocity (degrees/s)')
        rates.legend(loc='upper right')
        defl.grid('major')
        defl.set_xlabel('Time (s)')
        defl.set_ylabel('Deflections (degrees)')
        defl.legend(loc='upper right')
        velocity.grid('major')
        velocity.set_ylabel('Velocity (m/s)')
        velocity.legend(loc='upper right')



        # -------increment time-------------
        sim_time += SIM.ts_simulation