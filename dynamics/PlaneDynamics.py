import numpy as np 
import parameters.aerosonde_parameters as P
from numpy import cos as c
from numpy import sin as s
from numpy import tan as tn

class MAVDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = P.states0
        # simulation time step
        self.Ts = P.ts_simulation
        # Mass of the plane, kg
        self.mass = P.mass
        # Moments of inertia
        self.jx = P.jx
        self.jy = P.jy
        self.jz = P.jz
        self.jxz = P.jxz
        # gammas
        self.g = P.g
        self.g1 = P.g1
        self.g2 = P.g2
        self.g3 = P.g3
        self.g4 = P.g4
        self.g5 = P.g5
        self.g6 = P.g1
        self.g7 = P.g7
        self.g8 = P.g8
        # gravity
        # self.gravity=self.gravity

    def update(self, fx, fy, fz, l, m, n):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
    
        self.rk4_step(fx, fy, fz, l, m, n)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, fx, fy, fz, l, m, n):
        # states
        pn = state[0][0]
        pe = state[1][0]
        pd = state[2][0]
        u = state[3][0]
        v = state[4][0]
        w = state[5][0]
        phi = state[6][0]
        theta = state[7][0]
        psi = state[8][0]
        p = state[9][0]
        q = state[10][0]
        r = state[11][0]
    
        # The equations of motion.
        pndot = u*c(theta)*c(psi)+v*(s(phi)*s(theta)*c(psi)-c(phi)*s(psi))+w*(c(phi)*s(theta)*c(psi)+s(phi)*s(psi))
        pedot = u*c(theta)*s(psi)+v*(s(phi)*s(theta)*s(psi)+c(phi)*c(psi))+w*(c(phi)*s(theta)*s(psi)-s(phi)*c(psi))
        pddot = u*(-s(theta))+v*s(phi)*c(theta)+w*c(phi)*c(theta)
        udot = r*v-q*w+(fx/P.mass)
        vdot = p*w-r*u+(fy/P.mass)
        wdot = q*u-p*v+(fz/P.mass)
        phidot = p+q*s(phi)*tn(theta)+r*c(phi)*tn(theta)
        thetadot = q*c(phi)-r*s(phi)
        psidot = q*s(phi)/c(theta)+r*c(phi)/c(theta)
        pdot = P.g1*p*q-P.g2*q*r+P.g3*l+P.g4*n
        qdot = P.g5*p*r-P.g6*(p**2-r**2)+m/P.jy
        rdot = P.g7*p*q-P.g1*q*r+P.g4*l+P.g8*n
        # build xdot and return
        xdot = np.array([[pndot], [pedot], [pddot], [udot], [vdot], [wdot], [phidot], [thetadot], [psidot], [pdot], [qdot], [rdot]])
        return xdot

    def h(self):
        # return y = h(x)
        pn = self.state[0][0]
        pe = self.state[1][0]
        pd = self.state[2][0]
        u = self.state[3][0]
        v = self.state[4][0]
        w = self.state[5][0]
        phi = self.state[6][0]
        theta = self.state[7][0]
        psi = self.state[8][0]
        p = self.state[9][0]
        q = self.state[10][0]
        r = self.state[11][0]
        y = np.array([[pn], [pe], [pd], [u], [v], [w], [phi], [theta], [psi], [p], [q], [r]])
        return y

    def rk4_step(self, fx, fy, fz, l, m, n):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, fx, fy, fz, l, m, n)
        F2 = self.f(self.state + self.Ts / 2 * F1, fx, fy, fz, l, m, n)
        F3 = self.f(self.state + self.Ts / 2 * F2, fx, fy, fz, l, m, n)
        F4 = self.f(self.state + self.Ts * F3, fx, fy, fz, l, m, n)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)