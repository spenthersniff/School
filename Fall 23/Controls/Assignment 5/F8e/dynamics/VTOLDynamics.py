import numpy as np 
import parameters.VTOLParam as P

class VTOLDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # z initial position
            [P.zdot0],  # zdot initial velocity
            [P.h0],
            [P.hdot0],
            [P.theta0],
            [P.thetadot0]
            ])
        # simulation time step
        self.Ts = P.Ts
        # Mass of the cart, kg
        self.mc = P.mc * (1.+alpha*(2.*np.random.rand()-1.))
        self.mr = P.mr * (1.+alpha*(2.*np.random.rand()-1.))
        self.jc = P.jc * (1.+alpha*(2.*np.random.rand()-1.))
        # Damping coefficient, Ns/m
        self.mu = P.mu * (1.+alpha*(2.*np.random.rand()-1.))
        # Spring Constant
        self.d=P.d * (1.+alpha*(2.*np.random.rand()-1.))
        # gravity constant is well known, don't change.
        self.g = P.g
        self.force_limit = P.F_max

    def update(self, fr, fl):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
    
        self.rk4_step(fr,fl)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, fr, fl):
        # states
        z = state[0][0]
        zdot = state[1][0]
        h = state[2][0]
        hdot = state[3][0]
        theta = state[4][0]
        thetadot = state[5][0]
    
        # The equations of motion.
        zddot = ((-np.sin(theta)*fr)/(self.mc+2*self.mr))-((np.sin(theta)*fl)/(self.mc+2*P.mr))-((self.mu*zdot)/(P.mc+2*P.mr))
        hddot = ((np.cos(theta)*fr)/(self.mc+2*self.mr))+((np.cos(theta)*fl)/(self.mc+2*self.mr))-self.g
        thetaddot = ((self.d*fr)/(self.jc+2*self.mr*(self.d)**2))-((self.d*fl)/(self.jc+2*self.mr*(self.d)**2))
        # build xdot and return
        xdot = np.array([[zdot], [hdot], [thetadot], [zddot], [hddot], [thetaddot]])
        return xdot

    def h(self):
        # return y = h(x)
        z = self.state[0][0]
        h = self.state[2][0]
        theta = self.state[4][0]
        y = np.array([[z], [h], [theta]])
        return y

    def rk4_step(self, fr, fl):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, fr, fl)
        F2 = self.f(self.state + self.Ts / 2 * F1, fr, fl)
        F3 = self.f(self.state + self.Ts / 2 * F2, fr, fl)
        F4 = self.f(self.state + self.Ts * F3, fr, fl)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)