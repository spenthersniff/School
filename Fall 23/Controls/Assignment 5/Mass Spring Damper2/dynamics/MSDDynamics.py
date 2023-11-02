import numpy as np 
import parameters.MSDParam as P

class MSDDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # z initial position
            [P.zdot0]  # zdot initial velocity
        ])
        # simulation time step
        self.Ts = P.Ts
        # Mass of the cart, kg
        self.m = P.m * (1.+alpha*(2.*np.random.rand()-1.))
        # Damping coefficient, Ns/m
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))
        # Spring Constant
        self.k=P.k * (1.+alpha*(2.*np.random.rand()-1.))
        # gravity constant is well known, don't change.
        # self.g = P.g
        self.force_limit = P.F_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input force
        u = saturate(u, self.force_limit)
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, u):
        # Return xdot = f(x,u)
        z = state[0][0]
        zdot = state[1][0]
        f = u
        # The equations of motion.
        zddot = (u-self.b*zdot-self.k*z)/self.m
        # build xdot and return
        xdot = np.array([[zdot], [zddot]])
        return xdot

    def h(self):
        # return y = h(x)
        z = self.state[0][0]
        y = np.array([[z]])
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

        
def saturate(u, limit):
    if abs(u) > limit:
        u = limit*np.sign(u)
    return u
