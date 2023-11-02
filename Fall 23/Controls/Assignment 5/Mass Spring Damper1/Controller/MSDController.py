import numpy as np
import parameters.MSDParam as P


class controller:
    def __init__(self):
        self.m = P.m
        self.k = P.k
        self.b = P.b

        self.kp = P.kp
        self.kd = P.kd

    def update(self, z_r, state):
        z = state[0]
        z_dot = state[1]
        f_eq = P.k * z
        f_squig = self.kp * (z_r - z)  - self.kd * z_dot
        f = f_eq + f_squig
        return f[0]