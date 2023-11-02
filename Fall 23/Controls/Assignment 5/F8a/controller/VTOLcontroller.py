import numpy as np
import parameters.VTOLParam as P


class controller:
    def __init__(self):
        self.kp = P.kp
        self.kd = P.kd

    def update(self, h_r, state):
        h = state[1]
        h_dot = state[4]
        f_eq = (P.mc + (2.0*P.mr)) * P.g
        f_squig = self.kp * (h_r - h)  - self.kd * h_dot
        f = f_eq + f_squig
        return f[0]