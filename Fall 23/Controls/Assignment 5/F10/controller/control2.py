import parameters.VTOLParam as P
import numpy as np
from controller.PIDControl import PIDControl

class control:
    def __init__(self):
        self.mc = P.mc
        self.mr = P.mr
        self.kPtheta = 0.372
        self.kDtheta = 0.191
        self.kPz = -0.00771
        self.kDz = -0.0329
        self.kPh = 0.1134
        self.kDh = 0.5835
        self.kItheta = 0.5
        self.kIh = 0.5
        self.kIz = 0.5
        self.d = P.d
        self.g = P.g
        self.force_limit = P.F_max
        self.theta_max = P.theta_max
        self.Ts = P.Ts
        self.beta = P.beta

    

        self.torqueCtrl = PIDControl(self.kPtheta, self.kItheta, self.kDtheta, self.force_limit, self.beta, self.Ts)
        self.thetaCtrl = PIDControl(self.kPz, self.kIz, self.kDz, self.theta_max, self.beta, self.Ts)
        self.forceCtrl = PIDControl(self.kPh, self.kIh, self.kDh, self.force_limit, self.beta, self.Ts)

        self.limit = P.F_max

    def update(self, hr, zr, q):
        z = q[0][0]
        h = q[1][0]
        theta = q[2][0]

        tau_eq = 0.
        theta_r = self.thetaCtrl.PD(zr, z)
        tau_s = self.torqueCtrl.PID(theta_r, theta)
        tau = tau_eq + tau_s

        F_eq = (self.mc + (2.0*self.mr)) * self.g
        F_s = self.forceCtrl.PID(hr, h)
        F = F_eq + F_s

        fr = (tau + self.d*F)/(2*self.d)
        fl = F - fr

        fr = self.saturate(fr)
        fl = self.saturate(fl)

        return fr, fl
    
    def saturate(self, u):
        if abs(u) > self.force_limit:
            u = self.force_limit*np.sign(u)
        return u