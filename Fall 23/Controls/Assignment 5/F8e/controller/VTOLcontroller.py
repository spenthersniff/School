import numpy as np
import parameters.VTOLParam as P



class controller:
    def __init__(self):
        self.mr = P.mr
        self.mc = P.mc
        self.d = P.d
        self.g = P.g
        self.mu = P.mu
        self.jc = P.jc
        self.d = P.d
        self.kPh = 0.1134
        self.kDh = 0.5835
        self.kPtheta = 0.372
        self.kDtheta = 0.191
        self.kPz = -0.00771
        self.kDz = -0.0329
        self.Fmax = P.F_max

    def update(self, zc, hc, state):
        # z, zdot, h, hdot, theta, thetadot = state.flatten()
        z, h, theta, zdot, hdot, thetadot =  state.flatten()
        
        # Theta
        taueq = 0.
        zs = self.kPz*(zc - z) - self.kDz*zdot
        tauc = self.kPtheta*(zs - theta)  - self.kDtheta*thetadot
        tau = taueq + tauc
        
        # Height
        feq = self.g*(self.mc + 2*self.mr)
        fc = self.kPh*(hc - h)  - self.kDh*hdot
        f = feq + fc
        
        # Forces
        fr = (tau + self.d*f)/(2*self.d)
        fl = f - fr

        # fr, fl = self.saturate(fr, fl)

        return fr, fl