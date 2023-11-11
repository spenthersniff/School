import numpy as np
import control.matlab as mt
from tools.rotations import *


class wind():
    def __init__(self, Vs):
        self.Vs = Vs
   
    def Dryden(self, t, Va):
        # gust model params
        Lu = Lv = 200 # m
        Lw = 50 # m
        sigmau = sigmav = 1.06 # m/s
        sigmaw = 0.7 # m/s

        # transfer function
        Hu = mt.tf([0, sigmau*np.sqrt(2*Va/Lu)],[1,Va/Lu])
        Hv = mt.tf([sigmav*np.sqrt(3*Va/Lv), sigmav*np.sqrt(3*Va/Lv)*Va/(np.sqrt(3)*Lv)], [1, 2*Va/Lw, (Va/Lw)**2])
        Hw = mt.tf([sigmaw*np.sqrt(3*Va/Lw), sigmaw*np.sqrt(3*Va/Lw)*Va/(np.sqrt(3)*Lw)], [1, 2*Va/Lw, (Va/Lw)**2])

        # white noise
        wnu = np.random.normal(0, 1, 1)[0]
        wnv = np.random.normal(0, 1, 1)[0]
        wnw = np.random.normal(0, 1, 1)[0]

        # solve transfer function
        uwg, _, _ = mt.lsim(Hu, wnu, [0,t])
        vwg, _, _ = mt.lsim(Hv, wnv, [0,t])
        wwg, _, _ = mt.lsim(Hw, wnw, [0,t])

        return np.array([[uwg[1]],[vwg[1]],[wwg[1]]])
    
    def wendy(self, state, Va, sim_time):
        # pn = state[0,0]
        # pe = state[1,0]
        # pd = state[2,0]
        # u = state[3,0]
        # v = state[4,0]
        # w = state[5,0]
        # phi = state[6,0]
        # theta = state[7,0]
        # psi = state[8,0]
        # p = state[9,0]
        # q = state[10,0]
        # r = state[11,0]
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state.flatten()
        wind = v_to_b(phi,theta,psi)*self.Vs+self.Dryden(sim_time, Va)

        aspeed=np.array([[u-wind[0][0]],
                         [v-wind[1][0]],
                         [w-wind[2][0]]])
        # ur= u-wind[0][0]
        # vr=v-wind[1][0]
        # wr=w-wind[2][0]
        ur, vr, wr = aspeed.flatten()
        Va=np.sqrt(ur**2 + vr**2 + wr**2)
        alpha=np.arctan(wr/ur)
        beta=np.arcsin(vr/Va)
        return Va, alpha, beta



