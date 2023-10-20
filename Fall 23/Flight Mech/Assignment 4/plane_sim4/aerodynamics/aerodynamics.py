import numpy as np
from dynamics.PlaneDynamics import *
from parameters.aerosonde_parameters import *
from numpy import cos as C
from numpy import sin as s
from numpy import tan as tn
from numpy import exp as E

class forces_moments:
    # blending function
    def sigma(junk,alpha):
        return (1+E(-M*(alpha-alpha0))+E(M*(alpha+alpha0)))/((1+E(-M*(alpha-alpha0)))*(1+E(M*(alpha+alpha0))))
   
    def CLalpha(self, alpha):
        # return (1-self.sigma(alpha))*(C_L_0+C_L_alpha*alpha)+self.sigma(alpha)*(2*np.sign(alpha)*((s(alpha))**2)*C(alpha))
        return C_L_0+C_L_alpha*alpha
   
    def CDalpha(junk,alpha):
        # return C_D_p+((C_L_0+C_L_alpha*alpha)**2)/(np.pi*e*AR)
        return C_D_0+C_D_alpha*alpha
   
    def Cx(self, alpha):
        return -self.CDalpha(alpha)*C(alpha)+self.CLalpha(alpha)*s(alpha)
   
    def Cxq(junk,alpha):
        return -C_D_q*C(alpha)+C_L_q*s(alpha)
   
    def Cxde(junk,alpha):
        return -C_D_delta_e*C(alpha)+C_L_delta_e*s(alpha)
   
    def Cz(self, alpha):
        return -self.CDalpha(alpha)*s(alpha)-self.CLalpha(alpha)*C(alpha)
   
    def Czq(junk,alpha):
        return -C_D_q*s(alpha)-C_L_q*C(alpha)
   
    def Czde(junk,alpha):
        return -C_D_delta_e*s(alpha)-C_L_delta_e*C(alpha)
    
    def forces(self, state, alpha, beta, da, de, dr, dt, Va):
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

        pn,pe,pd,u,v,w,phi,theta,psi,p,q,r = state.flatten()
        
        # forces
        gravy=np.array([[-mass*gravity*s(theta)],
                        [mass*gravity*C(theta)*s(phi)],
                        [mass*gravity*C(theta)*C(phi)]])
        aero=0.5*rho*(Va**2)*S_wing*np.array([[self.Cx(alpha)+self.Cxq(alpha)*c*q/(2*Va)+self.Cxde(alpha)*de],
                                            [C_Y_0+C_Y_beta*beta+C_Y_p*b*p/(2*Va)+C_Y_r*b*r/(2*Va)+C_Y_delta_a*da+C_Y_delta_r*dr],
                                            [self.Cz(alpha)+self.Czq(alpha)*c*q/(2*Va)+self.Czde(alpha)*de]])
        prop=0.5*rho*S_prop*C_prop*np.array([[((k_motor*dt)**2)-(Va**2)], [0], [0]])
        forces=gravy+aero+prop
        return forces
    
    def moments(self, state, alpha, beta, da, de, dr, dt, Va):
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

        pn,pe,pd,u,v,w,phi,theta,psi,p,q,r = state.flatten()

        # moments
        aero=0.5*rho*(Va**2)*S_wing*np.array([[b*(C_ell_0+C_ell_beta*beta+C_ell_p*b*p/(2*Va)+C_ell_r*b*r/(2*Va)+C_ell_delta_a*da+C_ell_delta_r*dr)],
                                              [c*(C_m_0+C_m_alpha*alpha+C_m_q*c*q/(2*Va)+C_m_delta_e*de)],
                                              [b*(C_n_0+C_n_beta*beta+C_n_p*b*p/(2*Va)+C_n_r*b*r/(2*Va)+C_n_delta_a*da+C_n_delta_r*dr)]])
        prop=np.array([[-k_tp*(k_omega*dt)**2], [0], [0]])
        moments=aero+prop
        return moments