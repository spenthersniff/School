import numpy as np 
import parameters.aerosonde_parameters as P
from numpy import cos as c
from numpy import sin as s
from numpy import tan as tn
from tools.rotations import Quaternion2Euler, Quaternion2Rotation, Euler2Rotation

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
        # pn = state[0][0]
        # pe = state[1][0]
        # pd = state[2][0]
        # u = state[3][0]
        # v = state[4][0]
        # w = state[5][0]
        # phi = state[6][0]
        # theta = state[7][0]
        # psi = state[8][0]
        # p = state[9][0]
        # q = state[10][0]
        # r = state[11][0]

        pn,pe,pd,u,v,w,phi,theta,psi,p,q,r = state.flatten()
    
        # The equations of motion.
        pndot = u*c(theta)*c(psi)+v*(s(phi)*s(theta)*c(psi)-c(phi)*s(psi))+w*(c(phi)*s(theta)*c(psi)+s(phi)*s(psi))
        pedot = u*c(theta)*s(psi)+v*(s(phi)*s(theta)*s(psi)+c(phi)*c(psi))+w*(c(phi)*s(theta)*s(psi)-s(phi)*c(psi))
        pddot = u*(-s(theta))+v*s(phi)*c(theta)+w*c(phi)*c(theta)
        udot = r*v-q*w+(fx/self.mass)
        vdot = p*w-r*u+(fy/self.mass)
        wdot = q*u-p*v+(fz/self.mass)
        phidot = p+q*s(phi)*tn(theta)+r*c(phi)*tn(theta)
        thetadot = q*c(phi)-r*s(phi)
        psidot = q*s(phi)/c(theta)+r*c(phi)/c(theta)
        pdot = self.g1*p*q-self.g2*q*r+self.g3*l+self.g4*n
        qdot = self.g5*p*r-self.g6*(p**2-r**2)+m/self.jy
        rdot = self.g7*p*q-self.g1*q*r+self.g4*l+self.g8*n
        # build xdot and return
        # print(pndot)
        # print(pedot)
        # print(pddot)
        # print(udot)
        # print(vdot)
        # print(wdot)
        # print(phidot)
        # print(thetadot)
        # print(psidot)
        # print(pdot)
        # print(qdot)
        # print(rdot)
        xdot = np.array([[pndot], [pedot], [pddot], [udot[0]], [vdot[0]], [wdot[0]], [phidot], [thetadot], [psidot], [pdot[0]], [qdot[0]], [rdot[0]]],dtype=float)
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
    
    # def _derivatives(self, state, forces_moments):
    #     """
    #     for the dynamics xdot = f(x, u), returns f(x, u)
    #     """
    #     # extract the states
    #     # north = state.item(0)
    #     # east = state.item(1)
    #     # down = state.item(2)
    #     u = state.item(3)
    #     v = state.item(4)
    #     w = state.item(5)
    #     phi = state.item(6)
    #     theta = state.item(7)
    #     psi = state.item(8)
    #     #e3 = state.item(9)
    #     p = state.item(9)
    #     q = state.item(10)
    #     r = state.item(11)
    #     #   extract forces/moments
    #     fx = forces_moments.item(0)
    #     fy = forces_moments.item(1)
    #     fz = forces_moments.item(2)
    #     l_in = forces_moments.item(3)
    #     m_in = forces_moments.item(4)
    #     n_in= forces_moments.item(5)
    #     body_vel=np.array([u, v, w]).T
    #     inertial_vel=Euler2Rotation(phi,theta,psi)@body_vel
    #     #print(body_vel)
    # #print(R_b_w.shape)
    #     m=P.mass
    #     g=P.gravity
    #     ForceVecBody=1/m*np.array([[fx], [fy], [fz]],dtype=float)
    #     jx=self.jx
    #     jy=self.jy
    #     jz=self.jz
    #     jxz=self.jxz
    #     gamma=jx*jz-jxz**2
    #     gamma1 = (jxz*(jx-jy+jz))/gamma
    #     gamma2 = (jz*(jz-jy)+jxz**2)/gamma
    #     gamma3 = jz/gamma
    #     gamma4 = jxz/gamma
    #     gamma5 = (jz-jx)/jy
    #     gamma6 = jxz/jy
    #     gamma7 = ((jx-jy)*jx+jxz**2)/gamma
    #     gamma8 = jx/gamma
    #     #gravity_vec=np.array([0, 0, g]).T
    # #print(ThrustVecBody.shape)
        
    # #print(temp1)
      
    #     #Rgb=rotation_matrix_Gyro2Body(phi, theta)

    #     # position kinematics
    #     #pos_dot = 
    #     north_dot = inertial_vel[0]
    #     east_dot = inertial_vel[1]
    #     down_dot = inertial_vel[2]

    #     # position dynamics # body frame # cross product is missi
    #     temp1=np.array([[r*v-q*w],
    #                     [p*w-r*u],
    #                     [q*u-p*v]])+ForceVecBody
    #     #print(temp1)
    #     u_dot = temp1[0]
    #     v_dot = temp1[1]
    #     w_dot = temp1[2]

    #     # rotational kinematics
    #     ang_vel=np.array([[p], [q], [r]],dtype=float)
    #     Rgb=np.array([[1, s(phi)*tn(theta), c(phi)*tn(theta)],
    #     [0, c(phi), -s(phi)],
    #     [0, s(phi)/c(theta), c(phi)/c(theta)]])
    #     temp2=Rgb@ang_vel
    #    # print('temp2=',temp2)
    #     phi_dot=temp2[0]
    #     theta_dot=temp2[1]
    #     psi_dot=temp2[2]

    #     # rotatonal dynamics
    #     temp3=np.array([[gamma1*p*q-gamma2*q*r],[gamma5*p*r-gamma6*(p**2-r**2)],[gamma7*p*q-gamma1*q*r]])+np.array([[gamma3*l_in+gamma4*n_in],[(1/jy)*m_in],[gamma4*l_in+gamma8*n_in]],dtype=float)
    #     p_dot= temp3[0]
    #     q_dot= temp3[1]
    #     r_dot= temp3[2]
    #     #print(temp3)

    #     # collect the derivative of the states
    #     x_dot = np.array([[north_dot], [east_dot], [down_dot], [u_dot], [v_dot], [w_dot],
    #                        [phi_dot], [theta_dot], [psi_dot], [p_dot], [q_dot], [r_dot]],dtype=float)
    #     #print(x_dot)
    #     return x_dot

    def rk4_step(self, fx, fy, fz, l, m, n):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, fx, fy, fz, l, m, n)
        F2 = self.f(self.state + self.Ts / 2 * F1, fx, fy, fz, l, m, n)
        F3 = self.f(self.state + self.Ts / 2 * F2, fx, fy, fz, l, m, n)
        F4 = self.f(self.state + self.Ts * F3, fx, fy, fz, l, m, n)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)