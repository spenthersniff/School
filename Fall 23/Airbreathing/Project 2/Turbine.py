import numpy as np
cos=np.cos
sin=np.sin
tan=np.tan
atan=np.arctan

T01 = 2275.0              # turbine inlet temp, K
T03 = 755.329018          # compressor exit temp, K
P01 = 19.22525            # turbine inlet pressure, bar
n_inf = 0.92              # polytropic turbine efficiency
n_m = 0.99                # mechanical efficiency
w_comp = 437774.06        # compressor work required, J/kg
mdot_a = 280              # total mass flow of air, kg/s
mdot_core_a = 40          # core mass flow of air, kg/s
n_stages = 3.0            # number of turbine stages
cp = 1148.0               # for hot gas, J/(kg*K)
gamma = 1.333             # specific heat ratio
Ca1 = 280.0               # axial inlet speed, m/s
n_m = 0.99                # mechanical efficiency
n = 240.0433              # rotational speed, rev/s
omega = 2*np.pi*n         # rotational speed, rad/s
a1_inlet = np.radians(0.0)      # inlet relative gas angle, radians
R = 287                   # J/(kg*k)
d_T_fan = 319.7229897-288 # fan temp drop, K

f = (cp*T01-1005*T03)/(43100000-cp*T01)
mdot_f = f*mdot_core_a         # fuel mass flow, kg/s
mdot_core = mdot_core_a+mdot_f      # total mass flow, kg/s

# calculate fan work
wfan = 1005*d_T_fan

w_turb = (mdot_a*wfan+mdot_core_a*w_comp)/(n_m*mdot_core) # J/kg


def gas_angles(phi, psi, dor):
    b2 = atan((1/(2*phi))*((psi/2)-2*dor))
    b3 = atan((1/(2*phi))*((psi/2)+2*dor))
    a2 = atan(tan(b2)+(1/phi))
    a3 = atan(tan(b3)-(1/phi))
    return b2, b3, a2, a3

def meanline(T01, P01, Ca1, phi, psi, dor, omega, a1):
    b2, b3, a2, a3 = gas_angles(phi, psi, dor)
    
    # tip speed
    U = Ca1/phi

    # mean radius
    r_m = U/omega
   
    # inlet whirl velocity
    Cw1 = Ca1*tan(a1)

    C1 = Ca1/cos(a1)

    T013 = (U**2)*psi/(2*cp)

    # specific work for stage
    w = cp*T013

    # finish solving velocity triangle
    Ca2 = Ca1
    Cw2 = Ca2*tan(a2)
    C2 = Ca2/cos(a2)
    V2 = Ca2/cos(b2)

    Ca3 = Ca1
    Cw3 = Ca3*tan(a3)
    C3 = Ca3/cos(a3)
    V3 = Ca3/cos(b3)

    T03 = T01 - T013
    T_rat = T03/T01
    m_stuff = n_inf*(gamma-1)/gamma
    P_rat = T_rat**(1/m_stuff)
    P03 = P01*P_rat

    return b2, b3, a2, a3, U, r_m, Cw1, C1, T013, w, Ca2, Cw2, C2, V2, Ca3, Cw3, C3, V3, T03, T_rat, P_rat, P03


def final_stage_mean(T01, P01, Ca1, omega, a1, rm, w):
    
    U = omega*rm
    # no whirl at turbine exit
    a3 = 0.0
    Ca3 = Ca1
    Cw3 = Ca3*tan(a3)
    C3 = Ca3/cos(a3)
    b3 = atan(U/Ca3)
    V3 = Ca3/cos(b3)
   
    Cw1 = Ca1*tan(a1)
    
    C1 = Ca1/cos(a1)
    
    Ca2 = Ca1
    a2 = atan(w/(U*Ca2))
    Cw2 = Ca2*tan(a2)
    b2 = atan((Cw2 - U)/Ca2)
    
    C2 = Ca2/cos(a2)
    V2 = Ca2/cos(b2)

    T013 = w/cp
    T03 = T01 - T013
    T_rat = T03/T01
    m_stuff = n_inf*(gamma-1)/gamma
    P_rat = T_rat**(1/m_stuff)
    P03 = P01*P_rat
    
    phi = Ca1/U
    psi = 2*Ca1*(tan(b2)+tan(b3))/U
    dor = Ca1*(tan(b3)-tan(b2))/(2*U)

    return b2, b3, a2, a3, U, Cw1, C1, T013, Ca2, Cw2, C2, V2, Ca3, Cw3, C3, V3, T03, T_rat, P_rat, P03, phi, psi, dor

# mean stage 1 calculations
phi1_m = 0.78              # flow coefficient, >=0.78
psi1_m = 3.3               # blade loading (temp drop) coefficient, <=3.3
dor1_m = 0.5               # degree of reaction
b2_s1, b3_s1, a2_s1, a3_s1, U_s1, r_m1_s1, Cw1_s1, C1_s1, T013_s1, w1_s1, Ca2_s1, Cw2_s1, C2_s1, V2_s1, Ca3_s1, Cw3_s1, C3_s1, V3_s1, T03_s1, T_rat_s1, P_rat_s1, P03_s1 = meanline(T01, P01, Ca1, phi1_m, psi1_m, dor1_m, omega, a1_inlet)
print('stage 1 mean','\n','\nb2_s1:',np.degrees(b2_s1), '\nb3_s1', np.degrees(b3_s1), '\na2_s1', np.degrees(a2_s1), '\na3_s1', np.degrees(a3_s1), '\nU_s1',U_s1, '\nCw1_s1', Cw1_s1, '\nC1_s1', C1_s1, '\nT013_s1', T013_s1, '\nCa2_s1', Ca2_s1, '\nCw2_s1', Cw2_s1, '\nC2_s1', C2_s1, '\nV2_s1', V2_s1, '\nCa3_s1', Ca3_s1, '\nCw3_s1', Cw3_s1, '\nC3_s1', C3_s1, '\nV3_s1', V3_s1, '\nT03_s1', T03_s1, '\nT_rat_s1', T_rat_s1, '\nP_rat_s1', P_rat_s1, '\nP03_s1', P03_s1)


# mean stage 2 calculations
phi2_m = 0.78              # flow coefficient, >=0.78
psi2_m = 3.3               # blade loading (temp drop) coefficient, <=3.3
dor2_m = 0.5               # degree of reaction
b2_s2, b3_s2, a2_s2, a3_s2, U_s2, r_m2_s2, Cw1_s2, C1_s2, T013_s2, w2_s2, Ca2_s2, Cw2_s2, C2_s2, V2_s2, Ca3_s2, Cw3_s2, C3_s2, V3_s2, T03_s2, T_rat_s2, P_rat_s2, P03_s2 = meanline(T01, P01, Ca1, phi2_m, psi2_m, dor2_m, omega, a3_s1)
print('stage 2 mean','\n','\nb2_s2:',np.degrees(b2_s2), '\nb3_s2', np.degrees(b3_s2), '\na2_s2', np.degrees(a2_s2), '\na3_s2', np.degrees(a3_s2), '\nU_s2',U_s2, '\nCw1_s2', Cw1_s2, '\nC1_s2', C1_s2, '\nT013_s2', T013_s2, '\nCa2_s2', Ca2_s2, '\nCw2_s2', Cw2_s2, '\nC2_s2', C2_s2, '\nV2_s2', V2_s2, '\nCa3_s2', Ca3_s2, '\nCw3_s2', Cw3_s2, '\nC3_s2', C3_s2, '\nV3_s2', V3_s2, '\nT03_s2', T03_s2, '\nT_rat_s2', T_rat_s2, '\nP_rat_s2', P_rat_s2, '\nP03_s2', P03_s2)

# mean stage 3 calculations
    # find percent step of radius from stage 1 to 2, apply same step to estimate m3
r12_step = r_m2_s2/r_m1_s1
r_m3 = r_m2_s2*r12_step
    # leftover work required to power compressor
w3 = w_turb - w2_s2 - w1_s1
print('w3 ============= ',w3)
b2_s3, b3_s3, a2_s3, a3_s3, U_s3, Cw1_s3, C1_s3, T013_s3, Ca2_s3, Cw2_s3, C2_s3, V2_s3, Ca3_s3, Cw3_s3, C3_s3, V3_s3, T03_s3, T_rat_s3, P_rat_s3, P03_s3, phi_s3, psi_s3, dor_s3 = final_stage_mean(T01, P01, Ca1, omega, a3_s2, r_m3, w3)
print('final stage mean','\n','\nb2_s3:',np.degrees(b2_s3), '\nb3_s3', np.degrees(b3_s3), '\na2_s3', np.degrees(a2_s3), '\na3_s3', np.degrees(a3_s3), '\nU_s3',U_s3, '\nCw1_s3', Cw1_s3, '\nC1_s3', C1_s3, '\nT013_s3', T013_s3, '\nCa2_s3', Ca2_s3, '\nCw2_s3', Cw2_s3, '\nC2_s3', C2_s3, '\nV2_s3', V2_s3, '\nCa3_s3', Ca3_s3, '\nCw3_s3', Cw3_s3, '\nC3_s3', C3_s3, '\nV3_s3', V3_s3, '\nT03_s3', T03_s3, '\nT_rat_s3', T_rat_s3, '\nP_rat_s3', P_rat_s3, '\nP03_s3', P03_s3, '\nphi_s3', phi_s3, '\npsi_s3', psi_s3, '\ndor_s3', dor_s3)

def root_tip(Ca1, Cw2, r_m, omega, gamma, b3, T01, P01, cp, R, mdot):
    C1 = Ca1
    T1 = T01 - C1**2/(2*cp)
    P1 = P01*(T1/T01)**(gamma/(gamma-1))
    rho = P1*100000/(R*T1)
    A = mdot/(rho*Ca1)
    h = A/(2*np.pi*r_m)
    r_r = r_m - h/2
    r_t = r_m + h/2

    U_r = omega*r_r
    U_t = omega*r_t
    m_r = r_m/r_r
    m_t = r_m/r_t
    
    Cw2r = Cw2*m_r
    a2r = atan(Cw2r/Ca1)
    b2r = atan((Cw2r - U_r)/Ca1)
    C2r = Ca1/cos(a2r)
    V2r = Ca1/cos(b2r)
    phi_r = Ca1/U_r
    psi_r = 2*Ca1*(tan(b2r)+tan(b3))/U_r
    dor_r = Ca1*(tan(b3)-tan(b2r))/(2*U_r)


    Cw2t = Cw2*m_t
    a2t = atan(Cw2t/Ca1)
    b2t = atan((Cw2t - U_t)/Ca1)
    C2t = Ca1/cos(a2t)
    V2t = Ca1/cos(b2t)
    phi_t = Ca1/U_t
    psi_t = 2*Ca1*(tan(b2t)+tan(b3))/U_t
    dor_t = Ca1*(tan(b3)-tan(b2t))/(2*U_t)

    return Cw2t, C2r, V2r, phi_r, psi_r, dor_r, Cw2t, C2t, V2t, phi_t, psi_t, dor_t, r_r, r_t, a2r, b2r, a2t, b2t


# stage 1 root tip
Cw2r_s1, C2r_s1, V2r_s1, phi_r_s1, psi_r_s1, dor_r_s1, Cw2t_s1, C2t_s1, V2t_s1, phi_t_s1, psi_t_s1, dor_t_s1, r_r_s1, r_t_s1, a2r_s1, b2r_s1, a2t_s1, b2t_s1 = root_tip(Ca1, Cw2_s1, r_m1_s1, omega, gamma, b3_s1, T01, P01, cp, R, mdot_core)
print('stage 1 root tip','\n','\nCw2r_s1:', Cw2r_s1, '\nC2r_s1:',C2r_s1,'\nV2r_s1:', V2r_s1, '\nphi_r_s1:', phi_r_s1, '\npsi_r_s1:', psi_r_s1, '\ndor_r_s1:', dor_r_s1, '\nCw2t_s1:', Cw2t_s1, '\nC2t_s1:',C2t_s1, '\nV2t_s1:', V2t_s1, '\nphi_t_s1:', phi_t_s1, '\npsi_t_s1:', psi_t_s1, '\ndor_t_s1:', dor_t_s1, '\nr_r_s1:', r_r_s1, '\nr_t_s1:', r_t_s1, '\na2r_s1:', np.degrees(a2r_s1), '\nb2r_s1:', np.degrees(b2r_s1), '\na2t_s1:', np.degrees(a2t_s1), '\nb2t_s1:', np.degrees(b2t_s1))

# stage 2 root tip 
Cw2r_s2, C2r_s2, V2r_s2, phi_r_s2, psi_r_s2, dor_r_s2, Cw2t_s2, C2t_s2, V2t_s2, phi_t_s2, psi_t_s2, dor_t_s2, r_r_s2, r_t_s2, a2r_s2, b2r_s2, a2t_s2, b2t_s2 = root_tip(Ca1, Cw2_s2, r_m2_s2, omega, gamma, b3_s2, T01, P01, cp, R, mdot_core)
print('stage 2 root tip','\n','\nCw2r_s2:', Cw2r_s2, '\nC2r_s2:',C2r_s2,'\nV2r_s2:', V2r_s2, '\nphi_r_s2:', phi_r_s2, '\npsi_r_s2:', psi_r_s2, '\ndor_r_s2:', dor_r_s2, '\nCw2t_s2:', Cw2t_s2, '\nC2t_s2:',C2t_s2, '\nV2t_s2:', V2t_s2, '\nphi_t_s2:', phi_t_s2, '\npsi_t_s2:', psi_t_s2, '\ndor_t_s2:', dor_t_s2, '\nr_r_s2:', r_r_s2, '\nr_t_s2:', r_t_s2, '\na2r_s2:', np.degrees(a2r_s2), '\nb2r_s2:', np.degrees(b2r_s2), '\na2t_s2:', np.degrees(a2t_s2), '\nb2t_s2:', np.degrees(b2t_s2))

# need stage 3 root tip