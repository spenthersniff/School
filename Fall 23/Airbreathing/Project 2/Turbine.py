import numpy as np
cos=np.cos
sin=np.sin
tan=np.tan
atan=np.arctan

T01 = 2275.0              # turbine inlet temp, K
T03 = 755.329018          # compressor exit temp, K
P01 = 19.22525            # turbine inlet pressure, bar
n_inf = 0.92              # polytropic turbine efficiency
w_comp = 437774.06        # compressor work required, J/kg
mdot_a = 280.0            # mass flow of air, kg/s
n_stages = 3.0            # number of turbine stages
cp = 1148.0               # for hot gas, J/(kg*K)
gamma = 1.333             # specific heat ratio
Ca1 = 280.0               # axial inlet speed, m/s
n_m = 0.99                # mechanical efficiency
n = 240.0433              # rotational speed, rev/s
omega = 2*np.pi*n         # rotational speed, rad/s
a1 = np.radians(0.0)      # inlet relative gas angle, radians
R = 287                   # J/(kg*k)

f = (cp*T01-1005*T03)/(43100000-cp*T01)
mdot_f = f*mdot_a         # fuel mass flow, kg/s
mdot = mdot_a+mdot_f      # total mass flow, kg/s

def gas_angles(phi, psi, dor):
    b2 = atan((1/(2*phi))*((psi/2)-2*dor))
    b3 = atan((1/(2*phi))*((psi/2)+2*dor))
    a2 = atan(tan(b2)+(1/phi))
    a3 = atan(tan(b3)-(1/phi))
    return b2, b3, a2, a3

def meanline(T01, P01, Ca1, phi, psi, dor, omega):
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
b2_s1, b3_s1, a2_s1, a3_s1, U_s1, r_m1_s1, Cw1_s1, C1_s1, T013_s1, w1_s1, Ca2_s1, Cw2_s1, C2_s1, V2_s1, Ca3_s1, Cw3_s1, C3_s1, V3_s1, T03_s1, T_rat_s1, P_rat_s1, P03_s1 = meanline(T01, P01, Ca1, phi1_m, psi1_m, dor1_m, omega)
print(np.degrees(a3_s1))

# mean stage 2 calculations
phi2_m = 0.78              # flow coefficient, >=0.78
psi2_m = 3.3               # blade loading (temp drop) coefficient, <=3.3
dor2_m = 0.5               # degree of reaction
b2_s2, b3_s2, a2_s2, a3_s2, U_s2, r_m2_s2, Cw1_s2, C1_s2, T013_s2, w2_s2, Ca2_s2, Cw2_s2, C2_s2, V2_s2, Ca3_s2, Cw3_s2, C3_s2, V3_s2, T03_s2, T_rat_s2, P_rat_s2, P03_s2 = meanline(T01, P01, Ca1, phi2_m, psi2_m, dor2_m, omega)

# mean stage 3 calculations
    # find percent step of radius from stage 1 to 2, apply same step to estimate m3
r12_step = r_m2_s2/r_m1_s1
r_m3 = r_m2_s2*r12_step
    # leftover work required to power compressor
w3 = w_comp - w2_s2 - w1_s1
b2_s3, b3_s3, a2_s3, a3_s3, U_s3, Cw1_s3, C1_s3, T013_s3, Ca2_s3, Cw2_s3, C2_s3, V2_s3, Ca3_s3, Cw3_s3, C3_s3, V3_s3, T03_s3, T_rat_s3, P_rat_s3, P03_s3, phi_s3, psi_s3, dor_s3 = final_stage_mean(T01, P01, Ca1, omega, a1, r_m3, w3)

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

    return Cw2t, C2r, V2r, phi_r, psi_r, dor_r, Cw2t, C2t, V2t, phi_t, psi_t, dor_t, r_r, r_t


# stage 1 root tip
Cw2t_s1, C2r_s1, V2r_s1, phi_r_s1, psi_r_s1, dor_r_s1, Cw2t_s1, C2t_s1, V2t_s1, phi_t_s1, psi_t_s1, dor_t_s1, r_r_s1, r_t_s1 = root_tip(Ca1, Cw2_s1, r_m1_s1, omega, gamma, b3_s1, T01, P01, cp, R, mdot)

# stage 2 root tip 
Cw2t_s2, C2r_s2, V2r_s2, phi_r_s2, psi_r_s2, dor_r_s2, Cw2t_s2, C2t_s2, V2t_s2, phi_t_s2, psi_t_s2, dor_t_s2, r_r_s2, r_t_s2 = root_tip(Ca1, Cw2_s2, r_m2_s2, omega, gamma, b3_s2, T01, P01, cp, R, mdot)