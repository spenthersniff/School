import numpy as np
from trim.compute_trim import ComputeTrim
import parameters.aerosonde_parameters as P


jx = P.jx
jy = P.jy
jz = P.jz
jxz = P.jxz
gravity = P.gravity
m = P.mass

g = P.g
g1 = P.g1
g2 = P.g2
g3 = P.g3
g4 = P.g4
g5 = P.g5
g6 = P.g6
g7 = P.g7
g8 = P.g8

pn = P.states0[0][0]
pe = P.states0[1][0]
pd = P.states0[2][0]
u = P.states0[3][0]
v = P.states0[4][0]
w = P.states0[5][0]
phi = P.states0[6][0]
theta = P.states0[7][0]
psi = P.states0[8][0]
p = P.states0[9][0]
q = P.states0[10][0]
r = P.states0[11][0]

S_wing = P.S_wing
b = P.b
c = P.c
S_prop = P.S_prop
rho = P.rho
e = P.e
AR = P.AR
C_L_0 = P.C_L_0
C_D_0 = P.C_D_0
C_m_0 = P.C_m_0
C_L_alpha = P.C_L_alpha
C_D_alpha = P.C_D_alpha
C_m_alpha = P.C_m_alpha
C_L_q = P.C_L_q
C_D_q = P.C_D_q
C_m_q = P.C_m_q
C_L_delta_e = P.C_L_delta_e
C_D_delta_e = P.C_D_delta_e
C_m_delta_e = P.C_m_delta_e
M = P.M
alpha0 = P.alpha0
epsilon = P.epsilon
C_D_p = P.C_D_p
C_Y_0 = P.C_Y_0
C_ell_0 = P.C_ell_0
C_n_0 = P.C_n_0
C_Y_beta = P.C_Y_beta
C_ell_beta = P.C_ell_beta
C_n_beta = P.C_n_beta
C_Y_p = P.C_Y_p
C_ell_p = P.C_ell_p
C_n_p = P.C_n_p
C_Y_r = P.C_Y_r
C_ell_r = P.C_ell_r
C_n_r = P.C_n_r
C_Y_delta_a = P.C_Y_delta_a
C_ell_delta_a = P.C_ell_delta_a
C_n_delta_a = P.C_n_delta_a
C_Y_delta_r = P.C_Y_delta_r
C_ell_delta_r = P.C_ell_delta_r
C_n_delta_r = P.C_n_delta_r
C_prop = P.C_prop
k_motor = P.k_motor
k_tp = P.k_tp
k_omega = P.k_omega
        

trim=ComputeTrim()
Va=35.0
Y=np.radians(0)
R=np.inf

x_trim, u_trim = trim.compute_trim(Va, Y, R)


Va_trim = np.sqrt(x_trim[3] ** 2 + x_trim[4] ** 2 + x_trim[5] ** 2)
alpha_trim = np.arctan(x_trim[5] / x_trim[3])
beta_trim = np.arctan(x_trim[4]/Va_trim)
theta_trim = x_trim[7]
Cpp = g3 * C_ell_p + g4 * C_n_p
Cpdelta_a = g3 * C_ell_delta_a + g4 * C_n_delta_a

# Constants
a_phi1 = -.5 * rho * Va_trim**2 * S_wing * b * Cpp * (b / (2 * Va_trim))
a_phi2 = .5 * rho * Va_trim**2 * S_wing * b * Cpdelta_a
a_theta1 = -(rho * Va_trim**2 * c * S_wing)/(2 * jy) * C_m_q * (c/(2 * Va_trim))
a_theta2 = -(rho * Va_trim**2 * c * S_wing)/(2 * jy) * C_m_alpha
a_theta3 = (rho * Va_trim**2 * c * S_wing)/(2 * jy) * C_m_delta_e

a_V1 = ((rho * Va_trim * S_wing)/m) * (C_D_0 + (C_D_alpha * alpha_trim) + (C_D_delta_e * u_trim[0])) + (rho * S_prop)/(m) * C_prop * Va_trim
a_V2 = (rho * S_prop)/(m) * C_prop * k_motor**2 * u_trim[3]
a_V3 = gravity * np.cos(theta_trim - alpha_trim)

a_beta1 = -(rho * Va_trim * S_wing)/(2 * m) * C_Y_beta
a_beta2 = (rho * Va_trim * S_wing)/(2 * m) * C_Y_delta_r



# Va_trim = np.sqrt(x_trim[3] ** 2 + x_trim[4] ** 2 + x_trim[5] ** 2)
# alpha_trim = np.arctan(x_trim[5] / x_trim[3])
# beta_trim = np.arctan(x_trim[4]/Va_trim)
# theta_trim = x_trim[7]
# Cpp = g3 * C_ell_p + g4 * C_n_p
# Cpdelta_a = g3 * C_ell_delta_a + g4 * C_n_delta_a

# a_phi1 = -.5 * rho * Va_trim**2 * S_wing * b * Cpp * (b / (2 * Va_trim))
# a_phi2 = .5 * rho * Va_trim**2 * S_wing * b * Cpdelta_a
# a_theta1 = -(rho * Va_trim**2 * c * S_wing)/(2 * jy) * C_m_q * (c/(2 * Va_trim))
# a_theta2 = -(rho * Va_trim**2 * c * S_wing)/(2 * jy) * C_m_alpha
# a_theta3 = (rho * Va_trim**2 * c * S_wing)/(2 * jy) * C_m_delta_e


# lateral
zeta = .707
# roll
tr_roll = 0.5
wn_roll = 2.2/tr_roll
kp_roll = wn_roll**2/a_phi2
kd_roll = (2*zeta*wn_roll - a_phi1)/a_phi2
ki_roll = 0

# Course hold
tr_course = 1
wn_course = 2.2/tr_course
kp_course = (2 * zeta * wn_course * Va_trim)/gravity
kd_course = 0
ki_course = (wn_course**2 * Va_trim)/gravity


# Pitch Attitude Hold
zeta_pitch = .1
tr_pitch = .1
wn_pitch = 2.2/tr_pitch
kp_pitch = (wn_pitch**2 - a_theta2)/a_theta3
kd_pitch = (2 * zeta_pitch * wn_pitch - a_theta1)/a_theta3
ki_pitch = 0
ktheta_DC = (kp_pitch * a_theta3)/(a_theta2 + kp_pitch * a_theta3)

# altitude from Pitch Gain
tr_altitude = 2.0
wn_altitude = 2.2 / tr_altitude
kp_altitude = (2 * zeta * wn_altitude) / (ktheta_DC * Va_trim)
kd_altitude = 0
ki_altitude = wn_altitude**2 / (ktheta_DC * Va_trim)

# airspeed from pitch
tr_airspeed = 5.0
wn_airspeed = 2.2/tr_airspeed
kp_airspeed = (a_V1 - 2 * zeta * wn_airspeed) / ktheta_DC
kd_airspeed = 0
ki_airspeed = wn_airspeed**2 / (ktheta_DC * gravity)

# Airspeed from Throttle
tr_throttle = 15
wn_throttle = 2.2 / tr_throttle
kp_throttle = (2 * zeta * wn_throttle - a_V1)/a_V2
kd_throttle = 0
ki_throttle = wn_throttle**2 / a_V2

kp = np.array([kp_roll, kp_course, kp_pitch, kp_altitude, kp_airspeed, kp_throttle])
kd = np.array([kd_roll, kd_course, kd_pitch, kd_altitude, kd_airspeed, kd_throttle])
ki = np.array([ki_roll, ki_course, ki_pitch, ki_altitude, ki_airspeed, ki_throttle])