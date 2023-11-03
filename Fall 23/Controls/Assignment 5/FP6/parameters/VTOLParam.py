# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum known to the controller
mc = 1     # Mass of , kg
jc=0.0042
mr=0.25         # mass of , kg
ml=0.25        # mass of, kg
d=0.3           # arm length, m
mu=0.1         # kg/s
g = 9.8       # Gravity, m/s**2


# parameters for animation
wb = 0.5       # Width of the block, m
hb = 0.5      # Height of the block, m

# Initial Conditions
z0 = 0.0                # ,m
h0 = 5.0
theta0 = 0.0
zdot0 = 0.0             # ,m/s
hdot0 = 0.0
thetadot0 = 0.0
# f0 = 2                  # N

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 500.0  # End time of simulation
Ts = 0.1  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate

# saturation limits
F_max = 10.0                # Max Force, N