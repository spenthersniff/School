# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum known to the controller
# m = 5     # Mass of the block, kg
g = 9.8       # Gravity, m/s**2
# b = 0.5      # Damping coefficient, Ns/m
# k=3         # Spring Constant, N/m

m = np.random.uniform(0.8*5, 1.2*5)
k = np.random.uniform(0.8*3, 1.2*3)
b = np.random.uniform(0.8*0.5, 1.2*0.5)

# parameters for animation
w = 0.5       # Width of the block, m
h = 0.5      # Height of the block, m
gap = 0.005   # Gap between the block and x-axis

# Initial Conditions
z0 = 0.0                # ,m
zdot0 = 0.0             # ,m/s
f0 = 2                  # N

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 500.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate

# saturation limits
F_max = 6.0                # Max Force, N

kd = 7.2
kp = 3.05
ki = 0.4