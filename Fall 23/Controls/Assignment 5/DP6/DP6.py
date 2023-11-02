import numpy as np
import matplotlib.pyplot as plt
import control as ctl
from control.matlab import *
import MSDParam as P

# Create transfer function numerator and denominator
kD = 7.2
kP = 3.05
num = [1]
den = [P.m, (P.b + kD), (P.k + kP), 0.]
# Run transfer function
sys = ctl.tf(num, den)

# Create root locus plot
ctl.rlocus(sys)
plt.show()