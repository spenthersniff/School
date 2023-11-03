import numpy as np
import matplotlib.pyplot as plt
import control as ctl
from control.matlab import *
import parameters.VTOLParam as P

# create transfer function numerator and demominator
kDh = 0.5835
kPh = 0.11345
ki = 1
num = [1]
den = ki*[P.mc + 2*P.mr, kDh, kPh, 0.]

# run transfer function
sys = ctl.tf(num, den)
plt.figure("Height")
ctl.rlocus(sys)

# create transfer function numerator and demominator
kDt = -0.032858
kPt = -0.007095
ki = 1
num = [1]
Fe = 9.8*(P.mc + 2*P.mr)
den = ki*[(P.mc + 2*P.mr)/-Fe, kDt + P.mu/-Fe, kPt, 0.]

# run transfer function
sys = ctl.tf(num, den)
plt.figure("Z")
ctl.rlocus(sys)

plt.show()