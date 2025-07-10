from planet import *
from math import sqrt, pi, sin, cos
import numpy as np
import numpy.linalg as la

# Simulation of a low earth satellite
print('Simulation started.')

# Initial conditions
altitude = 400e3    # meters
inclination = 0     # radians

x0 = R + altitude
y0 = 0
z0 = 0
semimajor = la.norm(np.array([x0, y0, z0]))  
vCircular = sqrt(mu/semimajor)
xdot0 = 0           # velocity normal to earth
ydot0 = vCircular * cos(inclination)
zdot0 = vCircular * sin(inclination)

stateinitial = np.array([x0, y0, z0, xdot0, ydot0, zdot0])

# time window
  # semimajor axis of orbit
period = 2 * pi * sqrt(semimajor**3 / mu)
numberOfOrbits = 1
tspan = [0, period * numberOfOrbits]



print("didnt break")
# Integrate equations of motion
# in matlab, Id be using ode45...
# [tout, stateout] = ode45(@satellite, tspan, stateinitial)