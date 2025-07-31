import numpy as np
# Satellite inertial parameters
mass = 4                           # kilograms
Inertia = np.array([
    [0.9,   0,   0],
    [  0, 0.9,   0],
    [  0,   0, 0.3]
])                                 # kg m^2
# Satellite orbital parameters
altitude = 600e3                   # meters
inclination = 56 * np.pi / 180     # radians
numberOfOrbits = 1
# Satellite rotational paramters
# Attitude (euler angles):
phi0 = 0                           # radians
theta0 = 0                         # radians
psi0 = 0                           # radians
# Omega:
p0 = 0.08                          # rad/s
q0 = -0.02                         # rad/s
r0 = 0.015                         # rad/s
# Simulation Parameters
timeStep = 5                       # seconds

# Time to run simulation with diff time steps:
# timeStep = 10: 30-35 sec
# timeStep =  5: 60-70 sec
# timeStep =  1:  >300 sec