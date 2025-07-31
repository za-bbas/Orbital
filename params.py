import numpy as np
# Satellite inertial parameters
mass = 4
Inertia = np.array([
    [0.9,   0,   0],
    [  0, 0.9,   0],
    [  0,   0, 0.3]
])
# Satellite orbital parameters
altitude = 600e3
inclination = 56 * np.pi / 180
numberOfOrbits = 1
# Satellite rotational paramters
# Attitude (euler angles):
phi0 = 0
theta0 = 0
psi0 = 0
# Omega:
p0 = 0.08
q0 = -0.02
r0 = 0.015
# Simulation Parameters
# Time to run simulation with different time steps:
# timeStep = 10: 30-35 sec
# timeStep =  5: 60-70 sec
# timeStep =  1:  >300 sec
timeStep = 5