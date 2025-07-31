import numpy as np
# Satellite inertial parameters
mass = 4                           # kilograms
Inertia = np.array([
    [0.9,   0,   0],
    [  0, 0.9,   0],
    [  0,   0, 0.3]
])                                 # kg m^2
# Satellite orbital parameters
altitude = 400e3                   # meters
inclination = 30 * np.pi / 180     # radians
numberOfOrbits = 1
# Satellite rotational paramters
# Attitude (euler angles):
phi0 = 0                           # radians
theta0 = 0                         # radians
psi0 = 0                           # radians
# Omega:
p0 = 0.4                          # rad/s
q0 = -0.2                         # rad/s
r0 = 0.15                         # rad/s
# Simulation Parameters
timeStep = 3                      # seconds

# Time to run simulation with diff time steps:
# timeStep = 10: 30-35 sec
# timeStep =  5: 60-70 sec
# timeStep =  1:  >300 sec