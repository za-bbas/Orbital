import numpy as np
# Satellite inertial parameters
mass = 6                           # kilograms
Inertia = np.array([
    [0.01,   0,   0],
    [  0, 0.05,   0],
    [  0,   0, 0.05]
])                                 # kg m^2
# Satellite orbital parameters
altitude = 400e3                   # meters
inclination = 0 * np.pi / 180     # radians
numberOfOrbits = 17
# Satellite rotational paramters
# Attitude (euler angles):
phi0 = 0                           # radians
theta0 = 0                         # radians
psi0 = 0                           # radians
# Omega:
p0 = 0.4                          # rad/s
q0 = -0.3                         # rad/s
r0 = 0.15                        # rad/s
# Simulation Parameters
timeStep = 2                      # seconds