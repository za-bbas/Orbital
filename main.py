from planet import *
from satellite import *
from math import sqrt, pi, sin, cos
import numpy as np
import numpy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # required for 3D projection
from matplotlib import cm


# Simulation of a low earth satellite
print('Simulation started.')

# Initial conditions
altitude = 400e3                                # meters
inclination = 51 * pi / 180                     # radians

x0 = R + altitude
y0 = 0
z0 = 0
semimajor = la.norm(np.array([x0, y0, z0]))     # semimajor axis of orbit
vCircular = sqrt(mu/semimajor)                  # tangential velocity
xdot0 = 0                                       # velocity normal to earth so xdot = 0
ydot0 = vCircular * cos(inclination)
zdot0 = - vCircular * sin(inclination)

# State vector
stateInitial = np.array([x0, y0, z0, xdot0, ydot0, zdot0])

# time window
period = 2 * pi * sqrt(semimajor**3 / mu)
numberOfOrbits = 1
tSpan = [0, period * numberOfOrbits]

# Integrate equations of motion
solution = solve_ivp(
    fun=dStateDT,
    t_span=tSpan,
    y0=stateInitial,
    method='RK45',
    rtol=1e-8,
    atol=1e-10
)

tout = solution.t
stateout = solution.y.T


# Convert stateout from meters to kilometers
stateout_km = stateout / 1000

# Extract x, y, z components
xout = stateout_km[:, 0]
yout = stateout_km[:, 1]
zout = stateout_km[:, 2]

# Create a sphere to represent the Earth
u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
U, V = np.meshgrid(u, v)

# dimensions in km
X = (R / 1000) * np.cos(U) * np.sin(V)
Y = (R / 1000) * np.sin(U) * np.sin(V)
Z = (R / 1000) * np.cos(V)

# Plotting
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.patch.set_facecolor('white')

# Plot the satellite trajectory
ax.plot3D(xout, yout, zout, 'b-', linewidth=2)

# Plot the Earth
ax.plot_surface(X, Y, Z, rstride=4, cstride=4, color='lightblue', edgecolor='none', alpha=0.6)

# Formatting
ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
ax.grid(True)
ax.set_xlabel('X (km)')
ax.set_ylabel('Y (km)')
ax.set_zlabel('Z (km)')
plt.title("Satellite Orbit Around Earth")

plt.show()