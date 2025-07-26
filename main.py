from planet import *
from satellite import Satellite
from math import sqrt, pi, sin, cos
import numpy as np
import numpy.linalg as la
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D  # required for 3D projection
# from matplotlib import cm
# from datetime import datetime
# import ppigrf


# Simulation of a low earth satellite
print('Simulation started.')

sat = Satellite()

# Initial conditions
altitude = 600e3                                # meters
inclination = 56 * pi / 180                     # radians

x0 = R + altitude
y0 = 0
z0 = 0
semimajor = la.norm(np.array([x0, y0, z0]))     # semimajor axis of orbit
vCircular = sqrt(mu/semimajor)                  # tangential velocity
xdot0 = 0                                       # velocity normal to earth so xdot = 0
ydot0 = vCircular * cos(inclination)
zdot0 = vCircular * sin(inclination)

# State vector
stateInitial = np.array([x0, y0, z0, xdot0, ydot0, zdot0])

# time window
period = 2 * pi * sqrt(semimajor**3 / mu)
numberOfOrbits = 1
tSpan = [0, period * numberOfOrbits]

# Integrate equations of motion
solution = solve_ivp(
    fun=sat.dStateDT,
    t_span=tSpan,
    y0=stateInitial,
    method='RK45',
    rtol=1e-8,
    atol=1e-10
)

tout = solution.t
stateout = solution.y.T

# get magnetic field data

BxIout = []
ByIout = []
BzIout = []

for t, state in zip(tout, stateout):
    Bxyz = sat.get_B_inertial(t, state)
    BxIout.append(Bxyz[0])
    ByIout.append(Bxyz[1])
    BzIout.append(Bxyz[2])


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

# plotting magentic field throughout orbit
fig2, ax2 = plt.subplots(figsize=(10, 6))
fig2.patch.set_facecolor('white')

# Plot components and magnitude
ax2.plot(tout / 60, BxIout, label='Bx (nT)', color = 'blue')
ax2.plot(tout / 60, ByIout, label='By (nT)', color = 'green')
ax2.plot(tout / 60, BzIout, label='Bz (nT)', color = 'red')
# ax2.plot(tout / 60, Bmag, label='|B| (nT)', linestyle='--', linewidth=2)

# Formatting
ax2.set_title("Magnetic Field Components vs Time")
ax2.set_xlabel("Time (minutes)")
ax2.set_ylabel("Magnetic Field (nT)")
ax2.grid(True)
ax2.legend()


plt.show()