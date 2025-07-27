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

# the whole euler angles thing kind makes me blegh but its good enough for now
def eulerToQuat(angles):
    phi = angles[0]
    theta = angles[1]
    psi = angles[2]

    cph = np.cos(phi / 2)
    cth = np.cos(theta / 2)
    cps = np.cos(psi / 2)
    sph = np.sin(phi / 2)
    sth = np.sin(theta / 2)
    sps = np.sin(psi / 2)

    q0 = cph * cth * cps + sph * sth * sps
    q1 = sph * cth * cps - cph * sth * sps
    q2 = cph * sth * cps + sph * cth * sps
    q3 = cph * cth * sps - sph * sth * cps
    
    return np.array([q0, q1, q2, q3])

def quatToEuler(quat):
    q0, q1, q2, q3 = quat
    phi = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
    theta = np.arcsin(2*(q0*q2 - q1*q3))
    psi = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

    return np.array([phi, theta, psi])

# Simulation of a low earth satellite
print('Simulation started.')

sat = Satellite()

# Initial conditions (translational)
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

# Initial conditions (rotational)
# initial attitude (could use some revamping)
phi0 = 0
theta0 = 0
psi0 = 0
ptp0 = np.array([phi0, theta0, psi0])
quat0 = eulerToQuat(ptp0)
# initial angular velocity
p0 = 0
q0 = 0
r0 = 0

# State vector
stateInitial = np.array([x0, y0, z0, 
                         xdot0, ydot0, zdot0, 
                         quat0[0], quat0[1], quat0[2], quat0[3],
                         p0, q0, r0])

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
stateout_km = stateout[0:6] / 1000

# Extract x, y, z components
xout = stateout_km[:, 0]
yout = stateout_km[:, 1]
zout = stateout_km[:, 2]
# ISSUE: it isnt converting every state into ptp
# the stupid function is stupid
# gotta figure out a solution
quatout = stateout[:, 6:10]
ptpout = quatToEuler(quatout)
pqrout = stateout[:, 10:13]

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

fig3, ax3 = plt.subplots(figsize=(10,6))
fig3.patch.set_facecolor('white')
ax3.plot(tout, ptpout[0], label='phi',color='blue')
ax3.plot(tout, ptpout[1], label='theta',color='blue')
ax3.plot(tout, ptpout[2], label='psi',color='blue')
ax3.set_title("Euler Angles throughout an orbit")
ax3.set_xlabel("Time")
ax3.set_ylabel("something to do with angles")
ax3.grid(True)
ax3.legend

plt.show()