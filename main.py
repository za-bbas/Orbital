from planet import *
from params import *
from utils import *
from satellite import Satellite
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
import time # for benchmarking


# Simulation of a low earth satellite
print('Simulation started.')
startTime = time.time()


sat = Satellite()

# Initial conditions (translational)
x0 = R + altitude
y0 = 0
z0 = 0
semimajor = la.norm(np.array([x0, y0, z0]))         # semimajor axis of orbit
vCircular = np.sqrt(mu/semimajor)                   # tangential velocity
xdot0 = 0                                           # velocity normal to earth so xdot = 0
ydot0 = vCircular * np.cos(inclination)
zdot0 = vCircular * np.sin(inclination)

# Initial conditions (rotational)
ptp0 = np.array([phi0, theta0, psi0])
quat0 = eulerToQuat(ptp0)

# State vector
state = np.array([x0, y0, z0, 
                  xdot0, ydot0, zdot0, 
                  quat0[0], quat0[1], quat0[2], quat0[3],
                  p0, q0, r0])

# time window
period = 2 * np.pi * np.sqrt(semimajor**3 / mu)
tFinal = period * numberOfOrbits
tOut = np.arange(0, tFinal + timeStep, timeStep)
stateout = np.zeros((len(tOut), len(state)))

nextPrint = 1
lastPrint = 0

for idx in range (len(tOut)):
    stateout[idx] = state

    if tOut[idx] % 100 == 0:
        print("Time is ", tOut[idx])

    # RK 4 algorithm
    k1 = sat.dStateDT(tOut[idx], state)
    k2 = sat.dStateDT(tOut[idx] + timeStep/2, state + k1 * timeStep/2)
    k3 = sat.dStateDT(tOut[idx] + timeStep/2, state + k2 * timeStep/2)
    k4 = sat.dStateDT(tOut[idx] + timeStep  , state + k3 * timeStep)
    k = (k1 + 2*k2 + 2*k3 + k4) / 6
    state = state + k * timeStep


print("Completed simulation.")
endTime = time.time()
elapsed = endTime - startTime
print(f"Elapsed time: {elapsed:.3f} seconds")

# Convert stateout from meters to kilometers
xout = stateout[:, 0] / 1000  # x in km
yout = stateout[:, 1] / 1000  # y in km
zout = stateout[:, 2] / 1000  # z in km

# get attitude information
quatout = stateout[:, 6:10]
# Convert each quaternion to Euler angles for plotting
ptpout = np.array([quatToEuler(q) for q in quatout])
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


fig3, ax3 = plt.subplots(figsize=(10,6))
fig3.patch.set_facecolor('white')
ax3.plot(tOut, ptpout[:,0], label='phi',color='blue')
ax3.plot(tOut, ptpout[:,1], label='theta',color='green')
ax3.plot(tOut, ptpout[:,2], label='psi',color='red')
ax3.set_title("Euler Angles throughout an orbit")
ax3.set_xlabel("Time")
ax3.set_ylabel("Angles (rad)")
ax3.grid(True)
ax3.legend()

fig4, ax4 = plt.subplots(figsize=(10,6))
fig4.patch.set_facecolor('white')
ax4.plot(tOut, pqrout[:,0], label='p',color='blue')
ax4.plot(tOut, pqrout[:,1], label='q',color='green')
ax4.plot(tOut, pqrout[:,2], label='r',color='red')
ax4.set_title("Angular Velocities")
ax4.set_xlabel("Time")
ax4.set_ylabel("Anglular Velocity (rad/s)")
ax4.grid(True)
ax4.legend()

plt.show()