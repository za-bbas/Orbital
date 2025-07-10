from planet import *
from math import sqrt
import numpy as np
import numpy.linalg as la

m = 6       # kg

def dStateDT(t, state):
    x = state[0]
    y = state[1]
    z = state[2]
    xdot = state[3]
    ydot = state[4]
    zdot = state[5]

    # Kinematics
    vel = np.array([xdot, ydot, zdot])

    # gravity model
    r = np.array([x,y,z])
    rnorm = la.norm(r)
    rhat = r / rnorm
    Fgrav = -(G * M * m / rnorm**2) * rhat

    # dynamics
    F = Fgrav
    accel = F/m

    return np.array([vel, accel])