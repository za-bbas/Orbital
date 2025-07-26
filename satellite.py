from planet import *
from math import acos, atan2, pi
import numpy as np
import numpy.linalg as la
from datetime import datetime
import ppigrf

'''
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
    rho = la.norm(r)
    rhat = r / rho
    Fgrav = -(G * M * m / rho**2) * rhat

    # Call the magnetic field model
    # Convert cartesian x,y,x to lat, long, alt
    phiE = 0
    thetaE = acos(z / rho)              # colatitude (rad)
    psiE = atan2(y, x)                  # longitude  (rad)
    latitude = 90 - thetaE * 180 / pi   # degrees
    longitude = psiE * 180 / pi         # degrees
    altitude = (rho - R)/1000           # kilometers
    Be, Bn, Bu = ppigrf.igrf(longitude, latitude, altitude, datetime(2000, 1, 1)) # East, North, Up (ENU) convention...
    # Why not just use cartesian :-(
    # geocentric maybe?

    # Br, Btheta, Bphi = ppigrf.igrf_gc(rho/1000, thetaE * 180 / pi, psiE * 180 / pi, datetime(2000,1,1))
    # Br: radial B field
    # Btheta: B field in direction of increasing theta (effectivily south)
    # Bphi: B field in direction of increasing phi (effectively east)
    
    # dynamics
    F = Fgrav
    accel = F/m

    return np.concatenate((vel, accel))
'''

class Satellite:
    def __init__(self, mass = 6):
        self.m = mass
        self.lastBField = None  # Store last B-field for access
        self.lastPosition = None

    def dStateDT(self, t, state):
        x, y, z, xdot, ydot, zdot = state
        vel = np.array([xdot, ydot, zdot])

        # gravity model
        r = np.array([x,y,z])
        rho = la.norm(r)
        rhat = r / rho
        Fgrav = -(G * M * self.m / rho**2) * rhat

        # Call the magnetic field model
        # Convert cartesian x,y,x to lat, long, alt
        phiE = 0
        thetaE = np.arccos(z / rho)         # colatitude (rad)
        psiE = np.arctan2(y, x)             # longitude (rad)
        latitude = 90 - thetaE * 180 / pi   # degrees
        longitude = psiE * 180 / pi         # degrees
        altitude = (rho - R)/1000           # kilometers
        Be, Bn, Bu = ppigrf.igrf(longitude, latitude, altitude, datetime(2000, 1, 1)) # East, North, Up (ENU) convention...

        # Br, Btheta, Bphi = ppigrf.igrf_gc(rho/1000, thetaE * 180 / pi, psiE * 180 / pi, datetime(2000,1,1))
        # Br: radial B field
        # Btheta: B field in direction of increasing theta (effectivily south)
        # Bphi: B field in direction of increasing phi (effectively east)
        self.lastBField = np.array([Be, Bn, Bu])
        self.lastPosition = r

        # Dynamics
        accel = Fgrav / self.m
        return np.concatenate((vel, accel))
    
    def get_B_inertial(self, t, state):
        x, y, z = state[:3]
        rho = la.norm([x, y, z])
        thetaE = np.arccos(z / rho)
        psiE = np.arctan2(y, x)
        phiE = 0

        latitude = 90 - thetaE * 180 / pi
        longitude = psiE * 180 / pi
        altitude = (rho - R) / 1000  # km

        # Get magnetic field in ENU
        Be, Bn, Bu = ppigrf.igrf(longitude, latitude, altitude, datetime(2000, 1, 1))


        # Build transformation matrix from ENU to ECI (inertial for simplicity)
        # i.e. TIB matrix
        cph = np.cos(phiE)
        cps = np.cos(psiE)
        cth = np.cos(thetaE + np.pi)
        sph = np.sin(phiE)
        sps = np.sin(psiE)
        sth = np.sin(thetaE + np.pi)

        # matrix for converting from north east down to inertial coordinates
        # this is the matrix transformation from the spherical reference frame to the inertial refrence frame
        TIB = np.array([
            [cth*cps, sph*sth*cps - cph*sps, cph*sth*cps + sph*sps],
            [cth*sps, sph*sth*sps + cph*cps, cph*sth*sps - sph*cps],
            [ -1*sth, sph*cth,                             cph*cth]
        ])

        # The matrix is goes from NED (as opposed to the IGRF model output of ENU) to inertial
        B_ned = np.array([Bn, Be, -1 * Bu])
        B_xyz = TIB @ B_ned  # Now in ECI
        return B_xyz

    def getLastBField(self):
        return self.last_B_field

    def getLastPosition(self):
        return self.last_position