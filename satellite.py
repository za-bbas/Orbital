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

        latitude = 90 - thetaE * 180 / pi
        longitude = psiE * 180 / pi
        altitude = (rho - R) / 1000  # km

        # Get magnetic field in ENU
        Be, Bn, Bu = ppigrf.igrf(longitude, latitude, altitude, datetime(2000, 1, 1))

        # Convert ENU to ECEF (XYZ)
        # ENU basis at this lat/lon:
        # E -> x' = -sin(lon), y' = cos(lon), z' = 0
        # N -> x' = -sin(lat)*cos(lon), y' = -sin(lat)*sin(lon), z' = cos(lat)
        # U -> x' = cos(lat)*cos(lon), y' = cos(lat)*sin(lon), z' = sin(lat)

        lat_rad = latitude * pi / 180
        lon_rad = longitude * pi / 180

        # Build transformation matrix from ENU to ECEF (inertial for simplicity)
        T = np.array([
            [-np.sin(lon_rad), -np.sin(lat_rad) * np.cos(lon_rad),  np.cos(lat_rad) * np.cos(lon_rad)],
            [ np.cos(lon_rad), -np.sin(lat_rad) * np.sin(lon_rad),  np.cos(lat_rad) * np.sin(lon_rad)],
            [0,                 np.cos(lat_rad),                    np.sin(lat_rad)]
        ])

        B_enu = np.array([Be, Bn, Bu])
        B_xyz = T @ B_enu  # Now in ECEF/inertial
        return B_xyz

    def getLastBField(self):
        return self.last_B_field

    def getLastPosition(self):
        return self.last_position