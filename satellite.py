from planet import *
import numpy as np
import numpy.linalg as la
from datetime import datetime
import ppigrf

class Satellite:
    def __init__(self, mass = 6):
        self.m = mass
        # Inertia in kg m^2
        self.I = np.array([
            [0.9, 0, 0],
            [0, 0.9, 0],
            [0, 0, 0.3]
        ])
        self.invI = la.inv(self.I)
        self.lastBField = None  # Store last B-field for access
        self.lastPosition = None

    def dStateDT(self, t, state):
        x, y, z, xdot, ydot, zdot = state[0:6]
        # translational kinematics
        vel = np.array([xdot, ydot, zdot])

        # rotational kinematics
        quat = state[6:10]
        p = state[10]
        q = state[11]
        r = state[12]
        pqr = np.array([p, q, r])
        PQRMAT = np.array([
            [0, -p, -q, -r],
            [p,  0,  r, -q],
            [q, -r,  0,  p],
            [r,  q, -p,  0]
        ])

        quatdot = 0.5 * PQRMAT @ quat
        quat = quat / la.norm(quat)

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
        latitude = 90 - thetaE * 180 / np.pi   # degrees
        longitude = psiE * 180 / np.pi         # degrees
        altitude = (rho - R)/1000           # kilometers
        Be, Bn, Bu = ppigrf.igrf(longitude, latitude, altitude, datetime(2000, 1, 1)) # East, North, Up (ENU) convention...

        self.lastPosition = r

        # Translatioal Dynamics
        accel = Fgrav / self.m

        # Magtorquer model

        # TODO: finish later
        '''
        MT model:
        Generated torques are basically:
        [L M N] = n*A*[Bx]i
        where i is the current vector, which we can generate one of two ways:
        traditional bdot: i = k [Bx]pqr
        or the estimate: i = -k/|B'|^2 dB'/dt
        note that B' is the b field in the body frame
        effectively meaning that our torques will look something like:
        [L M N] = k [Bx]^2 pqr
        '''
        LMN_magtorquers = np.array([0,0,0])


        # rotational dynamics
        H = self.I @ pqr
        pqrdot = self.invI @ (LMN_magtorquers  - la.cross(pqr, H))

        self.lastBField = np.array([Be, Bn, Bu])

        return np.concatenate((vel, accel, quatdot, pqrdot))
    
    def get_B_inertial(self, t, state):
        x, y, z = state[:3]
        rho = la.norm([x, y, z])
        thetaE = np.arccos(z / rho)
        psiE = np.arctan2(y, x)
        phiE = 0

        latitude = 90 - thetaE * 180 / np.pi
        longitude = psiE * 180 / np.pi
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