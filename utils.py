import numpy as np

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

def crossSkew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]       
    ])