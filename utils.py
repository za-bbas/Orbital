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
    v = np.ravel(v)
    return np.array([
        [0, -1*v[2], v[1]],
        [v[2], 0, -1*v[0]],
        [-1*v[1], v[0], 0]       
    ])

def quatMultiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def rotateVector(q, v):
    v = v.flatten()
    v_q = np.array([0.0, *v])
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    rotated = quatMultiply(quatMultiply(q, v_q), q_conj)
    return rotated[1:]  # return vector part