"""Transforms map child coordinates into parent coordinates, in metres."""
import numpy as np
from scipy.spatial.transform import Rotation


def transform(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (7,) or not np.all(np.isfinite(values)):
        raise ValueError('Expected finite [x,y,z,qx,qy,qz,qw]')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    t[:3, 3] = values[:3]
    return t


def values(t):
    return np.r_[t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_quat()].tolist()


def pack(t):
    return np.r_[Rotation.from_matrix(t[:3, :3]).as_rotvec(), t[:3, 3]]


def unpack(p):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(p[:3]).as_matrix()
    t[:3, 3] = p[3:]
    return t


def inverse(t):
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def distance(a, b):
    return (float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
            float(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()))


def yaw(t):
    return float(np.arctan2(t[1, 0], t[0, 0]))


def wrap(angle):
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def marker_points(length):
    h = length / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)
