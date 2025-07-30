# Modified from Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# SPDX-License-Identifier: BSD-3-Clause

import numpy as np


def quat_from_euler_xyz(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Convert rotations given as Euler angles in radians to Quaternions.

    Note:
        The euler angles are assumed in XYZ convention (1-2-3 sequence).
        See https://ntrs.nasa.gov/citations/19770024290 for different conventions.

    Args:
        roll: Rotation around x-axis (in radians). Shape is (N,).
        pitch: Rotation around y-axis (in radians). Shape is (N,).
        yaw: Rotation around z-axis (in radians). Shape is (N,).

    Returns:
        The quaternion in (w, x, y, z). Shape is (N, 4).
    """
    sr = np.sin(roll * 0.5)
    cr = np.cos(roll * 0.5)
    sp = np.sin(pitch * 0.5)
    cp = np.cos(pitch * 0.5)
    sy = np.sin(yaw * 0.5)
    cy = np.cos(yaw * 0.5)
    # compute quaternion
    qw = -sr * sp * sy + cr * cp * cy
    qx =  sr * cp * cy + cr * sp * sy
    qy = -sr * cp * sy + cr * sp * cy
    qz =  sr * sp * cy + cr * cp * sy

    return np.stack([qw, qx, qy, qz], axis=-1)


def quat_from_euler_zyx(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Convert rotations given as Euler angles in radians to Quaternions.

    Note:
        The euler angles are assumed in ZYX convention (3-2-1 sequence).
        See https://ntrs.nasa.gov/citations/19770024290 for different conventions.

    Args:
        roll: Rotation around x-axis (in radians). Shape is (N,).
        pitch: Rotation around y-axis (in radians). Shape is (N,).
        yaw: Rotation around z-axis (in radians). Shape is (N,).

    Returns:
        The quaternion in (w, x, y, z). Shape is (N, 4).
    """
    sr = np.sin(roll * 0.5)
    cr = np.cos(roll * 0.5)
    sp = np.sin(pitch * 0.5)
    cp = np.cos(pitch * 0.5)
    sy = np.sin(yaw * 0.5)
    cy = np.cos(yaw * 0.5)
    # compute quaternion
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return np.stack([qw, qx, qy, qz], axis=-1)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions together.

    Args:
        q1: The first quaternion in (w, x, y, z). Shape is (..., 4).
        q2: The second quaternion in (w, x, y, z). Shape is (..., 4).

    Returns:
        The product of the two quaternions in (w, x, y, z). Shape is (..., 4).

    Note:
        Supports NumPy broadcasting. If shapes don't match, broadcasting will be applied.
    """
    # Apply broadcasting to match shapes
    q1_broadcasted, q2_broadcasted = np.broadcast_arrays(q1, q2)
    
    # reshape to (N, 4) for multiplication
    shape = q1_broadcasted.shape
    q1 = q1_broadcasted.reshape(-1, 4)
    q2 = q2_broadcasted.reshape(-1, 4)
    # extract components from quaternions
    w1, x1, y1, z1 = q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3]
    w2, x2, y2, z2 = q2[:, 0], q2[:, 1], q2[:, 2], q2[:, 3]
    # perform multiplication
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    return np.stack([w, x, y, z], axis=-1).reshape(shape)
