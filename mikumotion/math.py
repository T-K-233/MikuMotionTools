from typing import Optional

import numpy as np


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions together.

    Args:
        q1: The first quaternion in (w, x, y, z). Shape is (..., 4).
        q2: The second quaternion in (w, x, y, z). Shape is (..., 4).

    Returns:
        The product of the two quaternions in (w, x, y, z). Shape is (..., 4).

    Raises:
        ValueError: Input shapes of ``q1`` and ``q2`` are not matching.
    """
    # check input is correct
    if q1.shape != q2.shape:
        msg = f"Expected input quaternion shape mismatch: {q1.shape} != {q2.shape}."
        raise ValueError(msg)
    # reshape to (N, 4) for multiplication
    shape = q1.shape
    q1 = q1.reshape(-1, 4)
    q2 = q2.reshape(-1, 4)
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


def interpolate(
    a: np.ndarray,
    *,
    b: Optional[np.ndarray] = None,
    blend: Optional[np.ndarray] = None,
    start: Optional[np.ndarray] = None,
    end: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Linear interpolation between consecutive values.

    Args:
        a: The first value. Shape is (N, X) or (N, M, X).
        b: The second value. Shape is (N, X) or (N, M, X).
        blend: Interpolation coefficient between 0 (a) and 1 (b).
        start: Indexes to fetch the first value. If both, ``start`` and ``end` are specified,
            the first and second values will be fetches from the argument ``a`` (dimension 0).
        end: Indexes to fetch the second value. If both, ``start`` and ``end` are specified,
            the first and second values will be fetches from the argument ``a`` (dimension 0).

    Returns:
        Interpolated values. Shape is (N, X) or (N, M, X).
    """
    if start is not None and end is not None:
        return interpolate(a=a[start], b=a[end], blend=blend)
    if a.ndim >= 2:
        blend = blend.unsqueeze(-1)
    if a.ndim >= 3:
        blend = blend.unsqueeze(-1)
    return (1.0 - blend) * a + blend * b


def slerp(
    q0: np.ndarray,
    *,
    q1: Optional[np.ndarray] = None,
    blend: Optional[np.ndarray] = None,
    start: Optional[np.ndarray] = None,
    end: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Interpolation between consecutive rotations (Spherical Linear Interpolation).

    Args:
        q0: The first quaternion (wxyz). Shape is (N, 4) or (N, M, 4).
        q1: The second quaternion (wxyz). Shape is (N, 4) or (N, M, 4).
        blend: Interpolation coefficient between 0 (q0) and 1 (q1).
        start: Indexes to fetch the first quaternion. If both, ``start`` and ``end` are specified,
            the first and second quaternions will be fetches from the argument ``q0`` (dimension 0).
        end: Indexes to fetch the second quaternion. If both, ``start`` and ``end` are specified,
            the first and second quaternions will be fetches from the argument ``q0`` (dimension 0).

    Returns:
        Interpolated quaternions. Shape is (N, 4) or (N, M, 4).
    """
    if start is not None and end is not None:
        return slerp(q0=q0[start], q1=q0[end], blend=blend)
    if q0.ndim >= 2:
        blend = blend.unsqueeze(-1)
    if q0.ndim >= 3:
        blend = blend.unsqueeze(-1)

    qw, qx, qy, qz = 0, 1, 2, 3  # wxyz
    cos_half_theta = (
        q0[..., qw] * q1[..., qw]
        + q0[..., qx] * q1[..., qx]
        + q0[..., qy] * q1[..., qy]
        + q0[..., qz] * q1[..., qz]
    )

    neg_mask = cos_half_theta < 0
    q1 = q1.clone()
    q1[neg_mask] = -q1[neg_mask]
    cos_half_theta = np.abs(cos_half_theta)
    cos_half_theta = np.expand_dims(cos_half_theta, axis=-1)

    half_theta = np.arccos(cos_half_theta)
    sin_half_theta = np.sqrt(1.0 - cos_half_theta * cos_half_theta)

    ratio_a = np.sin((1 - blend) * half_theta) / sin_half_theta
    ratio_b = np.sin(blend * half_theta) / sin_half_theta

    new_q_x = ratio_a * q0[..., qx:qx + 1] + ratio_b * q1[..., qx:qx + 1]
    new_q_y = ratio_a * q0[..., qy:qy + 1] + ratio_b * q1[..., qy:qy + 1]
    new_q_z = ratio_a * q0[..., qz:qz + 1] + ratio_b * q1[..., qz:qz + 1]
    new_q_w = ratio_a * q0[..., qw:qw + 1] + ratio_b * q1[..., qw:qw + 1]

    new_q = np.concatenate([new_q_w, new_q_x, new_q_y, new_q_z], axis=-1)
    new_q = np.where(np.abs(sin_half_theta) < 0.001, 0.5 * q0 + 0.5 * q1, new_q)
    new_q = np.where(np.abs(cos_half_theta) >= 1, q0, new_q)
    return new_q
