"""
Convert a policy/tracking motion log (joint angles + floating-base pose) into a
:class:`~mikumotion.motion_sequence.MotionSequence` of **per-link world poses** via
MuJoCo forward kinematics.

The lite_pro MJCF has no floating base (``pelvis`` is a fixed root at the world
origin). We therefore run FK with the logged joint angles to obtain every body's
pose *relative to the pelvis frame*, then compose the logged base pose on top:

    W_body = T_base @ (pelvis_fk^-1 @ body_fk)

where ``T_base`` is the ``/odom`` base pose. At the zero pose the pelvis frame is the
identity, so this reduces to ``W_body = T_base @ body_fk``; the general form keeps it
correct even if the MJCF places the root body off-origin.

Output body frames are the MJCF body frames, which are identical to the URDF link
frames used to build the Blender armature/meshes in Stage 1 — so the resulting
``MotionSequence`` can drive that armature directly (Stage 3).
"""

from __future__ import annotations

import numpy as np
import mujoco

from .mcap_io import read_motion_mcap
from .motion_sequence import MotionSequence


def _quat_to_mat(q_wxyz: np.ndarray) -> np.ndarray:
    """(w,x,y,z) quaternion -> 3x3 rotation matrix (via MuJoCo)."""
    m = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(m, np.asarray(q_wxyz, dtype=np.float64))
    return m.reshape(3, 3)


def _mat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> (w,x,y,z) quaternion (via MuJoCo)."""
    q = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, dtype=np.float64).reshape(9))
    return q


def _make_tf(pos: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from position + (w,x,y,z) quaternion."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _quat_to_mat(quat_wxyz)
    T[:3, 3] = pos
    return T


def motion_from_policy_log(
    mcap_path: str,
    mjcf_path: str,
    *,
    base_body: str = "pelvis",
) -> MotionSequence:
    """Build a :class:`MotionSequence` (per-link world poses) from an mcap motion log.

    Args:
        mcap_path: Path to the ROS2 motion ``.mcap`` (see :func:`read_motion_mcap`).
        mjcf_path: Path to the robot MJCF used for forward kinematics.
        base_body: Name of the body whose world pose the ``/odom`` topic reports
            (the floating base; ``pelvis`` for lite_pro).

    Returns:
        A ``MotionSequence`` whose ``body_names`` are the MJCF body names (excluding
        ``world``), with world-space ``body_positions`` / ``body_rotations`` (wxyz),
        finite-difference body velocities, and the logged joint angles/velocities
        reordered to the MJCF joint order.
    """
    log = read_motion_mcap(mcap_path)
    fps = int(log["fps"])
    dt = 1.0 / fps

    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)

    # body list excluding the implicit 'world' body (index 0)
    body_ids = list(range(1, model.nbody))
    body_names = [model.body(b).name for b in body_ids]
    base_id = model.body(base_body).id

    # map logged joint name -> qpos address (hinge joints only, all 1-dof here)
    log_joint_names = log["joint_names"]
    log_name_to_col = {nm: i for i, nm in enumerate(log_joint_names)}

    model_joint_names: list[str] = []
    joint_qadr: list[int] = []
    joint_log_col: list[int] = []
    missing = []
    for j in range(model.njnt):
        jname = model.joint(j).name
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            continue  # base handled separately
        model_joint_names.append(jname)
        joint_qadr.append(int(model.jnt_qposadr[j]))
        if jname in log_name_to_col:
            joint_log_col.append(log_name_to_col[jname])
        else:
            joint_log_col.append(-1)
            missing.append(jname)
    if missing:
        print(f"[fk] WARNING: {len(missing)} model joints absent from log (held at 0): {missing[:6]}...")

    n = log["joint_positions"].shape[0]
    nb = len(body_ids)

    motion = MotionSequence(
        num_frames=n,
        joint_names=model_joint_names,
        body_names=body_names,
        fps=fps,
    )

    base_pos = log["base_positions"]
    base_quat = log["base_quaternions"]
    log_qpos = log["joint_positions"]
    log_qvel = log["joint_velocities"]

    qadr = np.asarray(joint_qadr, dtype=np.int64)
    lcol = np.asarray(joint_log_col, dtype=np.int64)
    valid = lcol >= 0

    for f in range(n):
        data.qpos[:] = 0.0
        data.qpos[qadr[valid]] = log_qpos[f, lcol[valid]]
        mujoco.mj_kinematics(model, data)

        # pelvis (base) FK pose, and the composed world base transform
        T_base = _make_tf(base_pos[f], base_quat[f])
        pelvis_fk = _make_tf(data.xpos[base_id], data.xquat[base_id])
        pelvis_fk_inv = np.linalg.inv(pelvis_fk)

        for k, b in enumerate(body_ids):
            body_fk = _make_tf(data.xpos[b], data.xquat[b])
            W = T_base @ pelvis_fk_inv @ body_fk
            motion._body_positions[f, k, :] = W[:3, 3]
            motion._body_rotations[f, k, :] = _mat_to_quat(W[:3, :3])

        # joint angles/velocities in model order
        motion._joint_positions[f, valid] = log_qpos[f, lcol[valid]]
        if log_qvel is not None:
            motion._joint_velocities[f, valid] = log_qvel[f, lcol[valid]]

        if f % 500 == 0:
            print(f"[fk] frame {f}/{n}", end="\r")

    # ---- finite-difference body velocities (world frame) ----
    pos = motion._body_positions
    motion._body_linear_velocities[1:] = (pos[1:] - pos[:-1]) / dt

    rot = motion._body_rotations
    # angular velocity from consecutive quaternions: dq = q_next * conj(q_prev)
    qn = rot[1:].reshape(-1, 4)
    qp = rot[:-1].reshape(-1, 4)
    conj = qp * np.array([1.0, -1.0, -1.0, -1.0])
    w0, x0, y0, z0 = conj[:, 0], conj[:, 1], conj[:, 2], conj[:, 3]
    w1, x1, y1, z1 = qn[:, 0], qn[:, 1], qn[:, 2], qn[:, 3]
    dq = np.stack([
        w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0,
        w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
        w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0,
        w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0,
    ], axis=-1)
    dq_w = np.clip(dq[:, 0], -1.0, 1.0)
    angle = 2.0 * np.arccos(dq_w)
    s = np.sqrt(np.maximum(1.0 - dq_w * dq_w, 1e-12))
    axis = dq[:, 1:] / s[:, None]
    omega = (axis * (angle / dt)[:, None]).reshape(rot.shape[0] - 1, nb, 3)
    motion._body_angular_velocities[1:] = omega

    print(f"\n[fk] built MotionSequence: {n} frames, {nb} bodies, {len(model_joint_names)} joints, {fps} fps")
    return motion


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Motion log (.mcap) -> MotionSequence (.npz) via MuJoCo FK.")
    parser.add_argument("--mcap", type=str, required=True)
    parser.add_argument("--mjcf", type=str, required=True)
    parser.add_argument("--out", type=str, required=True)
    args = parser.parse_args()

    m = motion_from_policy_log(args.mcap, args.mjcf)
    m.save(args.out)
    print("saved:", args.out)
