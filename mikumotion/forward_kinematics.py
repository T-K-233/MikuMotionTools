"""
**robot -> animation, step 1 of 2: into the hub.**
Converts a robot log into a motion; the mirror of
:func:`mikumotion.blender.armature_to_motion`, which does the same job for direction 1.

A log carries joint angles and a floating-base pose. MuJoCo forward kinematics turns
those into per-link world poses.

The lite_pro MJCF has no floating base: ``pelvis`` is a fixed root at the world origin.
FK on the logged joint angles therefore gives each body's pose relative to the pelvis
frame, and the logged base pose is composed on top:

    W_body = T_base @ (pelvis_fk^-1 @ body_fk)

``T_base`` is the ``/odom`` base pose. The pelvis frame is the identity at the zero pose,
so this reduces to ``W_body = T_base @ body_fk``. The general form stays correct if the
MJCF places the root body away from the origin.

The body frames are the MJCF body frames, which match the URDF link frames, so the motion
can drive an armature built from that URDF.
"""

from __future__ import annotations

import numpy as np
import mujoco

from .mcap_io import read_robot_log
from .motion_sequence import MotionSequence, fill_body_velocities


def pose_matrix(position: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """A 4x4 homogeneous transform from a position and a (w, x, y, z) quaternion."""
    rotation = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, np.asarray(quat_wxyz, dtype=np.float64))

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation.reshape(3, 3)
    transform[:3, 3] = position
    return transform


def matrix_to_quat(rotation: np.ndarray) -> np.ndarray:
    """A (w, x, y, z) quaternion from a 3x3 rotation matrix."""
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(rotation, dtype=np.float64).reshape(9))
    return quat


def robot_log_to_motion(
    mcap_path: str,
    mjcf_path: str,
    base_body: str = "pelvis",
) -> MotionSequence:
    """Build a :class:`MotionSequence` (per-link world poses) from an mcap motion log.

    Args:
        mcap_path: Path to the ROS2 motion ``.mcap`` (see :func:`read_robot_log`).
        mjcf_path: Path to the robot MJCF used for forward kinematics.
        base_body: Name of the body whose world pose the ``/odom`` topic reports
            (the floating base; ``pelvis`` for lite_pro).

    Returns:
        A ``MotionSequence`` whose ``body_names`` are the MJCF body names (excluding
        ``world``), with world-space ``body_positions`` / ``body_rotations`` (wxyz),
        finite-difference body velocities, and the logged joint angles/velocities
        reordered to the MJCF joint order.
    """
    log = read_robot_log(mcap_path)
    fps = int(log["fps"])

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
        T_base = pose_matrix(base_pos[f], base_quat[f])
        pelvis_fk = pose_matrix(data.xpos[base_id], data.xquat[base_id])
        pelvis_fk_inv = np.linalg.inv(pelvis_fk)

        for k, b in enumerate(body_ids):
            body_fk = pose_matrix(data.xpos[b], data.xquat[b])
            W = T_base @ pelvis_fk_inv @ body_fk
            motion.body_positions[f, k, :] = W[:3, 3]
            motion.body_rotations[f, k, :] = matrix_to_quat(W[:3, :3])

        # joint angles/velocities in model order
        motion.joint_positions[f, valid] = log_qpos[f, lcol[valid]]
        motion.joint_velocities[f, valid] = log_qvel[f, lcol[valid]]

        if f % 500 == 0:
            print(f"[fk] frame {f}/{n}", end="\r")

    fill_body_velocities(motion)

    print(f"\n[fk] built {motion!r}")
    return motion
