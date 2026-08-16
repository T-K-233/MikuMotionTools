import numpy as np

from .math import quat_from_euler_zyx, quat_mul


class MotionSequence:
    """
    A sequence of motion data: joint and body trajectories, all in world frame.

    This is the central data structure of MikuMotionTools — every other module is a
    converter into or out of it. It is a plain mutable container: producers allocate one
    and fill the arrays in place. Persistence lives in :mod:`mikumotion.rrd_io`, which
    reads and writes it as a Rerun ``.rrd``.

    Fields, for ``F`` frames, ``D`` joints and ``B`` bodies:

    ==========================  ==========  ==================================================
    ``fps``                     int         frame rate
    ``joint_names``             D           joint names
    ``body_names``              B           link names
    ``joint_positions``         (F, D)      joint angles, rad
    ``joint_velocities``        (F, D)      joint angular velocities, rad/s
    ``body_positions``          (F, B, 3)   link positions in world frame, m
    ``body_rotations``          (F, B, 4)   link rotations in world frame, (qw, qx, qy, qz)
    ``body_linear_velocities``  (F, B, 3)   link linear velocities in world frame, m/s
    ``body_angular_velocities`` (F, B, 3)   link angular velocities in world frame, rad/s
    ==========================  ==========  ==================================================

    Modified from Isaac Lab's motion_loader.py:
    https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/direct/humanoid_amp/motions/motion_loader.py

    Original work:
    Copyright (c) 2022-2026, The Isaac Lab Project Developers
    (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
    SPDX-License-Identifier: BSD-3-Clause

    This class is modified to use numpy instead of torch, suitable for a CPU-only
    environment, and to store its arrays as plain public attributes.
    """

    def __init__(self, num_frames, joint_names, body_names, fps=50):
        self.fps = int(fps)
        self.joint_names = list(joint_names)
        self.body_names = list(body_names)

        num_joints = len(self.joint_names)
        num_bodies = len(self.body_names)

        self.joint_positions = np.zeros((num_frames, num_joints), dtype=np.float32)
        self.joint_velocities = np.zeros((num_frames, num_joints), dtype=np.float32)
        self.body_positions = np.zeros((num_frames, num_bodies, 3), dtype=np.float32)
        self.body_rotations = np.zeros((num_frames, num_bodies, 4), dtype=np.float32)
        self.body_linear_velocities = np.zeros((num_frames, num_bodies, 3), dtype=np.float32)
        self.body_angular_velocities = np.zeros((num_frames, num_bodies, 3), dtype=np.float32)

        self.body_rotations[:, :, 0] = 1.0  # identity quaternion

    def __repr__(self):
        return (f"MotionSequence({self.num_frames} frames, {self.num_joints} joints, "
                f"{self.num_bodies} bodies, {self.fps} fps, {self.duration:.2f}s)")

    @property
    def num_frames(self):
        return self.body_positions.shape[0]

    @property
    def num_joints(self):
        return len(self.joint_names)

    @property
    def num_bodies(self):
        return len(self.body_names)

    @property
    def duration(self):
        """Length of the motion in seconds."""
        return (self.num_frames - 1) / self.fps

    def get_body_indices(self, body_names):
        """Indices of the named bodies, in the order given."""
        for name in body_names:
            assert name in self.body_names, f"unknown body {name!r}, have {self.body_names}"
        return [self.body_names.index(name) for name in body_names]

    def copy(self):
        """A deep copy, sharing no arrays with this sequence."""
        other = MotionSequence(self.num_frames, self.joint_names, self.body_names, self.fps)
        other.joint_positions[:] = self.joint_positions
        other.joint_velocities[:] = self.joint_velocities
        other.body_positions[:] = self.body_positions
        other.body_rotations[:] = self.body_rotations
        other.body_linear_velocities[:] = self.body_linear_velocities
        other.body_angular_velocities[:] = self.body_angular_velocities
        return other


def rotate_motion(motion, z_rotation):
    """
    Return a copy of ``motion`` rotated about the world Z axis by ``z_rotation`` radians.

    Joint angles are unaffected; everything expressed in world frame is rotated.
    """
    zero = np.zeros(1, dtype=np.float32)
    rotation = quat_from_euler_zyx(zero, zero, np.array([z_rotation], dtype=np.float32))[0]
    w, x, y, z = rotation
    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)

    rotated = motion.copy()
    rotated.body_positions[:] = motion.body_positions @ matrix.T
    rotated.body_linear_velocities[:] = motion.body_linear_velocities @ matrix.T
    rotated.body_angular_velocities[:] = motion.body_angular_velocities @ matrix.T
    rotated.body_rotations[:] = quat_mul(rotation, motion.body_rotations)
    return rotated


def translate_motion(motion, translation):
    """Return a copy of ``motion`` shifted by ``translation`` (metres, world frame)."""
    translated = motion.copy()
    translated.body_positions[:] = motion.body_positions + translation
    return translated
