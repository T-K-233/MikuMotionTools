"""Golden values for the quaternion helpers, in (qw, qx, qy, qz) order."""

import numpy as np
import pytest

from mikumotion.math import quat_from_euler_xyz
from mikumotion.motion_sequence import MotionSequence, rotate_motion, translate_motion

X30, Y60, Z90 = np.deg2rad(30), np.deg2rad(60), np.deg2rad(90)


@pytest.mark.parametrize("roll, pitch, yaw, expected", [
    (0, 0, 0, [1, 0, 0, 0]),
    (X30, 0, 0, [0.966, 0.259, 0, 0]),
    (-X30, 0, 0, [0.966, -0.259, 0, 0]),
    (0, Y60, 0, [0.866, 0, 0.5, 0]),
    (0, -Y60, 0, [0.866, 0, -0.5, 0]),
    (0, 0, Z90, [0.707, 0, 0, 0.707]),
    (0, 0, -Z90, [0.707, 0, 0, -0.707]),
    (Z90, Z90, 0, [0.5, 0.5, 0.5, 0.5]),
    (Z90, -Z90, 0, [0.5, 0.5, -0.5, -0.5]),
    (Z90, 0, Z90, [0.5, 0.5, -0.5, 0.5]),
    (Z90, 0, -Z90, [0.5, 0.5, 0.5, -0.5]),
])
def test_quat_from_euler_xyz(roll, pitch, yaw, expected):
    np.testing.assert_allclose(quat_from_euler_xyz(roll, pitch, yaw), expected, atol=1e-3)


def build_motion():
    motion = MotionSequence(num_frames=3, joint_names=["j"], body_names=["a", "b"], fps=30)
    motion.body_positions[:] = np.arange(18, dtype=np.float32).reshape(3, 2, 3)
    motion.joint_positions[:] = 0.5
    return motion


def test_translate_motion_shifts_positions_and_copies():
    motion = build_motion()
    moved = translate_motion(motion, np.array([1.0, 2.0, 3.0], dtype=np.float32))
    np.testing.assert_allclose(moved.body_positions, motion.body_positions + [1.0, 2.0, 3.0])

    moved.body_positions[:] = 0.0
    assert motion.body_positions.any(), "translate_motion must not alias the source arrays"


def test_rotate_motion_by_full_turn_is_identity():
    motion = build_motion()
    turned = rotate_motion(motion, 2 * np.pi)
    np.testing.assert_allclose(turned.body_positions, motion.body_positions, atol=1e-5)
    np.testing.assert_allclose(turned.joint_positions, motion.joint_positions)


def test_rotate_motion_quarter_turn_about_z():
    motion = MotionSequence(num_frames=1, joint_names=[], body_names=["a"], fps=30)
    motion.body_positions[0, 0] = [1.0, 0.0, 0.0]
    turned = rotate_motion(motion, np.pi / 2)
    np.testing.assert_allclose(turned.body_positions[0, 0], [0.0, 1.0, 0.0], atol=1e-6)
