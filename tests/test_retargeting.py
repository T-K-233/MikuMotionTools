"""A mocap bone and the robot link it drives disagree about which way their axes point."""

import numpy as np

from mikumotion.math import euler_xyz_to_quat, quat_mul


def conjugate(quat):
    return quat * np.array([1.0, -1.0, -1.0, -1.0])


def rest_offset(source_rest, link_zero):
    """The offset MotionRetargetingIK.rest_offsets builds, in one line."""
    return quat_mul(conjugate(source_rest), link_zero)


def euler(roll, pitch, yaw):
    return euler_xyz_to_quat(np.array([roll]), np.array([pitch]), np.array([yaw]))[0]


def angle_between(first, second):
    return np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(first, second))))))


def test_rest_pose_maps_onto_the_robots_zero_pose():
    """A rig standing at rest must ask for a robot standing at zero, not for a twist."""
    source_rest = euler(0.0, 0.0, np.pi / 2)   # the bone's frame, 90 deg off the link's
    link_zero = euler(0.0, 0.0, 0.0)

    target = quat_mul(source_rest, rest_offset(source_rest, link_zero))
    assert angle_between(target, link_zero) < 1e-4

    # and without the offset, the solver would have been aimed 90 degrees away
    assert angle_between(source_rest, link_zero) > 89.0


def test_a_turn_of_the_bone_is_the_same_turn_of_the_link():
    source_rest = euler(0.0, 0.0, np.pi / 2)
    link_zero = euler(np.pi / 2, 0.0, 0.0)
    offset = rest_offset(source_rest, link_zero)

    turn = euler(0.0, 0.0, 0.3)                       # a world-frame turn of the bone
    target = quat_mul(quat_mul(turn, source_rest), offset)

    np.testing.assert_allclose(np.abs(target), np.abs(quat_mul(turn, link_zero)), atol=1e-6)


def test_offsets_are_identity_when_the_frames_already_agree():
    """The pairs that line up at rest — pelvis, head, knees — must not be disturbed."""
    shared = euler(0.2, -0.4, 1.1)
    offset = rest_offset(shared, shared)
    np.testing.assert_allclose(np.abs(offset), [1.0, 0.0, 0.0, 0.0], atol=1e-6)
