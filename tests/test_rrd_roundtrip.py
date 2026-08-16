"""The motion sequence must survive a write/read cycle through the .rrd format exactly."""

import numpy as np
import pytest

from mikumotion.motion_sequence import MotionSequence
from mikumotion.rrd_io import MotionStore, quat_to_wxyz, quat_to_xyzw

URDF = """<?xml version="1.0"?>
<robot name="testbot">
  <link name="base"/>
  <link name="arm"/>
  <joint name="j0" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.15" upper="3.15" effort="1" velocity="1"/>
  </joint>
</robot>
"""

JOINT_NAMES = ["j0"]
BODY_NAMES = ["base", "arm"]
NUM_FRAMES = 7


def build_motion():
    """A small motion with distinct, non-symmetric values in every field."""
    rng = np.random.default_rng(0)
    motion = MotionSequence(
        num_frames=NUM_FRAMES,
        joint_names=JOINT_NAMES,
        body_names=BODY_NAMES,
        fps=50,
    )
    motion.joint_positions[:] = rng.uniform(-1.0, 1.0, motion.joint_positions.shape)
    motion.joint_velocities[:] = rng.uniform(-2.0, 2.0, motion.joint_velocities.shape)
    motion.body_positions[:] = rng.uniform(-1.0, 1.0, motion.body_positions.shape)
    motion.body_linear_velocities[:] = rng.uniform(-3.0, 3.0, motion.body_linear_velocities.shape)
    motion.body_angular_velocities[:] = rng.uniform(-4.0, 4.0, motion.body_angular_velocities.shape)

    quats = rng.uniform(-1.0, 1.0, motion.body_rotations.shape)
    quats /= np.linalg.norm(quats, axis=-1, keepdims=True)
    quats[quats[..., 0] < 0] *= -1.0  # keep w positive so the round trip is sign-stable
    motion.body_rotations[:] = quats
    return motion


@pytest.fixture
def store(tmp_path):
    urdf = tmp_path / "testbot.urdf"
    urdf.write_text(URDF)
    written = MotionStore(tmp_path / "motions")
    motion = build_motion()
    written.write_motion("clip", motion)
    written.write_preview("clip", motion, urdf)
    return written


def test_motion_round_trips_without_a_robot_model(tmp_path):
    """Direction 1 exports a Blender armature with no URDF; that must still be storable."""
    store = MotionStore(tmp_path / "motions")
    store.write_motion("clip", build_motion())
    motion = store.read_motion("clip")
    np.testing.assert_allclose(motion.body_positions, build_motion().body_positions, rtol=0, atol=1e-6)
    assert store.layer_paths("clip") == [store.motion_file("clip")]


def test_quaternion_order_is_involutive():
    quat = np.array([[0.5, 0.5, 0.5, 0.5], [1.0, 0.0, 0.0, 0.0]])
    assert np.array_equal(quat_to_wxyz(quat_to_xyzw(quat)), quat)


def test_metadata_round_trips(store):
    motion = store.read_motion("clip")
    assert motion.joint_names == JOINT_NAMES
    assert motion.body_names == BODY_NAMES
    assert motion.num_frames == NUM_FRAMES
    assert isinstance(motion.fps, int)
    assert motion.fps == 50


@pytest.mark.parametrize("field", [
    "joint_positions",
    "joint_velocities",
    "body_positions",
    "body_rotations",
    "body_linear_velocities",
    "body_angular_velocities",
])
def test_array_round_trips(store, field):
    expected = getattr(build_motion(), field)
    actual = getattr(store.read_motion("clip"), field)
    assert actual.shape == expected.shape
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-6)


def test_layer_paths_point_at_existing_files(store):
    paths = store.layer_paths("clip")
    assert [path.name for path in paths] == ["clip.rrd", "clip.rrd", "testbot.rrd", "testbot.rbl"]
    assert all(path.exists() for path in paths)
