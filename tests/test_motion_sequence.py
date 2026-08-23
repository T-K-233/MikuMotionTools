"""The motion sequence must survive a write/read cycle through the .rrd format exactly."""

import numpy as np
import pytest

from mikumotion.motion_sequence import (REFERENCE, MotionSequence, MotionStore, body_frame,
                                        fill_body_velocities, quat_to_wxyz, quat_to_xyzw,
                                        read_blueprint_overrides, read_entity_columns, read_properties,
                                        read_entity_components)

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
    """A motion at both stages: exported to reference/, then solved for testbot."""
    urdf = tmp_path / "testbot.urdf"
    urdf.write_text(URDF)
    written = MotionStore(tmp_path / "motions")
    motion = build_motion()
    written.write_reference_motion("clip", motion)
    written.write_robot_motion("clip", motion, urdf)
    return written


def test_motion_with_no_joints_round_trips(tmp_path):
    """A mocap armature export has bodies but no joints at all."""
    store = MotionStore(tmp_path / "motions")
    motion = MotionSequence(num_frames=4, joint_names=[], body_names=BODY_NAMES, fps=50)
    motion.body_positions[:] = np.arange(24, dtype=np.float32).reshape(4, 2, 3)
    store.write_reference_motion("mocap", motion)

    back = store.read_reference_motion("mocap")
    assert back.joint_names == []
    assert back.num_frames == 4
    np.testing.assert_allclose(back.body_positions, motion.body_positions, rtol=0, atol=1e-6)


def test_motion_round_trips_without_a_robot_model(tmp_path):
    """Direction 1 exports a Blender armature with no URDF; that must still be storable."""
    store = MotionStore(tmp_path / "motions")
    store.write_reference_motion("clip", build_motion())
    motion = store.read_reference_motion("clip")
    np.testing.assert_allclose(motion.body_positions, build_motion().body_positions, rtol=0, atol=1e-6)
    assert store.layer_paths("clip") == [store.reference_file("clip")]


def test_repeated_body_names_are_rejected(tmp_path):
    """Bodies are stored at /<layer>/body/poses/<name>, so two sharing a name collide."""
    store = MotionStore(tmp_path / "motions")
    motion = MotionSequence(num_frames=2, joint_names=[], body_names=["torso", "torso"])
    with pytest.raises(AssertionError, match="body names must be unique"):
        store.write_reference_motion("clash", motion)


def test_body_names_needing_escaping_round_trip(tmp_path):
    """MMD bones are Japanese and some rigs have spaces; both must survive the entity path."""
    store = MotionStore(tmp_path / "motions")
    names = ["腰", "left upper arm", "hand.L"]
    motion = MotionSequence(num_frames=3, joint_names=[], body_names=names)
    motion.body_positions[:] = np.arange(27, dtype=np.float32).reshape(3, 3, 3)
    store.write_reference_motion("exotic", motion)

    back = store.read_reference_motion("exotic")
    assert back.body_names == names
    np.testing.assert_allclose(back.body_positions, motion.body_positions, rtol=0, atol=1e-6)


def spinning_motion(rate, num_frames=240, fps=50):
    """A body sliding along +X while spinning about +Z at ``rate`` rad/s."""
    motion = MotionSequence(num_frames=num_frames, joint_names=[], body_names=["hub"], fps=fps)
    time = np.arange(num_frames) / fps
    motion.body_positions[:, 0, 0] = 2.0 * time
    half = 0.5 * rate * time
    motion.body_rotations[:, 0, 0] = np.cos(half)
    motion.body_rotations[:, 0, 3] = np.sin(half)
    return motion


def test_velocities_match_a_known_trajectory():
    motion = spinning_motion(rate=1.5)
    fill_body_velocities(motion)

    # frame 0 has no earlier pose to difference against
    np.testing.assert_allclose(motion.body_linear_velocities[0], 0.0, atol=0)
    np.testing.assert_allclose(motion.body_linear_velocities[1:, 0, 0], 2.0, rtol=1e-4)
    np.testing.assert_allclose(motion.body_angular_velocities[1:, 0, 2], 1.5, rtol=1e-3)
    np.testing.assert_allclose(motion.body_angular_velocities[1:, 0, :2], 0.0, atol=1e-6)


def test_angular_velocity_survives_quaternion_sign_flips():
    """
    Spinning far enough to wrap makes the quaternion flip sign. Read literally that is a
    near-full turn in one frame, which used to spike at 2*pi*fps.
    """
    motion = spinning_motion(rate=4.0, num_frames=400)
    fill_body_velocities(motion)

    omega = motion.body_angular_velocities[1:, 0, 2]
    assert np.abs(omega).max() < 2 * np.pi * motion.fps / 10
    np.testing.assert_allclose(omega, 4.0, rtol=1e-3)


def test_quaternion_order_is_involutive():
    quat = np.array([[0.5, 0.5, 0.5, 0.5], [1.0, 0.0, 0.0, 0.0]])
    assert np.array_equal(quat_to_wxyz(quat_to_xyzw(quat)), quat)


def test_metadata_round_trips(store):
    motion = store.read_reference_motion("clip", "testbot")
    assert motion.joint_names == JOINT_NAMES
    assert motion.body_names == BODY_NAMES
    assert motion.num_frames == NUM_FRAMES
    assert isinstance(motion.fps, int)
    assert motion.fps == 50


def test_reference_layer_is_robot_agnostic(store):
    """A joint angle only means something for one robot, so reference/ carries none."""
    reference = store.read_reference_motion("clip")
    assert reference.joint_names == []
    assert reference.body_names == BODY_NAMES
    np.testing.assert_allclose(reference.body_positions, build_motion().body_positions,
                               rtol=0, atol=1e-6)


def test_a_robot_layer_holds_everything_under_its_own_name(store):
    """
    A robot layer must not write outside ``/<robot>/``, or a second robot opened beside it
    would overwrite these entities. That includes the URDF's static transforms, which Rerun
    puts at ``/tf_static`` unless it is told otherwise.
    """
    paths = set(read_entity_components(store.robot_file("clip", "testbot")))
    assert {"/testbot/tf", "/testbot/tf_static"} <= paths
    assert "/tf_static" not in paths

    # "/__properties" is Rerun's own, and is keyed by the robot name inside
    stray = {path for path in paths if not path.startswith(("/testbot", "/__"))}
    assert not stray, stray


def test_a_robot_layer_stores_the_poses_the_solve_reached(store):
    """
    The reference poses are what the solve aimed at; the robot layer keeps what it reached.
    Both are stored, because the difference between them is the retarget's error.
    """
    entities = set(read_entity_columns(store.robot_file("clip", "testbot")))
    assert entities >= {"/testbot/joint/positions", "/testbot/joint/velocities",
                        "/testbot/body/poses/base", "/testbot/body/poses/arm",
                        "/testbot/body/linear_velocities", "/testbot/body/angular_velocities"}

    expected = build_motion()
    poses = read_entity_columns(store.robot_file("clip", "testbot"))
    for index, body in enumerate(BODY_NAMES):
        column = poses[f"/testbot/body/poses/{body}"]
        np.testing.assert_allclose(column["translation"].reshape(-1, 3),
                                   expected.body_positions[:, index], rtol=0, atol=1e-6)


def test_the_reader_returns_reference_poses_with_robot_joints(store):
    """
    Both layers now hold body poses, so which one a read returns has to be unambiguous: a
    training run tracks the reference, and reads the robot's own poses from the robot layer.
    """
    whole = store.read_reference_motion("clip", "testbot")
    assert whole.body_names == BODY_NAMES and whole.joint_names == JOINT_NAMES

    reference = read_entity_columns(store.reference_file("clip"))
    np.testing.assert_allclose(
        whole.body_positions[:, 0],
        reference["/reference/body/poses/base"]["translation"].reshape(-1, 3),
        rtol=0, atol=1e-6)


def test_reference_and_robot_frames_never_share_a_name(store):
    """
    A coordinate frame can have one parent. The robot's link frames come from its joint
    chain, so its body poses name ``<robot>/body/<link>`` and the reference names
    ``reference/body/<name>``: three namespaces that cannot collide.
    """
    assert body_frame(REFERENCE, "base") == "reference/body/base"
    assert body_frame("testbot", "base") == "testbot/body/base"
    assert body_frame(REFERENCE, "base") != body_frame("testbot", "base") != "testbot/base"


@pytest.mark.parametrize("layer", ["reference", "testbot"])
def test_velocity_arrows_name_the_frame_they_are_measured_in(store, layer):
    """
    An entity that names no frame is an orphan. The viewer invents one called
    ``tf#<entity path>``, cannot route it to the view's root, and draws nothing but an
    error — which is what a robot layer opened on its own used to show.
    """
    path = store.reference_file("clip") if layer == "reference" else         store.robot_file("clip", "testbot")
    components = read_entity_components(path)
    for quantity in ("linear", "angular"):
        entity = f"/{layer}/body/{quantity}_velocities"
        assert "CoordinateFrame:frame" in components[entity], components[entity]


def test_every_spatial_entity_is_attached_to_the_frame_graph(store):
    """
    The general form of the bug above, over both layers at once. Scalars are exempt: they
    are plotted against time, never placed in the 3D view, so they need no frame.
    """
    anchors = {"CoordinateFrame:frame", "Transform3D:child_frame"}
    for path in (store.reference_file("clip"), store.robot_file("clip", "testbot")):
        for entity, components in read_entity_components(path).items():
            spatial = {name for name in components
                       if ":" in name and not name.startswith(("Scalars:", "rerun.controls"))}
            if entity.startswith("/__") or not spatial:
                continue
            assert components & anchors, f"{entity} names no frame: {sorted(components)}"


def test_the_blueprint_hides_entities_instead_of_excluding_them(store):
    """
    A hidden entity stays in the view, so the entity tree lists it and one click brings it
    back. An excluded one vanishes from the tree, and only the view's filter can recover it.
    The robot's 75 link gizmos bury the robot, so they start off, but they must stay findable.
    """
    hidden = read_blueprint_overrides(store.robot_file("clip", "testbot"))
    assert "/testbot/body/frames" in hidden
    assert {"/testbot/body/linear_velocities", "/reference/body/linear_velocities"} <= hidden

    # the reference gizmos are what the IK aimed at, so they are the one thing left visible
    assert not any(path.startswith("/reference/body/frames") for path in hidden), hidden


def test_a_motion_opened_alone_hides_its_velocity_arrows(store):
    """Both layers embed a blueprint, so either file opens sensibly on its own."""
    assert read_blueprint_overrides(store.reference_file("clip")) == {
        "/reference/body/linear_velocities", "/reference/body/angular_velocities"}


def test_a_retarget_records_which_reference_body_drove_each_link(tmp_path):
    """
    A retarget pairs two rigs that do not share a vocabulary: the reference calls a body
    ``torso`` where the robot calls its link ``chest``. Without the pairing in the file, a
    reader has to know which retarget map produced it.
    """
    urdf = tmp_path / "testbot.urdf"
    urdf.write_text(URDF)
    store = MotionStore(tmp_path / "motions")

    solved = MotionSequence(num_frames=NUM_FRAMES, joint_names=JOINT_NAMES,
                            body_names=["base", "arm"], fps=50)
    store.write_reference_motion("clip", build_motion())
    store.write_robot_motion("clip", solved, urdf, ["pelvis", "upper_arm"])

    properties = read_properties(store.robot_file("clip", "testbot"))
    pairs = dict(zip([str(v) for v in properties["reference_body_names"]],
                     [str(v) for v in properties["body_names"]]))
    assert pairs == {"pelvis": "base", "upper_arm": "arm"}


def test_an_imported_log_pairs_each_body_with_itself(store):
    """
    An imported log is not retargeted, so its bodies are already the reference's own. The
    pairing is written anyway, as the identity it is, so a reader never has to branch.
    """
    properties = read_properties(store.robot_file("clip", "testbot"))
    assert [str(v) for v in properties["reference_body_names"]] == BODY_NAMES
    assert [str(v) for v in properties["body_names"]] == BODY_NAMES


def test_robot_layers_do_not_collide(tmp_path):
    """Two robots solved from one motion must not overwrite each other's entities."""
    first, second = tmp_path / "testbot.urdf", tmp_path / "otherbot.urdf"
    first.write_text(URDF)
    second.write_text(URDF.replace('robot name="testbot"', 'robot name="otherbot"'))

    store = MotionStore(tmp_path / "motions")
    motion = build_motion()
    store.write_reference_motion("clip", motion)
    store.write_robot_motion("clip", motion, first)
    doubled = motion.copy()
    doubled.joint_positions *= 2.0
    store.write_robot_motion("clip", doubled, second)

    assert store.robots("clip") == ["otherbot", "testbot"]
    np.testing.assert_allclose(store.read_reference_motion("clip", "testbot").joint_positions,
                               motion.joint_positions, rtol=0, atol=1e-6)
    np.testing.assert_allclose(store.read_reference_motion("clip", "otherbot").joint_positions,
                               doubled.joint_positions, rtol=0, atol=1e-6)


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
    actual = getattr(store.read_reference_motion("clip", "testbot"), field)
    assert actual.shape == expected.shape
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-6)


def test_layer_paths_point_at_existing_files(store):
    paths = store.layer_paths("clip")
    assert [path.name for path in paths] == ["clip.rrd", "clip.rrd", "testbot.rbl"]
    assert all(path.exists() for path in paths)
