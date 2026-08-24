"""
The motion sequence — the central data structure of MikuMotionTools — and the Rerun
``.rrd`` store it lives in. Every other module converts into or out of this format.

:class:`MotionSequence` is the in-memory form: plain numpy arrays that converters fill.
:class:`MotionStore` is the on-disk form, a Rerun recording.

A layer is a pipeline stage. A file is one motion segment: a dance, an episode, a take.
The layout under ``root`` (default ``data/motions/``) follows the Rerun dataset convention
of one directory per layer and one file per motion:

    reference/  <motion>.rrd  body poses as exported, robot-agnostic
    <robot>/    <motion>.rrd  everything one robot needs for that motion
    blueprints/ <robot>.rbl   viewer layout for that robot

``reference/`` holds the world-frame body poses and velocities a solve aims at. It has no
joints and needs no robot model, and nothing downstream reads it. It is provenance: the
export as it was, before any robot was chosen.

Each ``<robot>/`` layer is self-contained. It holds that robot's joint positions and
velocities, the goal its links track, and the robot's URDF geometry with the transforms that
animate it. A training run reads that one file and needs nothing else, and so does a
deployment.

A robot layer stores the **goal**, not the pose the solve reached, and says so in the path:
``/<robot>/goal/`` and not ``/<robot>/body/``. The reached pose is the forward kinematics of
the joints stored beside it, and ``/<robot>/tf`` already draws the robot there, so keeping it
would repeat what the file already holds. The goal is recoverable from nothing else. It is
the pose the IK aimed at and could not reach, carried onto the robot's own links and frame
convention, so a tracking reward compares it against the robot directly. The gap between a
goal gizmo and the link under it is the error of the solve, frame by frame.

The layout is also the wire format. A remote dataset mirrors these paths one for one, so a
training run or a deployment reads the same store a workstation wrote. See
:mod:`mikumotion.motion_remote`.

Poses live in a group, one per layer: ``reference/body`` says where the exported rig's bodies
are, and ``<robot>/goal`` says where a robot's links must reach. :func:`pose_group` is the only
place that pairing is written down, so the writer, the reader and the blueprint cannot disagree
about it. One writer and one reader serve either group:

    /<group>/poses/<name>             Transform3D, one entity per body, one row per frame
    /<group>/frames/<name>            a gizmo bound to a coordinate frame, static
    /<group>/linear_velocities        Arrows3D, one row per frame, every body an instance
    /<group>/angular_velocities       Arrows3D
    /reference/body/...               those four, for the exported rig's own bodies
    /<robot>/goal/...                 those four, for where that robot's links must reach
    /<robot>/body/frames/<link>       a gizmo on every link frame of the joint chain
    /<robot>/joint/positions          Scalars, one row per frame, every joint an instance
    /<robot>/joint/velocities         Scalars
    /<robot>/tf                       Transform3D per joint, plus the root link's placement
    /<robot>/tf_static                the URDF's fixed transforms
    /<robot>/visual_geometries/...    the URDF's geometry, named by Rerun's URDF loader

A goal names the robot's own links, never the source rig's bodies, and its rotation is
already carried onto the link's frame convention (see
:class:`mikumotion.motion_retargeting.MotionRetargetingIK`). A reader therefore needs no map
between the two rigs, and the file records none. For zamuza on lite_pro the two vocabularies
differ, ``spine1`` against ``chest``, and only the retarget map in :mod:`mikumotion.presets`
pairs them.

A robot layer's ``body`` is not a pose group. It holds those link gizmos and nothing else,
because the robot's own poses are the forward kinematics of its joints.

A goal names its own frames, ``<robot>/goal/<link>``. The joint chain in ``/<robot>/tf``
already defines the URDF link frames named ``<robot>/<link>``, and a frame can have only one
parent.

Every entity drawn in 3D must name a frame, even one that needs no transform. An entity with
no frame is an orphan. The viewer invents ``tf#<entity path>`` for it, finds no route to the
view's root, and draws an error in place of the entity. Scalars are exempt, because the
viewer plots them against time rather than placing them in the scene.

Filenames match across layers, so ``reference/zamuza.rrd`` and ``lite_pro/zamuza.rrd`` are
one segment at two stages. Every layer shares ``application_id = "mikumotion"`` and
``recording_id = <motion>``. The viewer pools layers by those two ids, so
``rerun data/motions/*/<motion>.rrd`` opens them as one recording.

A robot layer puts its entities under ``/<robot>/`` and prefixes its URDF frames and static
transforms the same way. Two robots, or a robot and the reference rig, would otherwise write
the same entity paths and overwrite each other. Each motion's robot layer carries its own
copy of the geometry. A shared file would need a different recording id, and Rerun cannot
compose a second recording against the motion (rerun-io/rerun#7316).

This module is the only place that touches the Rerun read and write APIs, or the quaternion
order difference (Rerun is xyzw, MotionSequence is wxyz).
"""

from pathlib import Path

import numpy as np
import pyarrow as pa
import rerun as rr
import rerun.blueprint as rrb

from .math import euler_zyx_to_quat, quat_mul

APP_ID = "mikumotion"
TIMELINE = "frame"
REFERENCE = "reference"  # the robot-agnostic layer, both on disk and in entity paths
BLUEPRINTS = "blueprints"  # viewer layouts, one per robot; not a layer, so not a stage
AXIS_LENGTH = 0.15  # metres; long enough to clear the robot's limbs, which would hide it
AXIS_WIDTH = 0.004

#: The per-frame arrays a motion is made of. Single source of truth for copying and storage.
ARRAY_FIELDS = (
    "joint_positions",
    "joint_velocities",
    "body_positions",
    "body_rotations",
    "body_linear_velocities",
    "body_angular_velocities",
)


class MotionSequence:
    """
    A sequence of motion data: joint and body trajectories, all in world frame.

    A plain mutable container — producers allocate one and fill the arrays in place.

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
        """Return the indices of the named bodies, in the order given."""
        for name in body_names:
            assert name in self.body_names, f"unknown body {name!r}, have {self.body_names}"
        return [self.body_names.index(name) for name in body_names]

    def copy(self):
        """Return a deep copy, sharing no arrays with this sequence."""
        other = MotionSequence(self.num_frames, self.joint_names, self.body_names, self.fps)
        for field in ARRAY_FIELDS:
            getattr(other, field)[:] = getattr(self, field)
        return other


def fill_body_velocities(motion):
    """
    Fill the body velocity arrays by differentiating the pose trajectory.

    Every producer calls this instead of differencing poses itself, because the angular
    part has two traps. It must come from the rotation between consecutive frames, because
    Euler-angle rates are not an angular velocity. That rotation must also go the short way
    round. A quaternion can flip sign between frames, and the flipped one reads as a
    near-full turn, which spikes at 2*pi*fps.

    Frame 0 keeps its zeros. There is no earlier pose to difference against.
    """
    dt = 1.0 / motion.fps
    motion.body_linear_velocities[1:] = np.diff(motion.body_positions, axis=0) / dt

    conjugate = motion.body_rotations[:-1] * np.array([1.0, -1.0, -1.0, -1.0])
    step = quat_mul(motion.body_rotations[1:], conjugate)  # world-frame, so post-multiply
    step = np.where(step[..., :1] < 0.0, -step, step)      # the short way round

    axis = step[..., 1:]
    sin_half = np.linalg.norm(axis, axis=-1)
    angle = 2.0 * np.arctan2(sin_half, step[..., 0])
    # angle / sin_half tends to 2 as the step vanishes, which is the value to use there
    scale = np.divide(angle, sin_half, out=np.full_like(angle, 2.0), where=sin_half > 1e-12)
    motion.body_angular_velocities[1:] = axis * (scale / dt)[..., None]


def rotate_motion(motion, z_rotation):
    """
    Return a copy of ``motion`` rotated about the world Z axis by ``z_rotation`` radians.

    The rotation leaves the joint angles alone, and turns every world-frame field.
    """
    zero = np.zeros(1, dtype=np.float32)
    rotation = euler_zyx_to_quat(zero, zero, np.array([z_rotation], dtype=np.float32))[0]
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


# ============================================================
# RERUN STORAGE
# ============================================================

def quat_to_xyzw(quat_wxyz):
    """Convert a (..., 4) wxyz quaternion array to the xyzw order Rerun expects."""
    q = np.asarray(quat_wxyz, dtype=np.float64)
    return np.stack([q[..., 1], q[..., 2], q[..., 3], q[..., 0]], axis=-1)


def quat_to_wxyz(quat_xyzw):
    """Convert a (..., 4) xyzw quaternion array back to the wxyz order MotionSequence uses."""
    q = np.asarray(quat_xyzw, dtype=np.float64)
    return np.stack([q[..., 3], q[..., 0], q[..., 1], q[..., 2]], axis=-1)


def body_poses_path(group, body_name):
    """Return the entity path of a body's pose. The name is escaped: MMD bones are Japanese."""
    return f"/{group}/poses/{rr.escape_entity_path_part(body_name)}"


def body_frames_path(group, body_name):
    """Return the entity path of the gizmo drawn on a body's frame."""
    return f"/{group}/frames/{rr.escape_entity_path_part(body_name)}"


def pose_group(layer):
    """
    Return the entity group that holds one layer's poses.

    The reference layer stores where its own bodies are. A robot layer stores no pose of its
    own, because its joints already place its links, so what it stores is the goal they track.

    The group is also the prefix of the coordinate frames those poses define. That keeps a
    reference body apart from a robot's link of the same name, and keeps ``<robot>/goal/<link>``
    apart from ``<robot>/<link>``, the URDF link frame the joint chain defines. A frame can
    have only one parent.
    """
    return f"{REFERENCE}/body" if layer == REFERENCE else f"{layer}/goal"


def axis_triad():
    """
    Return a red/green/blue gizmo along +X/+Y/+Z, as one mesh of three thin boxes.

    This is geometry rather than ``TransformAxes3D`` because only geometry can be bound to
    a coordinate frame. A 3D view that holds both frame-bound geometry and entity-transform
    axes draws neither.
    """
    corners = np.array([[0, -1, -1], [0, -1, 1], [0, 1, -1], [0, 1, 1],
                        [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]], dtype=np.float32)
    faces = np.array([[0, 1, 3], [0, 3, 2], [4, 7, 5], [4, 6, 7], [0, 5, 1], [0, 4, 5],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]],
                     dtype=np.uint32)
    arm = corners * (AXIS_LENGTH, AXIS_WIDTH, AXIS_WIDTH)

    positions, triangles, colors = [], [], []
    for axis, color in enumerate(((255, 70, 70), (70, 220, 70), (90, 140, 255))):
        positions.append(np.roll(arm, axis, axis=1))  # swing the arm onto X, then Y, then Z
        triangles.append(faces + axis * len(corners))
        colors.append(np.tile(color, (len(corners), 1)))
    return rr.Mesh3D(vertex_positions=np.concatenate(positions),
                     triangle_indices=np.concatenate(triangles),
                     vertex_colors=np.concatenate(colors).astype(np.uint8))


def hidden_velocities(layer):
    """
    Return the overrides that start one layer's velocity arrows switched off.

    The velocities are vectors without origins, so every arrow would fan out of the world
    origin. This hides them rather than excluding them from the view, so they still appear in
    the entity tree and one click brings them back. An excluded entity is not in the view at
    all, and the only way back is to edit the view's filter.
    """
    return {f"/{pose_group(layer)}/{quantity}_velocities": rrb.EntityBehavior(visible=False)
            for quantity in ("linear", "angular")}


def reference_blueprint():
    """Return the viewer layout for an exported motion: body frames in 3D, timeline open."""
    return rrb.Blueprint(
        rrb.Spatial3DView(origin="/", name="motion", overrides=hidden_velocities(REFERENCE)),
        rrb.TimePanel(state="expanded"),
    )


def robot_blueprint(robot):
    """
    Return the viewer layout for a motion solved onto ``robot``: the robot in 3D beside its
    joint-angle plots, with the goal gizmos drawn on top of it.

    Seeing both is the point. A goal is what the IK aimed at, so the gap between a goal gizmo
    and the link under it is the retarget's error, frame by frame.

    The robot's *own* link frames start switched off, because all 75 at once bury the robot.
    They are still in the view, at ``/<robot>/body/frames/<link>``, so the entity tree lists
    them and one click brings any of them back. The override sits on the parent, so it covers
    the whole subtree.
    """
    hidden = ({f"/{robot}/body/frames": rrb.EntityBehavior(visible=False)}
              | hidden_velocities(REFERENCE) | hidden_velocities(robot))
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/", name=robot, overrides=hidden),
            rrb.TimeSeriesView(origin=f"/{robot}/joint/positions", name="joint angles"),
            column_shares=[3, 1],
        ),
        rrb.TimePanel(state="expanded"),
    )


def is_numeric_column(field):
    """
    Return True for the float and int component columns, False for the rest.

    Row ids, strings and the frame name column are the rest.

    Components nest their lists (a Transform3D translation arrives as
    ``list<fixed_size_list<float>[3]>``), so unwrap until the leaf type.
    """
    dtype = field.type
    while pa.types.is_list(dtype) or pa.types.is_large_list(dtype) or pa.types.is_fixed_size_list(dtype):
        dtype = dtype.value_type
    return pa.types.is_floating(dtype) or pa.types.is_integer(dtype)


def read_entity_columns(path):
    """
    Read an ``.rrd`` into ``{entity_path: {component_name: numpy array}}``, frame-ordered.

    Rerun 0.36 has no dataframe API, so this streams chunks and stitches them back together.
    Each row holds that frame's whole instance batch, so a body-pose column comes back
    shaped ``(frames, bodies, 3)`` without any reshaping here.

    The filter runs on the chunk manifest, so a static chunk is dropped before its data
    loads. That is what keeps this cheap on a robot layer, where the URDF geometry and the
    gizmos are most of the bytes and none of them are on the timeline.
    """
    store = rr.experimental.RrdReader(str(path)).store()

    rows = {}
    for chunk in store.stream().filter(has_timeline=TIMELINE):
        batch = chunk.to_record_batch()
        frames = np.asarray(batch.column(TIMELINE).to_pylist(), dtype=np.int64)
        entity = rows.setdefault(str(chunk.entity_path), {})
        for field in batch.schema:
            if field.name == TIMELINE or not is_numeric_column(field):
                continue
            component = field.name.split(":")[-1]
            entity.setdefault(component, []).append((frames, batch.column(field.name).to_pylist()))

    columns = {}
    for entity, components in rows.items():
        columns[entity] = {}
        for component, parts in components.items():
            frames = np.concatenate([frame for frame, _ in parts])
            values = [value for _, chunk_values in parts for value in chunk_values]
            order = np.argsort(frames, kind="stable")
            columns[entity][component] = np.array([values[i] for i in order], dtype=np.float64)
    return columns


def read_entity_components(path):
    """
    Return every entity in an ``.rrd`` and the components on it, static entities included.

    ``read_entity_columns`` sees only what is on the timeline, so geometry, gizmos and the
    URDF's static transforms are invisible to it.
    """
    store = rr.experimental.RrdReader(str(path)).store()
    components = {}
    for chunk in store.stream():
        names = {field.name for field in chunk.to_record_batch().schema}
        components.setdefault(str(chunk.entity_path), set()).update(names)
    return components


def read_blueprint_overrides(path):
    """
    Return the entity paths that a file's blueprint starts switched off.

    Hiding an entity is not the same as excluding it. A hidden entity is still in the view,
    so the entity tree lists it and one click brings it back. An excluded one is not in the
    view at all, and only editing the view's filter can recover it.
    """
    reader = rr.experimental.RrdReader(str(path))
    hidden = set()
    for entry in reader.blueprints():
        for chunk in reader.stream(store=entry):
            batch = chunk.to_record_batch()
            for field in batch.schema:
                if not field.name.endswith("EntityBehavior:visible"):
                    continue
                if [False] in batch.column(field.name).to_pylist():
                    override = str(chunk.entity_path).split("/overrides/")[-1]
                    hidden.add("/" + override.removesuffix("/visualizers"))
    return hidden


def read_properties(path):
    """
    Read a recording's properties: fps and body_names, plus robot and joint_names on a robot
    layer, whose body_names name its goal.

    Properties are their own content, so ask for them by name. A plain path expression would
    not do: those exclude the ``__properties`` subtree.
    """
    store = rr.experimental.RrdReader(str(path)).store()
    only_properties = rr.catalog.ContentFilter.nothing().include_properties()
    properties = {}
    for chunk in store.stream().filter(content=only_properties):
        batch = chunk.to_record_batch()
        for field in batch.schema:
            values = batch.column(field.name).to_pylist()
            if values and isinstance(values[0], list):
                properties[field.name.split(":")[-1]] = values[0]
    return properties


def send_bodies(stream, layer, motion):
    """
    Write one layer's body poses, gizmos and velocities under its :func:`pose_group`.

    Each body gets its own entity, so it is addressable by name, and its own gizmo, so a 3D
    view can draw the frame that pose defines. The velocities stay in one batched entity
    each, one row per frame. A row per body would spend more bytes on Rerun row ids than on
    the data. They are siblings of the poses, not children: an Arrows3D below a posed entity
    inherits the pose and turns its vectors.
    """
    group = pose_group(layer)
    repeated = {body for body in motion.body_names if motion.body_names.count(body) > 1}
    assert not repeated, (
        f"body names must be unique, but {sorted(repeated)} repeat. Each body is stored at "
        f"/{group}/poses/<name>, so two bodies sharing a name would overwrite each other. "
        f"A mocap preset that fills two SMPL slots from one bone needs a distinct name for "
        f"the second slot (see MOCAP_RIGS in mikumotion.presets), and a retarget map needs "
        f"a distinct link per source body (see RETARGET_MAPS)."
    )
    frames = rr.TimeColumn(TIMELINE, sequence=np.arange(motion.num_frames))
    for index, body in enumerate(motion.body_names):
        # The pose names a frame, the shape the URDF importer uses. Every layer is then one
        # graph rooted at "world", so a layer's gizmos and a robot's geometry share one view.
        frame = f"{group}/{body}"
        rr.send_columns(body_poses_path(group, body), [frames], rr.Transform3D.columns(
            translation=motion.body_positions[:, index],
            quaternion=quat_to_xyzw(motion.body_rotations[:, index]),
            child_frame=[frame] * motion.num_frames,
            parent_frame=["world"] * motion.num_frames,
        ), recording=stream)
        rr.log(body_frames_path(group, body), axis_triad(), rr.CoordinateFrame(frame=frame),
               static=True, recording=stream)

    bodies_per_frame = [motion.num_bodies] * motion.num_frames
    for name, vectors in (("linear", motion.body_linear_velocities),
                          ("angular", motion.body_angular_velocities)):
        # The velocities are world-frame vectors, so say so. An entity that names no frame
        # is an orphan, and the viewer draws an error in its place.
        rr.log(f"/{group}/{name}_velocities", rr.CoordinateFrame(frame="world"),
               static=True, recording=stream)
        rr.send_columns(f"/{group}/{name}_velocities", [frames], rr.Arrows3D.columns(
            vectors=vectors.reshape(-1, 3),
        ).partition(bodies_per_frame), recording=stream)


def columns_to_motion(columns, properties, layer):
    """
    Read one layer's columns back into a MotionSequence: the read side of :func:`send_bodies`.

    ``columns`` and ``properties`` come from one file, each read once. The bodies are looked
    up by name rather than taken in file order, which no archetype promises to preserve.

    A robot layer carries joints too. The reference layer has none, and Rerun drops a property
    whose list is empty, so the missing ``joint_names`` is the signal.
    """
    group = pose_group(layer)
    body_names = [str(value) for value in properties["body_names"]]
    joint_names = [str(value) for value in properties.get("joint_names", [])]
    poses = [columns[body_poses_path(group, body)] for body in body_names]

    motion = MotionSequence(
        num_frames=len(poses[0]["translation"]),
        joint_names=joint_names,
        body_names=body_names,
        fps=int(properties["fps"][0]),
    )
    # a Transform3D row is a batch of one, so drop that instance axis
    for index, pose in enumerate(poses):
        motion.body_positions[:, index] = pose["translation"].reshape(-1, 3)
        motion.body_rotations[:, index] = quat_to_wxyz(pose["quaternion"].reshape(-1, 4))
    motion.body_linear_velocities[:] = columns[f"/{group}/linear_velocities"]["vectors"]
    motion.body_angular_velocities[:] = columns[f"/{group}/angular_velocities"]["vectors"]
    if joint_names:
        motion.joint_positions[:] = columns[f"/{layer}/joint/positions"]["scalars"]
        motion.joint_velocities[:] = columns[f"/{layer}/joint/velocities"]["scalars"]
    return motion


class MotionStore:
    """A directory of Rerun motion layers. See the module docstring for the layout."""

    def __init__(self, root="data/motions"):
        self.root = Path(root)

    def reference_file(self, name):
        return self.root / REFERENCE / f"{name}.rrd"

    def robot_file(self, name, robot):
        return self.root / robot / f"{name}.rrd"

    def blueprint_file(self, robot):
        return self.root / BLUEPRINTS / f"{robot}.rbl"

    def robots(self, name):
        """Return the robots this motion has been solved for, read from the layers on disk."""
        layers = (path for path in self.root.iterdir() if path.is_dir())
        return sorted(layer.name for layer in layers
                      if layer.name not in (REFERENCE, BLUEPRINTS)
                      and (layer / f"{name}.rrd").exists())

    def write_reference_motion(self, name, motion):
        """
        Write the export stage as ``reference/<name>.rrd``: body poses and velocities, no
        robot.

        Joints are never written here, even when the sequence carries them. A joint angle
        only means something for one robot, so it belongs to that robot's layer.
        """
        path = self.reference_file(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        with rr.RecordingStream(APP_ID, recording_id=name) as stream:
            stream.save(path, reference_blueprint())
            rr.send_property("motion", rr.AnyValues(
                fps=motion.fps,
                body_names=list(motion.body_names),
            ), recording=stream)

            send_bodies(stream, REFERENCE, motion)
        return path

    def write_robot_motion(self, name, motion, urdf_path, goal=None):
        """
        Write the solve stage as ``<robot>/<name>.rrd``, plus that robot's viewer layout.

        The layer holds the robot's joints, the goal its links track, and the robot's
        geometry, so the file animates the robot on its own and a training run reads nothing
        else. Everything sits under ``/<robot>/``, including the URDF's frames and its static
        transforms, so you can open a second robot beside it.

        ``motion`` carries the robot's joints and, in ``body_names``, at least the root link.
        The joint chain cannot place the robot in the world, so the root's reached pose is
        written into ``/<robot>/tf`` with the joints. The rest of the reached poses are not
        stored at all: they are the forward kinematics of those joints.

        ``goal`` is the pose the links were solved for, named by the robot's own links and in
        their frame convention, which is what a tracking reward compares against. It defaults
        to ``motion``, which is what an imported log wants: a log reached its own goal, so the
        two are one pose set.
        """
        goal = motion if goal is None else goal
        assert (goal.num_frames, goal.fps) == (motion.num_frames, motion.fps), (
            f"the goal must be sampled with the motion, but {goal!r} is not {motion!r}. Both "
            f"go on one frame timeline, so a different length or rate would store the goal "
            f"against the wrong frames."
        )
        robot = rr.urdf.UrdfTree.from_file_path(urdf_path).name
        tree = rr.urdf.UrdfTree.from_file_path(
            urdf_path, frame_prefix=f"{robot}/",
            static_transform_entity_path=f"{robot}/tf_static")
        frames = rr.TimeColumn(TIMELINE, sequence=np.arange(motion.num_frames))
        prefix = f"/{robot}"

        blueprint = self.blueprint_file(robot)
        blueprint.parent.mkdir(parents=True, exist_ok=True)
        robot_blueprint(robot).save(APP_ID, blueprint)

        path = self.robot_file(name, robot)
        path.parent.mkdir(parents=True, exist_ok=True)
        with rr.RecordingStream(APP_ID, recording_id=name) as stream:
            stream.save(path, robot_blueprint(robot))
            rr.send_property(robot, rr.AnyValues(
                robot=robot,
                fps=motion.fps,
                joint_names=list(motion.joint_names),
                body_names=list(goal.body_names),  # the goal's: the only poses stored here
            ), recording=stream)

            tree.log_urdf_to_recording(recording=stream)

            # A gizmo on every link frame, so you can read the solve's own frames off the
            # robot. These ride the URDF chain, so they cover every link, not only the ones
            # the solve tracked, and they show where the robot actually got to. The blueprint
            # hides them by default.
            links = [tree.root_link()] + [tree.get_joint_child(joint) for joint in tree.joints()]
            for link in links:
                rr.log(body_frames_path(f"{robot}/body", link.name), axis_triad(),
                       rr.CoordinateFrame(frame=f"{robot}/{link.name}"),
                       static=True, recording=stream)

            send_bodies(stream, robot, goal)

            joints_per_frame = [motion.num_joints] * motion.num_frames
            rr.send_columns(f"{prefix}/joint/positions", [frames], rr.Scalars.columns(
                scalars=motion.joint_positions.reshape(-1),
            ).partition(joints_per_frame), recording=stream)
            rr.send_columns(f"{prefix}/joint/velocities", [frames], rr.Scalars.columns(
                scalars=motion.joint_velocities.reshape(-1),
            ).partition(joints_per_frame), recording=stream)

            # the frame graph that puts the geometry where the joints say it is
            for joint in tree.joints():
                if joint.name not in motion.joint_names:
                    continue
                angles = motion.joint_positions[:, motion.joint_names.index(joint.name)]
                rr.send_columns(f"{prefix}/tf", [frames], joint.compute_transform_columns(
                    angles.astype(np.float64), clamp=False), recording=stream)

            root = tree.root_link().name
            index = motion.body_names.index(root)
            rr.send_columns(f"{prefix}/tf", [frames], rr.Transform3D.columns(
                translation=motion.body_positions[:, index],
                quaternion=quat_to_xyzw(motion.body_rotations[:, index]),
                child_frame=[f"{robot}/{root}"] * motion.num_frames,
                parent_frame=["world"] * motion.num_frames,
            ), recording=stream)

        return path

    def read_reference_motion(self, name):
        """
        Read the export stage, ``reference/<name>.rrd``, back into a MotionSequence.

        The bodies are the exported rig's own and there are no joints, because a joint angle
        means something for one robot only. Read that robot's layer for its joints and for
        the goal a training run tracks.
        """
        path = self.reference_file(name)
        return columns_to_motion(read_entity_columns(path), read_properties(path), REFERENCE)

    def read_robot_motion(self, name, robot):
        """
        Read ``<robot>/<name>.rrd`` back into a MotionSequence, out of that one file.

        The joints are the ones the solve reached. The body arrays are the **goal** they were
        solved for, named by the robot's own links, which is what a training run tracks. A
        training image therefore needs this file and no other.
        """
        path = self.robot_file(name, robot)
        return columns_to_motion(read_entity_columns(path), read_properties(path), robot)

    def paths(self):
        """
        Return every file in the store, as store-relative posix paths.

        This is the address a remote mirrors: ``root / path`` here, the same ``path`` there.
        See :mod:`mikumotion.motion_remote`, which selects the motion files out of the listing.
        """
        return [path.relative_to(self.root).as_posix()
                for path in self.root.rglob("*") if path.is_file()]

    def layer_paths(self, name):
        """
        Return every file that makes up ``name``, for handing to the Rerun viewer.

        The layers share a recording id, so opening them together is all the composition
        Rerun needs. A motion with no robot layer is still viewable, as body gizmos.
        """
        paths = [self.reference_file(name)]
        for robot in self.robots(name):
            paths += [self.robot_file(name, robot), self.blueprint_file(robot)]
        return [path for path in paths if path.exists()]
