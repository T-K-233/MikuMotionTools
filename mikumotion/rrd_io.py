"""
Read and write the Rerun ``.rrd`` motion sequence — the central data structure of
MikuMotionTools. Every other module is a converter into or out of this format.

Layout under ``root`` (default ``data/motions/``), following the Rerun dataset convention
of one directory per layer and one file per motion:

    base/       <name>.rrd   the motion sequence itself — this is what training consumes
    preview/    <name>.rrd   joint transforms that animate the robot in the viewer
    robot/      <robot>.rrd  URDF geometry + static transforms, shared by every motion
    blueprints/ <robot>.rbl  viewer layout used when previewing a motion on its robot

``base/<name>.rrd`` is self-sufficient and needs no robot model: it carries the joint
names/positions/velocities and the world-frame body transforms and velocities. It is also
by far the smallest layer, because the other layers exist only so a human can *watch* the
motion. Keeping them separate is what lets a training job pull the data without the
geometry, and a viewer pull the geometry once for many motions.

Rerun composes data only within a single recording, so previewing a motion means rewriting
the shared layers onto the motion's recording id and merging them (see ``mikumotion.cli``).
Every recording uses ``application_id = "mikumotion"`` and ``recording_id = <motion name>``
so catalog queries stay predictable.

This module is the only place that touches the Rerun read/write APIs or the quaternion
order difference (Rerun is xyzw, MotionSequence is wxyz), so an SDK change is a one-file fix.
"""

from pathlib import Path

import numpy as np
import pyarrow as pa
import rerun as rr
import rerun.blueprint as rrb

from .motion_sequence import MotionSequence

APP_ID = "mikumotion"
TIMELINE = "frame"


def quat_to_xyzw(quat_wxyz):
    """Convert a (..., 4) wxyz quaternion array to the xyzw order Rerun expects."""
    q = np.asarray(quat_wxyz, dtype=np.float64)
    return np.stack([q[..., 1], q[..., 2], q[..., 3], q[..., 0]], axis=-1)


def quat_to_wxyz(quat_xyzw):
    """Convert a (..., 4) xyzw quaternion array back to the wxyz order MotionSequence uses."""
    q = np.asarray(quat_xyzw, dtype=np.float64)
    return np.stack([q[..., 3], q[..., 0], q[..., 1], q[..., 2]], axis=-1)


def motion_blueprint():
    """Viewer layout: the robot in 3D beside its joint-angle plots, with the timeline open."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/", name="robot"),
            rrb.TimeSeriesView(origin="/signals/joint", name="joint angles"),
            column_shares=[3, 1],
        ),
        rrb.TimePanel(state="expanded"),
    )


def is_numeric_column(field):
    """
    True for the float/int component columns; skips row ids, strings and frame names.

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
    Component values arrive one list per row; the leading element is the value itself.
    """
    store = rr.experimental.RrdReader(str(path)).store()

    rows = {}
    for chunk in store.stream():
        batch = chunk.to_record_batch()
        names = [field.name for field in batch.schema]
        if TIMELINE not in names:
            continue
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


def read_properties(path):
    """Read the recording properties (fps, robot, joint_names, body_names) from an ``.rrd``."""
    store = rr.experimental.RrdReader(str(path)).store()
    properties = {}
    for chunk in store.stream():
        if not str(chunk.entity_path).startswith(rr.RECORDING_PROPERTIES_PATH):
            continue
        batch = chunk.to_record_batch()
        for field in batch.schema:
            values = batch.column(field.name).to_pylist()
            if values and isinstance(values[0], list):
                properties[field.name.split(":")[-1]] = values[0]
    return properties


class MotionStore:
    """A directory of Rerun motion layers. See the module docstring for the layout."""

    def __init__(self, root="data/motions"):
        self.root = Path(root)

    def motion_file(self, name):
        return self.root / "base" / f"{name}.rrd"

    def preview_file(self, name):
        return self.root / "preview" / f"{name}.rrd"

    def robot_file(self, robot):
        return self.root / "robot" / f"{robot}.rrd"

    def blueprint_file(self, robot):
        return self.root / "blueprints" / f"{robot}.rbl"

    def write_motion(self, name, motion):
        """
        Write a MotionSequence as ``base/<name>.rrd``.

        Takes no robot model: a motion exported from a Blender armature has no URDF, and
        training does not need one. Call ``write_preview`` to make the motion viewable.
        """
        frames = rr.TimeColumn(TIMELINE, sequence=np.arange(motion.num_frames))

        path = self.motion_file(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        with rr.RecordingStream(APP_ID, recording_id=name) as stream:
            stream.save(path, motion_blueprint())
            rr.send_property("motion", rr.AnyValues(
                fps=int(np.asarray(motion.fps).reshape(-1)[0]),
                joint_names=list(motion.joint_names),
                body_names=list(motion.body_names),
            ), recording=stream)

            # World-frame body poses, body velocities and joint signals. Each is one entity
            # holding every body/joint as an instance, so there is a single row per frame:
            # a row per body would spend more bytes on Rerun row ids than on the data.
            # The velocity arrows carry vectors only; origins would duplicate body_positions.
            bodies_per_frame = [len(motion.body_names)] * motion.num_frames
            joints_per_frame = [len(motion.joint_names)] * motion.num_frames

            rr.send_columns("/bodies", [frames], rr.InstancePoses3D.columns(
                translations=motion.body_positions.reshape(-1, 3),
                quaternions=quat_to_xyzw(motion.body_rotations).reshape(-1, 4),
            ).partition(bodies_per_frame), recording=stream)

            rr.send_columns("/velocities/linear", [frames], rr.Arrows3D.columns(
                vectors=motion.body_linear_velocities.reshape(-1, 3),
            ).partition(bodies_per_frame), recording=stream)
            rr.send_columns("/velocities/angular", [frames], rr.Arrows3D.columns(
                vectors=motion.body_angular_velocities.reshape(-1, 3),
            ).partition(bodies_per_frame), recording=stream)

            rr.send_columns("/signals/joint", [frames], rr.Scalars.columns(
                scalars=motion.joint_positions.reshape(-1),
            ).partition(joints_per_frame), recording=stream)
            rr.send_columns("/signals/joint_velocity", [frames], rr.Scalars.columns(
                scalars=motion.joint_velocities.reshape(-1),
            ).partition(joints_per_frame), recording=stream)

        return path

    def write_preview(self, name, motion, urdf_path):
        """
        Write everything needed to *watch* ``name`` on the robot described by ``urdf_path``:
        the shared robot geometry and viewer layout, plus a ``preview/<name>.rrd`` layer whose
        joint transforms animate that geometry.

        This layer restates ``joint_positions`` as per-joint transforms, which is why it is
        kept out of ``base/`` — it is the largest layer and only a viewer needs it.
        """
        tree = rr.urdf.UrdfTree.from_file_path(urdf_path, entity_path_prefix="/robot")
        frames = rr.TimeColumn(TIMELINE, sequence=np.arange(motion.num_frames))

        robot_path = self.robot_file(tree.name)
        robot_path.parent.mkdir(parents=True, exist_ok=True)
        with rr.RecordingStream(APP_ID, recording_id=tree.name) as stream:
            stream.save(robot_path)
            tree.log_urdf_to_recording(recording=stream)

        blueprint = self.blueprint_file(tree.name)
        blueprint.parent.mkdir(parents=True, exist_ok=True)
        motion_blueprint().save(APP_ID, blueprint)

        path = self.preview_file(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        with rr.RecordingStream(APP_ID, recording_id=name) as stream:
            stream.save(path)
            rr.send_property("preview", rr.AnyValues(robot=tree.name), recording=stream)

            for joint in tree.joints():
                if joint.name not in motion.joint_names:
                    continue
                angles = motion.joint_positions[:, motion.joint_names.index(joint.name)]
                rr.send_columns("/tf", [frames], joint.compute_transform_columns(
                    angles.astype(np.float64), clamp=False), recording=stream)

            root = tree.root_link().name
            index = motion.body_names.index(root)
            rr.send_columns("/tf", [frames], rr.Transform3D.columns(
                translation=motion.body_positions[:, index],
                quaternion=quat_to_xyzw(motion.body_rotations[:, index]),
                child_frame=[root] * motion.num_frames,
                parent_frame=["world"] * motion.num_frames,
            ), recording=stream)

        return path

    def read_motion(self, name):
        """Read ``base/<name>.rrd`` back into a MotionSequence."""
        path = self.motion_file(name)
        properties = read_properties(path)
        columns = read_entity_columns(path)

        bodies = columns["/bodies"]
        motion = MotionSequence(
            num_frames=len(bodies["translations"]),
            # a motion exported from a mocap armature has bodies but no joints, and an
            # empty name list is not written at all, so treat it as absent
            joint_names=[str(value) for value in properties.get("joint_names", [])],
            body_names=[str(value) for value in properties["body_names"]],
            fps=int(properties["fps"][0]),
        )
        motion.body_positions[:] = bodies["translations"]
        motion.body_rotations[:] = quat_to_wxyz(bodies["quaternions"])
        motion.body_linear_velocities[:] = columns["/velocities/linear"]["vectors"]
        motion.body_angular_velocities[:] = columns["/velocities/angular"]["vectors"]

        if motion.num_joints:
            motion.joint_positions[:] = columns["/signals/joint"]["scalars"]
            motion.joint_velocities[:] = columns["/signals/joint_velocity"]["scalars"]
        return motion

    def layer_paths(self, name):
        """
        Every file that makes up ``name``, for handing to the Rerun viewer.

        A motion that was never given a preview layer is still viewable — you just get its
        body poses and signals rather than the robot's geometry.
        """
        paths = [self.motion_file(name)]
        preview = self.preview_file(name)
        if not preview.exists():
            return paths

        robot = str(read_properties(preview)["robot"][0])
        paths += [preview, self.robot_file(robot), self.blueprint_file(robot)]
        return [path for path in paths if path.exists()]
