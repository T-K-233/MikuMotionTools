import numpy as np
import bpy
from dataclasses import dataclass
from typing import Any

from bpy.types import Object, PoseBone
from mathutils import Matrix, Quaternion, Vector


from .armature_tree import ArmatureTree

# ============================================================
# RETARGET CONFIG
# ============================================================

_current_retarget_config: "RetargetConfig | None" = None


@dataclass
class RetargetConfig:
    """Configuration for motion retargeting. Set via set_retarget_config() before invoking the operator."""

    source_armature_name: str
    target_armature_name: str
    bone_map: "dict[str, str]"  # target -> source
    translation_root_source: str
    translation_root_target: str
    auto_map_same_names: bool = False
    ignore_twist: bool = False
    bone_axis_local: "Vector" = None
    use_scene_frame_range: bool = True
    frame_start: int = 1
    frame_end: int = 250
    insert_keyframes: bool = True
    clear_existing_keys: bool = True
    frames_per_tick: int = 1
    timer_step_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.bone_axis_local is None:
            self.bone_axis_local = Vector((0.0, 1.0, 0.0))


def set_retarget_config(config: RetargetConfig) -> None:
    """Set the retarget config to use when the modal operator is invoked (by scripts)."""
    global _current_retarget_config
    _current_retarget_config = config


def get_retarget_config() -> "RetargetConfig | None":
    """Get the current retarget config, if any."""
    return _current_retarget_config
from .motion_sequence import MotionSequence
from .math import quat_mul


C = bpy.context
D = bpy.data
O = bpy.ops


"""
Generic quick references:

# getting an armature
armature = D.objects.get("Armature")

# get pose bones
bones = armature.pose.bones

# get bone location
bones.get("bone_name").location

# get bone rotation
bones.get("bone_name").rotation_quaternion

# get bone rotation matrix
bones.get("bone_name").matrix

# get bone scale
bones.get("bone_name").scale

# get edit bones
armature.data.bones
"""


def cleanup_usd_axis_display(display_size: float = 0.01) -> None:
    """
    This function shrinks the axis display of the imported IsaacLab USD object
    to make this looks cleaner.

    Args:
        display_size: the size of the axes
    """
    for object in D.objects:
        if "visuals" in object.name or "collisions" in object.name:
            object.empty_display_size = display_size


def set_scene_fps(fps: int) -> None:
    """
    Set the scene FPS.
    """
    C.scene.render.fps = fps
    C.scene.render.fps_base = 1.0


def set_scene_animation_range(start: int, end: int) -> None:
    """
    Set the animation range for the current scene.

    Args:
        start: The start frame.
        end: The end frame.
    """
    C.scene.frame_start = start
    C.scene.frame_end = end


def set_armature_to_rest(armature: Object) -> None:
    """
    Set the armature to rest pose.
    """
    armature.data.pose_position = "REST"


def set_armature_to_pose(armature: Object) -> None:
    """
    Set the armature to animation pose.
    """
    armature.data.pose_position = "POSE"


def set_bones_to_1d_rotation(armature: Object) -> None:
    """
    Set the bones to 1D rotation mode, and allow only Y-axis rotation (along the bone axis).
    Use this function to convert arbitrary armature to "realistic" robot armature with revolute joints.

    Args:
        armature: The armature object.
    """

    for bone in armature.pose.bones:
        print(f"found bone {bone.name}")
        # set to XYZ rotation mode
        bone.rotation_mode = "XYZ"

        # allow only Y-axis rotation
        bone.lock_rotation[0] = False
        bone.lock_rotation[1] = False
        bone.lock_rotation[2] = False

        # allow only Y-axis rotation in IK
        bone.lock_ik_x = False
        bone.lock_ik_y = False
        bone.lock_ik_z = False


def build_body_motion_data(
    armature: Object,
    bone_names: list[str],
) -> MotionSequence:
    """
    Build rigid body motion data from the source armature.
    The joint motion data is not included in this function, and will be initialized
    as a properly-dimensioned zero array. The joint motion data will be populated during the
    IK retargeting phase of the pipeline.

    Args:
        armature: The source armature object.
        bone_names: The list of bone names to extract.

    Returns:
        A MotionSequence object containing the motion data.
    """
    fps_float = C.scene.render.fps / C.scene.render.fps_base
    start_frame = C.scene.frame_start
    end_frame = C.scene.frame_end

    assert end_frame >= start_frame, f"Frame range is invalid: {start_frame} to {end_frame}"

    n_frames = end_frame - start_frame + 1

    motion = MotionSequence(
        num_frames=n_frames,
        joint_names=[],
        body_names=bone_names,
        fps=fps_float,
    )

    # used to calculate angular velocities
    body_rotations_euler = np.zeros((n_frames, len(motion.body_names), 3), dtype=np.float32)

    # === extract motion data ===
    for frame in range(n_frames):
        # navigate to the corresponding frame
        bpy.context.scene.frame_set(start_frame + frame)
        bpy.context.view_layer.update()

        # force UI update to update bone pose matrix
        bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)

        # read bone positions
        for idx, bone_name in enumerate(motion.body_names):

            source_bone: PoseBone = armature.pose.bones.get(bone_name)
            if not source_bone:
                print(f"WARNING: cannot find source bone {bone_name}")
                continue

            # bone position is defined by the head
            bone_position: Vector = source_bone.head.copy()
            motion._body_positions[frame, idx, :] = bone_position

            # get the rotation offset in (w, x, y, z) quaternion
            bone_rotation: Quaternion = source_bone.matrix.to_quaternion().copy()
            motion._body_rotations[frame, idx, :] = bone_rotation

            body_rotations_euler[frame, idx, :] = bone_rotation.to_euler()

        print(f"Processing: #{frame}/{n_frames} ({frame / n_frames * 100:.2f}%)", end="\r")

    # === post-process motion data ===
    # cancel first frame global offset
    offset_x = np.mean(motion._body_positions[0, :, 0])
    offset_y = np.mean(motion._body_positions[0, :, 1])
    motion._body_positions[:, :, 0] -= offset_x
    motion._body_positions[:, :, 1] -= offset_y

    # calculate velocities
    motion._body_linear_velocities[1:] = np.diff(motion._body_positions, axis=0) / (1. / fps_float)

    # calculate angular velocities
    # handle euler angle discontinuity
    # TODO: this is not quite correct, we need to use quaternions to calculate angular velocities
    body_rotations_euler = np.unwrap(body_rotations_euler, axis=0)
    motion._body_angular_velocities[1:] = np.diff(body_rotations_euler, axis=0) / (1. / fps_float)

    # handle euler angle wrapping
    motion._body_angular_velocities = np.unwrap(motion._body_angular_velocities, axis=0)

    print(f"Done generating {n_frames} frames ({n_frames / fps_float:.2f} seconds)")

    return motion


def matrix_from_translation_rotation(
    translation: np.ndarray = np.zeros((3,), dtype=np.float32),
    rotation: np.ndarray = np.array([1, 0, 0, 0], dtype=np.float32),
) -> Matrix:
    """ Convert a transformation to a matrix. """
    quat = Quaternion(rotation)  # (w, x, y, z)
    return Matrix.Translation(Vector(translation)) @ quat.to_matrix().to_4x4()


def build_armature(
    tree: ArmatureTree,
    name="Armature",
    default_length=0.1,
    show_names=True,
    show_axes=True,
):
    """
    Build an armature from an ArmatureTree.

    Args:
        tree: The ArmatureTree to build the armature from.
        name: The name of the armature.
        default_length: The default length of the bones (applied to leaf bones).
        show_names: Whether to show the names of the bones on the armature.
        show_axes: Whether to show the axes of the bones on the armature.
    """
    # delete the existing armature, if any
    armature = D.objects.get(name)
    if armature:
        O.object.mode_set(mode="OBJECT")
        armature.select_set(True)
        O.object.delete()

    O.object.armature_add(enter_editmode=False, align="WORLD", scale=(1, 1, 1))
    armature = C.active_object
    armature.name = name

    # switch to edit mode
    O.object.mode_set(mode="EDIT")
    edit_bones = armature.data.edit_bones

    if (tree.local_translations[0] != 0).any():
        print(f"WARNING: root bone {tree.body_names[0]} has non-zero translation: {tree.local_translations[0]}")
        print("Adding new root bone")
        all_root = edit_bones[0]
        all_root.name = "root"
        all_root.length = default_length
        tree_root_name = tree.body_names[0]
        edit_bones.new(tree_root_name)
        tree_root = edit_bones.get(tree_root_name)
        tree_root.head = Vector(tree.local_translations[0])
        tree_root.tail = Vector(tree.local_translations[0] + [default_length, 0, 0])  # root always points towards forward (+X axis)
        tree_root.parent = all_root

    else:
        # reconfigure root bone
        root = edit_bones[0]
        root.name = tree.body_names[0]
        root.head = Vector(tree.local_translations[0])
        root.tail = Vector([default_length, 0, 0])  # root always points towards forward (+X axis)

    # first, create all bones
    # root is already created by the armature_add operation
    for body_name in tree.body_names[1:]:
        edit_bones.new(body_name)

    # blender uses world frame for bone transformation,
    # so we need to compute the global transformations for all bodies
    global_transforms: list[Matrix] = []
    parent_indices = tree.body_parent_indices

    for body_index in range(tree.num_bodies):
        parent_index = parent_indices[body_index]
        local_transform = matrix_from_translation_rotation(
            tree.local_translations[body_index],
            tree.local_rotations[body_index],
        )
        if parent_index == -1:
            global_transforms.append(local_transform)
        else:
            parent_transform = global_transforms[parent_index]

            transform = parent_transform @ local_transform   # G_child = G_parent @ L_child
            global_transforms.append(transform)

    for body_name in tree.body_names[1:]:
        body_index = tree.get_index(body_name)
        parent_name = tree.get_parent_name(body_name)
        parent_index = tree.get_parent_index(body_name)
        bone = edit_bones.get(body_name)

        parent_bone = edit_bones.get(parent_name)

        world_loc = global_transforms[body_index].to_translation()
        world_rot = global_transforms[body_index].to_quaternion()
        bone.head = world_loc
        bone.parent = parent_bone

        bone_vector = Vector((0, 0, default_length))
        bone_vector.rotate(world_rot)
        print(f"bone {body_name}: {world_loc}, {world_rot}")
        bone.tail = bone.head + bone_vector

        # update the parent to have correct length
        # TODO: this is not very correct, since the bone might be pointing in a different
        # direction rather than towards the child's head
        # offset_from_parent = world_loc - global_transforms[parent_index].to_translation()
        # parent_bone.length = offset_from_parent.length

    O.object.mode_set(mode="OBJECT")

    bones = D.objects.get(name).pose.bones
    for bone in bones:
        bone.rotation_mode = "XYZ"

    armature.data.show_names = show_names
    armature.data.show_axes = show_axes


def bind_to_armature(skeleton_tree: dict):

    armature = D.objects.get("Armature")

    for idx, link_name in enumerate(skeleton_tree["link_names"]):
        frame = D.objects.get(link_name)
        bone_name = skeleton_tree["node_names"][idx]

        print(f"binding {link_name} to {bone_name}")

        # the use of matrix world is to maintain the original transform
        # i.e. the equivalent of "Keep Transform" GUI option

        # save original world matrix
        matrix_world = frame.matrix_world.copy()

        frame.parent = armature
        frame.parent_bone = bone_name
        frame.parent_type = "BONE"

        # restore world matrix to maintain global transform
        frame.matrix_world = matrix_world.copy()


# ============================================================
# HELPERS
# ============================================================

def quat_normalized_safe(q: Quaternion) -> Quaternion:
    mag2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z
    if mag2 < 1e-16:
        return Quaternion((1.0, 0.0, 0.0, 0.0))
    mag = mag2 ** 0.5
    return Quaternion((q.w / mag, q.x / mag, q.y / mag, q.z / mag))


def rot3_orthonormalized(m3: Matrix) -> Matrix:
    r = m3.copy()
    r.normalize()
    return r


def remove_twist_from_quat(q: Quaternion, twist_axis_world: Vector) -> Quaternion:
    qn = quat_normalized_safe(q)
    axis = twist_axis_world.normalized()
    v = Vector((qn.x, qn.y, qn.z))
    proj = axis * v.dot(axis)
    twist = Quaternion((qn.w, proj.x, proj.y, proj.z))
    twist = quat_normalized_safe(twist)
    swing = qn @ twist.inverted()
    return quat_normalized_safe(swing)


def get_bone_depth(arm_obj, bone_name):
    bone = arm_obj.data.bones.get(bone_name)
    d = 0
    while bone and bone.parent:
        d += 1
        bone = bone.parent
    return d


def get_rest_world_matrix(arm_obj, bone_name) -> Matrix:
    return arm_obj.matrix_world @ arm_obj.data.bones[bone_name].matrix_local


def get_pose_world_matrix(arm_obj, bone_name) -> Matrix:
    return arm_obj.matrix_world @ arm_obj.pose.bones[bone_name].matrix


def build_bone_map(
    source_obj: Object,
    target_obj: Object,
    base_bone_map: "dict[str, str]",
    auto_map_same_names: bool = False,
) -> "dict[str, str]":
    bone_map = dict(base_bone_map)

    if auto_map_same_names:
        src_names = set(source_obj.pose.bones.keys())
        tgt_names = set(target_obj.pose.bones.keys())
        for name in sorted(src_names & tgt_names):
            if name not in bone_map:
                bone_map[name] = name

    valid = {}
    for tgt_name, src_name in bone_map.items():
        if source_obj.pose.bones.get(src_name) and target_obj.pose.bones.get(tgt_name):
            valid[tgt_name] = src_name
        else:
            print(f"[WARN] Skipping invalid bone map: target '{tgt_name}' <- source '{src_name}'")
    return valid


def get_source_delta_world(source_obj, source_bone_name):
    src_rest_w = get_rest_world_matrix(source_obj, source_bone_name)
    src_pose_w = get_pose_world_matrix(source_obj, source_bone_name)
    return src_pose_w @ src_rest_w.inverted()


def build_desired_target_pose_matrix_objspace(
    source_obj: Object,
    target_obj: Object,
    source_bone_name: str,
    target_bone_name: str,
    *,
    allow_translation: bool = False,
    translation_source_bone_name: str | None = None,
    ignore_twist: bool = False,
    bone_axis_local: Vector | None = None,
) -> Matrix:
    """
    Builds desired PoseBone.matrix in target ARMATURE OBJECT SPACE.
    - Root: translation + rotation
    - Non-root: rotation only
    """
    if bone_axis_local is None:
        bone_axis_local = Vector((0.0, 1.0, 0.0))

    # ---------------- Rotation source (normal mapped source bone)
    src_delta_w = get_source_delta_world(source_obj, source_bone_name)
    src_delta_world_r = rot3_orthonormalized(src_delta_w.to_3x3())

    if ignore_twist:
        src_bone = source_obj.data.bones[source_bone_name]
        src_rest_rot_w = (source_obj.matrix_world @ src_bone.matrix_local).to_3x3()
        src_rest_rot_w = rot3_orthonormalized(src_rest_rot_w)
        twist_axis_w = (src_rest_rot_w @ bone_axis_local).normalized()

        dq = src_delta_world_r.to_quaternion()
        dq = remove_twist_from_quat(dq, twist_axis_w)
        src_delta_world_r = rot3_orthonormalized(dq.to_matrix())

    tgt_rest_w = get_rest_world_matrix(target_obj, target_bone_name)
    tgt_rest_world_r = rot3_orthonormalized(tgt_rest_w.to_3x3())

    tgt_desired_world_r = src_delta_world_r @ tgt_rest_world_r
    tgt_desired_world_r = rot3_orthonormalized(tgt_desired_world_r)

    arm_world_r = rot3_orthonormalized(target_obj.matrix_world.to_3x3())
    tgt_desired_obj_r = arm_world_r.inverted() @ tgt_desired_world_r
    tgt_desired_obj_r = rot3_orthonormalized(tgt_desired_obj_r)

    tgt_pb = target_obj.pose.bones[target_bone_name]
    cur_obj_pose = tgt_pb.matrix.copy()

    # ---------------- Translation source (separate configurable source for root)
    if allow_translation:
        if translation_source_bone_name is None:
            translation_source_bone_name = source_bone_name

        src_delta_w_for_translation = get_source_delta_world(source_obj, translation_source_bone_name)

        tgt_desired_world_full = src_delta_w_for_translation @ tgt_rest_w
        tgt_desired_obj_full = target_obj.matrix_world.inverted() @ tgt_desired_world_full
        loc = tgt_desired_obj_full.to_translation()
    else:
        loc = cur_obj_pose.to_translation()

    return Matrix.LocRotScale(
        loc,
        tgt_desired_obj_r.to_quaternion(),
        Vector((1.0, 1.0, 1.0)),
    )


def clear_existing_keyframes_for_mapped_bones(target_obj, bone_map, translation_root_target):
    """Remove old F-curves for mapped bones (and translation root target)."""
    ad = target_obj.animation_data
    if not ad or not ad.action:
        return

    action = ad.action
    mapped = set(bone_map.keys())
    mapped.add(translation_root_target)

    to_remove = []

    for fc in action.fcurves:
        dp = fc.data_path
        for b in mapped:
            prefix = f'pose.bones["{b}"].'
            if dp.startswith(prefix) and (
                dp.endswith("location") or
                dp.endswith("rotation_quaternion") or
                dp.endswith("scale")
            ):
                to_remove.append(fc)
                break

    for fc in to_remove:
        action.fcurves.remove(fc)

    if to_remove:
        print(f"[INFO] Removed {len(to_remove)} existing F-curves for mapped/translation-root bones.")


def clear_target_pose_for_frame(target_obj, bone_map):
    """
    Reset channels each frame.
    All mapped bones get zero loc / identity rot / unit scale first.
    """
    for tgt_name in bone_map.keys():
        pb = target_obj.pose.bones[tgt_name]
        pb.rotation_mode = 'QUATERNION'
        pb.rotation_quaternion = Quaternion((1.0, 0.0, 0.0, 0.0))
        pb.scale = Vector((1.0, 1.0, 1.0))
        pb.location = Vector((0.0, 0.0, 0.0))
    bpy.context.view_layer.update()


def retarget_frame(
    source_obj: Object,
    target_obj: Object,
    bone_map: "dict[str, str]",
    translation_root_target: str,
    translation_root_source: str,
    *,
    ignore_twist: bool = False,
    bone_axis_local: Vector | None = None,
) -> None:
    sorted_target_bones = sorted(bone_map.keys(), key=lambda n: get_bone_depth(target_obj, n))

    clear_target_pose_for_frame(target_obj, bone_map)

    for tgt_name in sorted_target_bones:
        src_name = bone_map[tgt_name]
        pb = target_obj.pose.bones[tgt_name]

        allow_translation = (tgt_name == translation_root_target)

        desired_obj_pose = build_desired_target_pose_matrix_objspace(
            source_obj=source_obj,
            target_obj=target_obj,
            source_bone_name=src_name,
            target_bone_name=tgt_name,
            allow_translation=allow_translation,
            translation_source_bone_name=(translation_root_source if allow_translation else None),
            ignore_twist=ignore_twist,
            bone_axis_local=bone_axis_local,
        )

        pb.matrix = desired_obj_pose
        pb.scale = Vector((1.0, 1.0, 1.0))

        if not allow_translation:
            pb.location = Vector((0.0, 0.0, 0.0))

        bpy.context.view_layer.update()


# ============================================================
# MODAL OPERATOR (responsive UI, ESC cancel)
# ============================================================

class WM_OT_modal_retarget_motion(bpy.types.Operator):
    bl_idname = "wm.modal_retarget_motion"
    bl_label = "Modal Retarget Motion"
    bl_description = "Retarget animation without freezing Blender UI (ESC to cancel)"
    bl_options = {'REGISTER'}

    _timer = None
    _state = None

    def _cleanup(self, context):
        wm = context.window_manager
        if self._timer is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None
        try:
            wm.progress_end()
        except Exception:
            pass
        if context.area:
            context.area.tag_redraw()

    def _fail(self, context, msg):
        self.report({'ERROR'}, msg)
        print("[ERROR]", msg)
        self._cleanup(context)
        return {'CANCELLED'}

    def invoke(self, context: Any, event: Any) -> Any:
        cfg = get_retarget_config()
        if cfg is None:
            return self._fail(
                context,
                "No retarget config. Call set_retarget_config(RetargetConfig(...)) before invoking.",
            )

        source_obj = bpy.data.objects.get(cfg.source_armature_name)
        target_obj = bpy.data.objects.get(cfg.target_armature_name)

        if source_obj is None or target_obj is None:
            return self._fail(context, "Could not find source/target armature objects.")
        if source_obj.type != 'ARMATURE' or target_obj.type != 'ARMATURE':
            return self._fail(context, "Source/target objects must be ARMATUREs.")

        bone_map = build_bone_map(
            source_obj,
            target_obj,
            cfg.bone_map,
            cfg.auto_map_same_names,
        )
        if not bone_map:
            return self._fail(context, "No valid bone mappings found.")

        if cfg.translation_root_target not in bone_map:
            return self._fail(
                context,
                f'TRANSLATION_ROOT_TARGET "{cfg.translation_root_target}" must be in BONE_MAP target names.',
            )
        if source_obj.pose.bones.get(cfg.translation_root_source) is None:
            return self._fail(
                context,
                f'TRANSLATION_ROOT_SOURCE "{cfg.translation_root_source}" not found in source armature.',
            )

        scene = context.scene
        frame_start = scene.frame_start if cfg.use_scene_frame_range else cfg.frame_start
        frame_end = scene.frame_end if cfg.use_scene_frame_range else cfg.frame_end

        if cfg.clear_existing_keys:
            clear_existing_keyframes_for_mapped_bones(
                target_obj, bone_map, cfg.translation_root_target
            )

        self._state = {
            "source_obj": source_obj,
            "target_obj": target_obj,
            "bone_map": bone_map,
            "translation_root_target": cfg.translation_root_target,
            "translation_root_source": cfg.translation_root_source,
            "frame_start": frame_start,
            "frame_end": frame_end,
            "frame": frame_start,
            "cancelled": False,
            "insert_keyframes": cfg.insert_keyframes,
            "ignore_twist": cfg.ignore_twist,
            "bone_axis_local": cfg.bone_axis_local,
            "frames_per_tick": cfg.frames_per_tick,
        }

        print(f"[INFO] Translation source bone: {cfg.translation_root_source}")
        print(f"[INFO] Translation target bone: {cfg.translation_root_target}")
        print(f"[INFO] Retargeting frames {frame_start}..{frame_end}")
        print(f"[INFO] Bone count: {len(bone_map)}")
        print("[INFO] Modal retarget started. Press ESC to cancel.")

        wm = context.window_manager
        wm.progress_begin(frame_start, frame_end + 1)

        self._timer = wm.event_timer_add(cfg.timer_step_sec, window=context.window)
        wm.modal_handler_add(self)

        if context.area:
            context.area.tag_redraw()

        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if self._state is None:
            return self._fail(context, "Internal state missing.")

        # Allow user cancel
        if event.type in {'ESC'}:
            self._state["cancelled"] = True
            print("[INFO] Retarget cancelled by user.")
            self.report({'WARNING'}, "Retarget cancelled.")
            self._cleanup(context)
            return {'CANCELLED'}

        if event.type != 'TIMER':
            return {'PASS_THROUGH'}

        st = self._state
        scene = context.scene

        try:
            frames_per_tick = st.get("frames_per_tick", 1)
            insert_keyframes = st.get("insert_keyframes", True)
            ignore_twist = st.get("ignore_twist", False)
            bone_axis_local = st.get("bone_axis_local")

            for _ in range(max(1, frames_per_tick)):
                f = st["frame"]
                if f > st["frame_end"]:
                    print("[INFO] Retargeting complete.")
                    self.report({'INFO'}, "Retarget complete.")
                    self._cleanup(context)
                    return {'FINISHED'}

                scene.frame_set(f)
                bpy.context.view_layer.update()

                retarget_frame(
                    source_obj=st["source_obj"],
                    target_obj=st["target_obj"],
                    bone_map=st["bone_map"],
                    translation_root_target=st["translation_root_target"],
                    translation_root_source=st["translation_root_source"],
                    ignore_twist=ignore_twist,
                    bone_axis_local=bone_axis_local,
                )

                if insert_keyframes:
                    for tgt_name in st["bone_map"].keys():
                        pb = st["target_obj"].pose.bones[tgt_name]
                        pb.keyframe_insert(data_path="rotation_quaternion", frame=f)
                        pb.keyframe_insert(data_path="scale", frame=f)
                        pb.keyframe_insert(data_path="location", frame=f)

                context.window_manager.progress_update(f)

                if f % 10 == 0:
                    print(f"[INFO] frame {f}/{st['frame_end']}")

                st["frame"] += 1

            # Request viewport / UI redraw
            for window in context.window_manager.windows:
                for area in window.screen.areas:
                    area.tag_redraw()

            return {'RUNNING_MODAL'}

        except Exception as e:
            import traceback
            traceback.print_exc()
            return self._fail(context, f"Retarget error: {e}")


def load_replay(skeleton_tree: dict, armature: Object, data_path: str):
    # TODO: refactor the replay motion format to use the same format as the motion data
    data = np.load(data_path)

    fps = data["fps"]
    joint_order = data["joint_order"].tolist()
    root_positions = data["root_positions"]
    root_quaternions = data["root_quaternions"]
    joint_positions = data["joint_positions"]


    print(joint_positions.shape)

    n_frames = joint_positions.shape[0]
    n_dof = joint_positions.shape[-1]

    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = n_frames

    for frame in range(n_frames):
        bpy.context.scene.frame_set(frame)
        bpy.context.view_layer.update()

        root = armature.pose.bones.get("pelvis")
        root.location = root_positions[frame]
        # root.location[2] -= 0.6
        root.keyframe_insert(data_path="location", frame=frame)

        root.rotation_mode = "QUATERNION"
        root.rotation_quaternion = root_quaternions[frame]
        root.keyframe_insert(data_path="rotation_quaternion", frame=frame)

        for joint_idx, joint_name in enumerate(joint_order):
            bone_name = joint_name.replace("_joint", "")
            bone = armature.pose.bones.get(bone_name)
            # ensure using Euler angles
            bone.rotation_mode = "XYZ"
            bone.rotation_euler[1] = joint_positions[frame, joint_idx]

            # insert rotation_euler keyframe, for index 1 (Y-axis)
            bone.keyframe_insert(data_path="rotation_euler", index=1, frame=frame)

        print(f"Processing: #{frame}/{n_frames} ({frame / n_frames * 100:.2f}%)")
