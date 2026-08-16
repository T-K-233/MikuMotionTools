import os

import numpy as np
import bpy
from dataclasses import dataclass
from typing import Any

from bpy.types import Object, PoseBone
from mathutils import Matrix, Quaternion, Vector


from .armature_tree import ArmatureTree
from .urdf import RobotModel, rpy_to_quat

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
    use_rest_orientation_offsets: bool = True
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
    blender_bone_postfix: str = ".frame",
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

            blender_bone_name = bone_name + blender_bone_postfix
            source_bone: PoseBone = armature.pose.bones.get(blender_bone_name)
            if not source_bone:
                print(f"WARNING: cannot find source bone {blender_bone_name}")
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
    # ensure we start in OBJECT mode; the scene's active object may currently be in
    # EDIT or POSE mode (e.g. another armature), which makes object operators fail with
    # "context is incorrect".
    if C.object is not None and C.object.mode != "OBJECT":
        O.object.mode_set(mode="OBJECT")

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
# URDF -> ARMATURE + MESH
# ============================================================

def armature_tree_world_transforms(tree: ArmatureTree) -> "dict[str, Matrix]":
    """
    Compute the world-space rest transform of every link frame in an ArmatureTree.

    These are the link frames as defined by the URDF/MJCF (NOT the Blender bone
    orientations, which point head->tail). They are used to place visual meshes at
    their correct location and orientation. Assumes the armature object sits at the
    world origin.

    Args:
        tree: The ArmatureTree.

    Returns:
        A dict mapping body name -> 4x4 world transform Matrix.
    """
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
            global_transforms.append(global_transforms[parent_index] @ local_transform)

    return {tree.body_names[i]: global_transforms[i] for i in range(tree.num_bodies)}


def import_stl(filepath: str, name: "str | None" = None) -> Object:
    """
    Import an STL mesh file and return the created object.

    Uses the Blender 4.x importer (``wm.stl_import``), falling back to the legacy
    ``import_mesh.stl`` operator on older Blender versions.

    Args:
        filepath: Path to the STL file.
        name: Optional name to assign to the imported object and its mesh data.

    Returns:
        The imported mesh Object.
    """
    before = set(D.objects)
    if hasattr(O.wm, "stl_import"):
        O.wm.stl_import(filepath=filepath)
    else:
        O.import_mesh.stl(filepath=filepath)

    new_objects = [obj for obj in D.objects if obj not in before]
    if not new_objects:
        raise RuntimeError(f"Failed to import STL: {filepath}")
    obj = new_objects[0]

    if name:
        obj.name = name
        if obj.data:
            obj.data.name = name
    return obj


def get_or_create_material(name: str, rgba) -> "bpy.types.Material":
    """
    Get an existing material by name, or create a new one with the given color.

    Args:
        name: Material name.
        rgba: An (r, g, b) or (r, g, b, a) color, or None.

    Returns:
        The material.
    """
    mat = D.materials.get(name)
    if mat is not None:
        return mat

    mat = D.materials.new(name)
    mat.use_nodes = True
    if rgba is not None:
        r, g, b = float(rgba[0]), float(rgba[1]), float(rgba[2])
        a = float(rgba[3]) if len(rgba) > 3 else 1.0
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf is not None:
            bsdf.inputs["Base Color"].default_value = (r, g, b, a)
        # viewport display color (Solid shading)
        mat.diffuse_color = (r, g, b, a)
    return mat


def parent_object_to_bone(obj: Object, armature: Object, bone_name: str) -> None:
    """
    Parent an object to an armature bone while preserving its world transform.

    Mirrors the behaviour of the GUI "Keep Transform" parenting option.

    Args:
        obj: The object to parent.
        armature: The armature object.
        bone_name: The name of the bone to parent to.
    """
    matrix_world = obj.matrix_world.copy()
    obj.parent = armature
    obj.parent_bone = bone_name
    obj.parent_type = "BONE"
    obj.matrix_world = matrix_world


def build_robot_from_urdf(
    robot: RobotModel,
    name: str = "robot",
    with_meshes: bool = True,
    default_length: float = 0.05,
    show_names: bool = False,
    show_axes: bool = True,
) -> Object:
    """
    Build a Blender armature (one bone per link) and the visual meshes for a parsed URDF robot.

    The armature contains one bone per URDF link, with bone heads placed at the link
    origins (see ``build_armature``). Each link's visual mesh is imported, placed at the
    link frame (offset by the visual's own ``<origin>``), given a material from its color,
    and parented to the corresponding bone so it follows the bone during animation.

    Note: the bone orientations point head->tail and do NOT match the URDF link frames.
    Mesh placement therefore uses the independently-computed link world transforms
    (``armature_tree_world_transforms``), and "Keep Transform" parenting keeps the mesh in
    place regardless of bone orientation.

    Args:
        robot: The parsed RobotModel (see ``mikumotion.urdf.RobotModel.from_file``).
        name: Name of the armature object.
        with_meshes: Whether to import and attach the visual meshes.
        default_length: Default bone length passed to ``build_armature``.
        show_names: Whether to display bone names.
        show_axes: Whether to display bone axes.

    Returns:
        The created armature Object.
    """
    tree = robot.to_armature_tree()
    build_armature(
        tree,
        name=name,
        default_length=default_length,
        show_names=show_names,
        show_axes=show_axes,
    )
    armature = D.objects.get(name)

    # ensure the armature sits at the world origin so link world transforms == placement
    armature.matrix_world = Matrix.Identity(4)

    if not with_meshes:
        return armature

    world_tf = armature_tree_world_transforms(tree)

    num_placed = 0
    for link_name, link in robot.links.items():
        if link_name not in world_tf:
            print(f"WARNING: link {link_name} not found in armature, skipping its meshes")
            continue
        link_world = world_tf[link_name]

        for vi, visual in enumerate(link.visuals):
            if not visual.mesh_path:
                continue
            if not os.path.isfile(visual.mesh_path):
                print(f"WARNING: mesh file not found for {link_name}: {visual.mesh_path}")
                continue

            obj_name = f"{link_name}_visual" if len(link.visuals) == 1 else f"{link_name}_visual_{vi}"
            obj = import_stl(visual.mesh_path, name=obj_name)

            visual_local = matrix_from_translation_rotation(
                visual.origin_xyz,
                rpy_to_quat(visual.origin_rpy),
            )
            scale_mat = Matrix.Diagonal(Vector(visual.scale)).to_4x4()
            obj.matrix_world = link_world @ visual_local @ scale_mat

            if visual.rgba is not None:
                mat = get_or_create_material(visual.material_name or f"{link_name}_material", visual.rgba)
                obj.data.materials.clear()
                obj.data.materials.append(mat)

            parent_object_to_bone(obj, armature, link_name)
            num_placed += 1

    print(f"Built robot '{name}': {tree.num_bodies} bones, {num_placed} visual meshes")
    return armature


# ============================================================
# MOTION -> ARMATURE (drive the faithful rig from a MotionSequence)
# ============================================================

def load_motion_to_armature(
    motion: MotionSequence,
    armature: Object,
    tree: ArmatureTree,
    *,
    frame_start: int = 1,
    frame_stride: int = 1,
    set_scene_range: bool = True,
) -> int:
    """
    Drive a faithful (one-bone-per-link) armature from a MotionSequence of per-link
    world poses, inserting a keyframe per sampled frame.

    Each link's Blender bone is posed so that its rigidly-parented visual mesh lands
    exactly at that link's world pose from ``motion`` (``body_positions`` /
    ``body_rotations``). This is *exact* — no ball-joint merging / approximation. The
    pose is computed analytically as a per-bone ``matrix_basis`` from the world-space
    deltas, so no per-bone ``view_layer.update()`` is needed (keeps long sequences fast).

    The armature must sit at the world origin (as built by ``build_robot_from_urdf``)
    and its bones must be named by link (matching ``motion.body_names`` / ``tree``).

    Args:
        motion: Source MotionSequence (body frames == URDF link frames).
        armature: The faithful armature Object (visual meshes parented to its bones).
        tree: The ArmatureTree the armature was built from (for rest link frames).
        frame_start: Blender frame number that motion frame 0 maps to.
        frame_stride: Sample every Nth motion frame (1 = every frame). Scene FPS is
            scaled by ``1/stride`` so playback speed is preserved.
        set_scene_range: If True, set the scene FPS + frame range from the motion.

    Returns:
        The number of keyframed frames.
    """
    assert armature.type == "ARMATURE", "target must be an armature"
    # the world-pose math assumes the armature object is at the origin
    armature.matrix_world = Matrix.Identity(4)

    rest_world = armature_tree_world_transforms(tree)  # link name -> rest world Matrix

    data_bones = armature.data.bones
    drive_names = [n for n in motion.body_names if n in data_bones and n in rest_world]
    missing = [n for n in motion.body_names if n not in data_bones]
    if missing:
        print(f"[motion] WARNING: {len(missing)} motion bodies have no matching bone (skipped): {missing[:6]}")

    # cache per-bone rest matrix (armature space) + parent bone name; force quaternion mode
    rest_bone: "dict[str, Matrix]" = {}
    parent_name: "dict[str, str | None]" = {}
    for name in drive_names:
        b = data_bones[name]
        rest_bone[name] = b.matrix_local.copy()
        parent_name[name] = b.parent.name if b.parent else None
        armature.pose.bones[name].rotation_mode = "QUATERNION"

    body_index = {n: motion.body_names.index(n) for n in drive_names}
    identity = Matrix.Identity(4)

    motion_fps = int(np.asarray(motion.fps).reshape(-1)[0])
    frames = list(range(0, motion.num_frames, max(1, frame_stride)))
    if set_scene_range:
        set_scene_fps(max(1, int(round(motion_fps / max(1, frame_stride)))))
        set_scene_animation_range(frame_start, frame_start + len(frames) - 1)

    for out_i, f in enumerate(frames):
        bl_frame = frame_start + out_i

        # 1) world-space delta of each driven link: delta = M_anim @ rest_link^-1
        deltas: "dict[str, Matrix]" = {}
        for name in drive_names:
            idx = body_index[name]
            m_anim = matrix_from_translation_rotation(
                motion.body_positions[f, idx],
                motion.body_rotations[f, idx],
            )
            deltas[name] = m_anim @ rest_world[name].inverted()

        # 2) convert each link delta into the bone's local matrix_basis, then keyframe.
        #    Blender: pose = parent_pose @ (parent_rest^-1 @ rest) @ basis, so with each
        #    bone posed to (delta @ rest) the required basis is:
        #        basis = rest^-1 @ delta_parent^-1 @ delta @ rest
        for name in drive_names:
            R = rest_bone[name]
            pn = parent_name[name]
            d_parent = deltas.get(pn, identity) if pn is not None else identity
            basis = R.inverted() @ d_parent.inverted() @ deltas[name] @ R

            pb = armature.pose.bones[name]
            pb.matrix_basis = basis
            pb.keyframe_insert(data_path="location", frame=bl_frame)
            pb.keyframe_insert(data_path="rotation_quaternion", frame=bl_frame)

        if out_i % 100 == 0:
            print(f"[motion] keyframing {out_i}/{len(frames)}", end="\r")

    print(f"\n[motion] keyframed {len(frames)} frames onto '{armature.name}' "
          f"({len(drive_names)} bones), scene {frame_start}..{frame_start + len(frames) - 1} "
          f"@ {C.scene.render.fps}fps (motion {motion_fps}fps)")
    return len(frames)


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


def compute_rest_orientation_offsets(
    source_obj: Object,
    target_obj: Object,
    bone_map: "dict[str, str]",
) -> "dict[str, Matrix]":
    """
    Per target bone:
      R_offset_world = R_tgt_rest_world * inv(R_src_rest_world)
    so we can do:
      R_tgt_pose_world = R_offset_world * R_src_pose_world
    """
    offsets = {}
    for tgt_name, src_name in bone_map.items():
        src_rest_r = rot3_orthonormalized(get_rest_world_matrix(source_obj, src_name).to_3x3())
        tgt_rest_r = rot3_orthonormalized(get_rest_world_matrix(target_obj, tgt_name).to_3x3())
        offsets[tgt_name] = rot3_orthonormalized(tgt_rest_r @ src_rest_r.inverted())
    return offsets


def get_source_pose_world_rot(source_obj: Object, source_bone_name: str) -> Matrix:
    pose_w = get_pose_world_matrix(source_obj, source_bone_name)
    return rot3_orthonormalized(pose_w.to_3x3())


def get_target_desired_world_rotation(
    source_obj: Object,
    target_obj: Object,
    source_bone_name: str,
    target_bone_name: str,
    *,
    rest_rot_offsets: "dict[str, Matrix] | None" = None,
    use_rest_orientation_offsets: bool = True,
    ignore_twist: bool = False,
    bone_axis_local: "Vector | None" = None,
) -> Matrix:
    """
    Returns desired target WORLD rotation (3x3).
    Supports two modes:
      - offset mode: R_tgt = R_offset * R_src_pose
      - old delta mode: R_tgt = delta_src * R_tgt_rest
    """
    if bone_axis_local is None:
        bone_axis_local = Vector((0.0, 1.0, 0.0))

    if use_rest_orientation_offsets and rest_rot_offsets is not None:
        src_pose_r = get_source_pose_world_rot(source_obj, source_bone_name)

        if ignore_twist:
            src_rest_r = rot3_orthonormalized(get_rest_world_matrix(source_obj, source_bone_name).to_3x3())
            delta_r = rot3_orthonormalized(src_pose_r @ src_rest_r.inverted())
            twist_axis_w = (src_rest_r @ bone_axis_local).normalized()
            dq = remove_twist_from_quat(delta_r.to_quaternion(), twist_axis_w)
            delta_r = rot3_orthonormalized(dq.to_matrix())
            src_pose_r = rot3_orthonormalized(delta_r @ src_rest_r)

        tgt_desired_world_r = rot3_orthonormalized(rest_rot_offsets[target_bone_name] @ src_pose_r)
        return tgt_desired_world_r

    # Fallback (old assumption: same rest orientation)
    src_delta_w = get_source_delta_world(source_obj, source_bone_name)
    src_delta_world_r = rot3_orthonormalized(src_delta_w.to_3x3())

    if ignore_twist:
        src_rest_r = rot3_orthonormalized(get_rest_world_matrix(source_obj, source_bone_name).to_3x3())
        twist_axis_w = (src_rest_r @ bone_axis_local).normalized()
        dq = remove_twist_from_quat(src_delta_world_r.to_quaternion(), twist_axis_w)
        src_delta_world_r = rot3_orthonormalized(dq.to_matrix())

    tgt_rest_r = rot3_orthonormalized(get_rest_world_matrix(target_obj, target_bone_name).to_3x3())
    return rot3_orthonormalized(src_delta_world_r @ tgt_rest_r)


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
    rest_rot_offsets: "dict[str, Matrix] | None" = None,
    use_rest_orientation_offsets: bool = True,
) -> Matrix:
    """
    Builds desired PoseBone.matrix in target ARMATURE OBJECT SPACE.
    - Root: translation + rotation
    - Non-root: rotation only
    """
    if bone_axis_local is None:
        bone_axis_local = Vector((0.0, 1.0, 0.0))

    # ---------------- Rotation (with rest-offset mapping support)
    tgt_desired_world_r = get_target_desired_world_rotation(
        source_obj=source_obj,
        target_obj=target_obj,
        source_bone_name=source_bone_name,
        target_bone_name=target_bone_name,
        rest_rot_offsets=rest_rot_offsets,
        use_rest_orientation_offsets=use_rest_orientation_offsets,
        ignore_twist=ignore_twist,
        bone_axis_local=bone_axis_local,
    )

    arm_world_r = rot3_orthonormalized(target_obj.matrix_world.to_3x3())
    tgt_desired_obj_r = rot3_orthonormalized(arm_world_r.inverted() @ tgt_desired_world_r)

    tgt_pb = target_obj.pose.bones[target_bone_name]
    cur_obj_pose = tgt_pb.matrix.copy()

    # ---------------- Translation source (separate configurable source for root)
    if allow_translation:
        if translation_source_bone_name is None:
            translation_source_bone_name = source_bone_name

        src_delta_w_for_translation = get_source_delta_world(source_obj, translation_source_bone_name)
        tgt_rest_w = get_rest_world_matrix(target_obj, target_bone_name)

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
    rest_rot_offsets: "dict[str, Matrix] | None" = None,
    use_rest_orientation_offsets: bool = True,
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
            rest_rot_offsets=rest_rot_offsets,
            use_rest_orientation_offsets=use_rest_orientation_offsets,
        )

        pb.matrix = desired_obj_pose
        pb.scale = Vector((1.0, 1.0, 1.0))

        if not allow_translation:
            pb.location = Vector((0.0, 0.0, 0.0))

        bpy.context.view_layer.update()


def bake_retarget(
    source_obj: Object,
    target_obj: Object,
    bone_map: "dict[str, str]",
    translation_root_target: str,
    translation_root_source: str,
    *,
    frame_start: int,
    frame_end: int,
    ignore_twist: bool = False,
    bone_axis_local: "Vector | None" = None,
    use_rest_orientation_offsets: bool = True,
    clear_existing: bool = True,
    track_root_world_position: bool = False,
) -> int:
    """
    Headless (non-modal) batch retarget: bake source->target animation onto the
    target armature over ``[frame_start, frame_end]`` (inclusive), one keyframe per
    frame. Same math as ``WM_OT_modal_retarget_motion`` but as a plain loop suitable
    for ``blender --background``.

    Args:
        source_obj: Source armature (already animated/keyframed).
        target_obj: Target armature to receive baked keyframes.
        bone_map: ``{target_bone: source_bone}`` mapping (invalid pairs are skipped).
        translation_root_target/source: bone pair whose world translation drives the
            target root (only this target bone receives translation).
        frame_start, frame_end: inclusive Blender frame range to bake.
        ignore_twist: remove twist about ``bone_axis_local`` from transferred rotation.
        use_rest_orientation_offsets: if True, transfer rotation in the *rest-relative*
            (local-delta) sense ``R_tgt = R_tgt_rest·R_src_rest⁻¹·R_src_pose`` — only
            correct when source and target bones share local axis conventions. If False,
            transfer the source's *world* rotation delta ``R_tgt = (R_src_pose·R_src_rest⁻¹)·R_tgt_rest``
            — the right choice when the two rigs have the SAME rest pose (e.g. both
            T-pose) but unrelated local frames, as with a URDF robot -> VRM humanoid.
        clear_existing: remove pre-existing F-curves for the mapped bones first.
        track_root_world_position: override the translation-root bone so its world
            *position* tracks the source root's world position directly (offset so the
            two align at ``frame_start``), instead of the default rest-relative transfer.
            Use this when source and target roots have different rest positions (e.g. a
            URDF robot whose pelvis rest is at the origin vs. a VRM whose hips rest is at
            standing height): the default transfer rotates the target's tall rest offset
            by the root's rotation, which sinks/launches the root on large root rotations
            (e.g. lying prone). Position-only tracking keeps the root where the source is.

    Returns:
        Number of frames baked.
    """
    assert source_obj.type == "ARMATURE" and target_obj.type == "ARMATURE", "both must be armatures"

    bone_map = build_bone_map(source_obj, target_obj, bone_map, auto_map_same_names=False)
    if not bone_map:
        raise ValueError("no valid bone mappings (check source/target bone names)")
    if translation_root_target not in bone_map:
        raise ValueError(f"translation_root_target '{translation_root_target}' not in bone_map")
    if source_obj.pose.bones.get(translation_root_source) is None:
        raise ValueError(f"translation_root_source '{translation_root_source}' missing on source")

    rest_rot_offsets = (
        compute_rest_orientation_offsets(source_obj, target_obj, bone_map)
        if use_rest_orientation_offsets else None
    )

    if clear_existing:
        clear_existing_keyframes_for_mapped_bones(target_obj, bone_map, translation_root_target)

    # for root position tracking: offset so the target root's world position equals the
    # source root's world position, aligned to the target's rest position at frame_start.
    root_pos_offset = None
    if track_root_world_position:
        C.scene.frame_set(frame_start)
        bpy.context.view_layer.update()
        src_pb0 = source_obj.pose.bones[translation_root_source]
        src_w0 = (source_obj.matrix_world @ src_pb0.matrix).to_translation()
        tgt_rest0 = get_rest_world_matrix(target_obj, translation_root_target).to_translation()
        root_pos_offset = tgt_rest0 - src_w0

    n = 0
    for f in range(frame_start, frame_end + 1):
        C.scene.frame_set(f)
        bpy.context.view_layer.update()

        retarget_frame(
            source_obj=source_obj,
            target_obj=target_obj,
            bone_map=bone_map,
            translation_root_target=translation_root_target,
            translation_root_source=translation_root_source,
            ignore_twist=ignore_twist,
            bone_axis_local=bone_axis_local,
            rest_rot_offsets=rest_rot_offsets,
            use_rest_orientation_offsets=use_rest_orientation_offsets,
        )

        if root_pos_offset is not None:
            # keep the baked root rotation, but set its world position to track the source
            src_pb = source_obj.pose.bones[translation_root_source]
            desired_world = (source_obj.matrix_world @ src_pb.matrix).to_translation() + root_pos_offset
            tgt_pb = target_obj.pose.bones[translation_root_target]
            m_world = target_obj.matrix_world @ tgt_pb.matrix
            m_world.translation = desired_world
            tgt_pb.matrix = target_obj.matrix_world.inverted() @ m_world
            bpy.context.view_layer.update()

        for tgt_name in bone_map.keys():
            pb = target_obj.pose.bones[tgt_name]
            pb.keyframe_insert(data_path="rotation_quaternion", frame=f)
            pb.keyframe_insert(data_path="location", frame=f)

        n += 1
        if (f - frame_start) % 100 == 0:
            print(f"[retarget] frame {f}/{frame_end}", end="\r")

    print(f"\n[retarget] baked {n} frames onto '{target_obj.name}' ({len(bone_map)} mapped bones)")
    return n


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

        rest_rot_offsets = (
            compute_rest_orientation_offsets(source_obj, target_obj, bone_map)
            if cfg.use_rest_orientation_offsets
            else None
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
            "rest_rot_offsets": rest_rot_offsets,
            "use_rest_orientation_offsets": cfg.use_rest_orientation_offsets,
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
        print(f"[INFO] Rest orientation offsets: {'ENABLED' if cfg.use_rest_orientation_offsets else 'DISABLED'}")
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
                    rest_rot_offsets=st.get("rest_rot_offsets"),
                    use_rest_orientation_offsets=st.get("use_rest_orientation_offsets", True),
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
