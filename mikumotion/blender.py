"""
Everything that runs inside Blender: building armatures, reading a rig's animation into a
motion, and playing a motion back on a rig.

This module imports ``bpy``, so it only loads inside Blender. The entry points Blender runs
live in ``scripts/blender/``.
"""

import os

import numpy as np
import bpy

from bpy.types import Object, PoseBone
from mathutils import Matrix, Quaternion, Vector

from .armature_tree import ArmatureTree
from .motion_sequence import MotionSequence, fill_body_velocities
from .urdf import RobotModel, rpy_to_quat


C = bpy.context
D = bpy.data
O = bpy.ops


def set_scene_fps(fps: int) -> None:
    """Set the frame rate of the current scene."""
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
    """Show the armature in its rest pose, ignoring any animation."""
    armature.data.pose_position = "REST"


def set_armature_to_pose(armature: Object) -> None:
    """Show the armature in its animated pose."""
    armature.data.pose_position = "POSE"


def armature_to_motion(
    armature: Object,
    bone_names: list[str],
    blender_bone_postfix: str = ".frame",
) -> MotionSequence:
    """
    **animation -> robot, step 1 of 2: into the hub.**
    Reads a Blender armature's animation as a motion. It mirrors
    ``forward_kinematics.robot_log_to_motion``, and inverts ``motion_to_armature``.

    It reads body poses only. A rig has no joint angles. Those come from the IK solve, which
    writes them to the robot's own layer.

    Args:
        armature: The source armature object.
        bone_names: The bones to read, in order. They become the motion's body names.
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

    for frame in range(n_frames):
        # frame_set re-evaluates the depsgraph, so the pose matrices below are current
        bpy.context.scene.frame_set(start_frame + frame)
        bpy.context.view_layer.update()

        for idx, bone_name in enumerate(motion.body_names):

            blender_bone_name = bone_name + blender_bone_postfix
            source_bone: PoseBone = armature.pose.bones.get(blender_bone_name)
            if not source_bone:
                print(f"WARNING: cannot find source bone {blender_bone_name}")
                continue

            # A pose bone's head and matrix are in armature space, so they are only world
            # poses when the armature object happens to sit at the origin unrotated. Go
            # through matrix_world so a placed or turned rig exports the same motion.
            bone_matrix: Matrix = armature.matrix_world @ source_bone.matrix

            # bone position is defined by the head
            bone_position: Vector = armature.matrix_world @ source_bone.head
            motion.body_positions[frame, idx, :] = bone_position

            # get the rotation offset in (w, x, y, z) quaternion
            bone_rotation: Quaternion = bone_matrix.to_quaternion()
            motion.body_rotations[frame, idx, :] = bone_rotation

        print(f"Processing: #{frame}/{n_frames} ({frame / n_frames * 100:.2f}%)", end="\r")

    # === post-process motion data ===
    # cancel first frame global offset
    offset_x = np.mean(motion.body_positions[0, :, 0])
    offset_y = np.mean(motion.body_positions[0, :, 1])
    motion.body_positions[:, :, 0] -= offset_x
    motion.body_positions[:, :, 1] -= offset_y

    fill_body_velocities(motion)

    print(f"Done generating {n_frames} frames ({n_frames / fps_float:.2f} seconds)")

    return motion


def translation_rotation_to_matrix(
    translation: np.ndarray = np.zeros((3,), dtype=np.float32),
    rotation: np.ndarray = np.array([1, 0, 0, 0], dtype=np.float32),
) -> Matrix:
    """ Convert a transformation to a matrix. """
    quat = Quaternion(rotation)  # (w, x, y, z)
    return Matrix.Translation(Vector(translation)) @ quat.to_matrix().to_4x4()


def build_armature(tree: ArmatureTree, name="Armature", default_length=0.1):
    """
    Build an armature from an ArmatureTree.

    Args:
        tree: The ArmatureTree to build the armature from.
        name: The name of the armature.
        default_length: The length given to leaf bones.
    """
    # Start in OBJECT mode. The scene's active object may be in EDIT or POSE mode, another
    # armature for example, and an object operator then fails with "context is incorrect".
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
        local_transform = translation_rotation_to_matrix(
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

    armature.data.show_names = False
    armature.data.show_axes = False


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
        local_transform = translation_rotation_to_matrix(
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


BONE_LENGTH = 0.05


def robot_model_to_armature(robot: RobotModel, name: str, with_meshes: bool) -> Object:
    """
    Build a Blender armature (one bone per link) and the visual meshes for a parsed URDF robot.

    The armature holds one bone per URDF link, with the bone heads at the link origins
    (see ``build_armature``). Each link's visual mesh is then imported and placed at the
    link frame, offset by the visual's own ``<origin>``. It takes a material from its color,
    and parents to the matching bone, so it follows that bone during animation.

    Note: the bone orientations point head to tail, and do not match the URDF link frames.
    Mesh placement therefore uses the link world transforms that
    ``armature_tree_world_transforms`` computes separately. "Keep Transform" parenting then
    holds each mesh in place, whatever its bone's orientation.

    Args:
        robot: The parsed RobotModel (see ``mikumotion.urdf.RobotModel.from_file``).
        name: Name of the armature object.
        with_meshes: Whether to import and attach the visual meshes. A rig used only to
            carry motion into a retarget does not need them.

    Returns:
        The created armature Object.
    """
    tree = robot.to_armature_tree()
    build_armature(tree, name=name, default_length=BONE_LENGTH)
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

            visual_local = translation_rotation_to_matrix(
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

def motion_to_armature(
    motion: MotionSequence,
    armature: Object,
    tree: ArmatureTree,
    frame_start: int = 1,
) -> int:
    """
    **robot -> animation, step 2 of 2: out of the hub.**
    Plays a motion on a Blender armature. It inverts ``armature_to_motion``.

    Drives a faithful armature, one bone per link, from a MotionSequence of per-link world
    poses. It inserts one keyframe per frame, and sets the scene's fps and range.

    Each link's Blender bone takes the pose that lands its rigidly-parented visual mesh on
    that link's world pose from ``motion`` (``body_positions`` and ``body_rotations``). The
    result is *exact*: it merges no ball joints and approximates nothing. Each pose is a
    per-bone ``matrix_basis``, computed straight from the world-space deltas, so no bone
    needs its own ``view_layer.update()``. That is what keeps a long sequence fast.

    The armature must sit at the world origin, as ``robot_model_to_armature`` builds it.
    Its bones must be named by link, to match ``motion.body_names`` and ``tree``.

    Args:
        motion: Source MotionSequence (body frames == URDF link frames).
        armature: The faithful armature Object (visual meshes parented to its bones).
        tree: The ArmatureTree the armature was built from (for rest link frames).
        frame_start: Blender frame number that motion frame 0 maps to.

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

    set_scene_fps(motion.fps)
    set_scene_animation_range(frame_start, frame_start + motion.num_frames - 1)

    for f in range(motion.num_frames):
        bl_frame = frame_start + f

        # 1) world-space delta of each driven link: delta = M_anim @ rest_link^-1
        deltas: "dict[str, Matrix]" = {}
        for name in drive_names:
            idx = body_index[name]
            m_anim = translation_rotation_to_matrix(
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

        if f % 500 == 0:
            print(f"[motion] keyframing {f}/{motion.num_frames}")

    print(f"[motion] keyframed {motion.num_frames} frames onto '{armature.name}' "
          f"({len(drive_names)} bones) at {motion.fps}fps")
    return motion.num_frames


# ============================================================
# HELPERS
# ============================================================

def rot3_orthonormalized(m3: Matrix) -> Matrix:
    """
    Return a copy of ``m3`` with its axes renormalized.

    A chain of matrix products drifts away from orthonormal, and Blender reads the drift as
    scale or shear on the bone.
    """
    r = m3.copy()
    r.normalize()
    return r


def get_bone_depth(arm_obj, bone_name):
    """
    Return how many parents a bone has, counting up to the armature root.

    ``retarget_frame`` poses bones in this order, so a parent is already posed when its
    children read their own world matrices.
    """
    bone = arm_obj.data.bones.get(bone_name)
    d = 0
    while bone and bone.parent:
        d += 1
        bone = bone.parent
    return d


def get_rest_world_matrix(arm_obj, bone_name) -> Matrix:
    """Return a bone's rest matrix in world space, not in armature space."""
    return arm_obj.matrix_world @ arm_obj.data.bones[bone_name].matrix_local


def build_bone_map(source_obj, target_obj, bone_map) -> "dict[str, str]":
    """Drop any target<-source pair naming a bone that one of the armatures lacks."""
    valid = {}
    for tgt_name, src_name in bone_map.items():
        if source_obj.pose.bones.get(src_name) and target_obj.pose.bones.get(tgt_name):
            valid[tgt_name] = src_name
        else:
            print(f"[retarget] skipping '{tgt_name}' <- '{src_name}': bone missing")
    return valid


def desired_pose_matrix(source_obj, target_obj, source_bone_name, target_bone_name) -> Matrix:
    """
    Return the target bone's pose matrix, in target armature space, rotation only.

    The source bone's *world* rotation delta carries onto the target's rest orientation:

        R_target = (R_source_pose . R_source_rest^-1) . R_target_rest

    Cancelling the source's own rest orientation is what makes this work between rigs that
    share a rest pose but not their local bone axes, such as a URDF robot's link frames
    against a VRM's head-to-tail bones. Mapping through the rest orientations instead,
    as (R_target_rest . R_source_rest^-1 . R_source_pose), mangles such a pair in silence.
    """
    # how far the source bone has moved from its own rest pose, in world space
    source_pose = source_obj.matrix_world @ source_obj.pose.bones[source_bone_name].matrix
    source_rest = get_rest_world_matrix(source_obj, source_bone_name)
    source_delta = rot3_orthonormalized((source_pose @ source_rest.inverted()).to_3x3())
    target_rest = rot3_orthonormalized(get_rest_world_matrix(target_obj, target_bone_name).to_3x3())
    desired_world = rot3_orthonormalized(source_delta @ target_rest)

    armature_world = rot3_orthonormalized(target_obj.matrix_world.to_3x3())
    desired_local = rot3_orthonormalized(armature_world.inverted() @ desired_world)

    current = target_obj.pose.bones[target_bone_name].matrix
    return Matrix.LocRotScale(current.to_translation(), desired_local.to_quaternion(),
                              Vector((1.0, 1.0, 1.0)))


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


def retarget_frame(source_obj: Object, target_obj: Object, bone_map: "dict[str, str]") -> None:
    """Pose every mapped target bone for the current frame. Rotation only."""
    clear_target_pose_for_frame(target_obj, bone_map)

    for tgt_name in sorted(bone_map, key=lambda name: get_bone_depth(target_obj, name)):
        pose_bone = target_obj.pose.bones[tgt_name]
        pose_bone.matrix = desired_pose_matrix(source_obj, target_obj, bone_map[tgt_name], tgt_name)
        pose_bone.scale = Vector((1.0, 1.0, 1.0))
        pose_bone.location = Vector((0.0, 0.0, 0.0))
        bpy.context.view_layer.update()


def retarget_armature(
    source_obj: Object,
    target_obj: Object,
    bone_map: "dict[str, str]",
    translation_root_target: str,
    translation_root_source: str,
    frame_start: int,
    frame_end: int,
) -> int:
    """
    **robot -> animation, rig to rig.**
    Transfers an already-posed armature onto a differently-proportioned one, a robot rig
    onto a character rig. It mirrors ``motion_retargeting.MotionRetargetingIK``, which goes
    the other way by IK.

    Bakes a source armature's motion onto a target armature, one keyframe per frame over
    ``[frame_start, frame_end]`` inclusive.

    Every bone takes the source's world rotation delta (see ``desired_pose_matrix``). The
    root also takes a world *position* that follows the source root, offset so the two line
    up at ``frame_start``. Deriving that position from the rest pose instead would turn the
    target's rest hip offset by the root's rotation. The character then sinks through the
    floor as soon as the root turns far, when it lies prone for example.

    Returns the number of frames baked.
    """
    assert source_obj.type == "ARMATURE" and target_obj.type == "ARMATURE", "both must be armatures"

    bone_map = build_bone_map(source_obj, target_obj, bone_map)
    if not bone_map:
        raise ValueError("no valid bone mappings (check source/target bone names)")
    if translation_root_target not in bone_map:
        raise ValueError(f"translation root '{translation_root_target}' is not in the bone map")
    if source_obj.pose.bones.get(translation_root_source) is None:
        raise ValueError(f"source armature has no bone '{translation_root_source}'")

    clear_existing_keyframes_for_mapped_bones(target_obj, bone_map, translation_root_target)

    C.scene.frame_set(frame_start)
    bpy.context.view_layer.update()
    source_root = source_obj.pose.bones[translation_root_source]
    root_offset = (get_rest_world_matrix(target_obj, translation_root_target).to_translation()
                   - (source_obj.matrix_world @ source_root.matrix).to_translation())

    for frame in range(frame_start, frame_end + 1):
        C.scene.frame_set(frame)
        bpy.context.view_layer.update()

        retarget_frame(source_obj, target_obj, bone_map)

        target_root = target_obj.pose.bones[translation_root_target]
        world = target_obj.matrix_world @ target_root.matrix
        world.translation = (source_obj.matrix_world @ source_root.matrix).to_translation() + root_offset
        target_root.matrix = target_obj.matrix_world.inverted() @ world
        bpy.context.view_layer.update()

        for tgt_name in bone_map:
            pose_bone = target_obj.pose.bones[tgt_name]
            pose_bone.keyframe_insert(data_path="rotation_quaternion", frame=frame)
            pose_bone.keyframe_insert(data_path="location", frame=frame)

        if (frame - frame_start) % 100 == 0:
            print(f"[retarget] frame {frame}/{frame_end}")

    baked = frame_end - frame_start + 1
    print(f"[retarget] baked {baked} frames onto '{target_obj.name}' ({len(bone_map)} bones)")
    return baked
