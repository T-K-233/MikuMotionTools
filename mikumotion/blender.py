import numpy as np
import bpy
from bpy.types import Object, PoseBone
from mathutils import Vector, Quaternion, Matrix

from .armature_tree import ArmatureTree
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
    scaling_ratio: float = 1.0,
) -> MotionSequence:
    """
    Build rigid body motion data from the source armature.
    The dof motion data is not included in this function, which will be initialized
    as a properly-dimensioned zero array.

    Args:
        armature: The source armature object.
        bone_names: The list of bone names to extract.
        scaling_ratio: The scaling ratio of the armature.

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
        dof_names=[],
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

    # in Blender, scaling the armature does not scale the retreived bone position, so we need to
    # manually apply the scaling to the sampled data here.
    motion._body_positions[:] *= scaling_ratio

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
