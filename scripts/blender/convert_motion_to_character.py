"""
Direction 2, robot -> animation, step 3 of 3: a motion becomes a VRM character's animation.

Builds the robot's armature from its URDF and drives it with the motion, then bakes each VRM
humanoid bone from the robot link that carries that segment's world orientation
(``presets.LITE_PRO_TO_VROID_BONE_MAP``). The robot rig is only a carrier for the poses, so it
is built without meshes and hidden afterwards. With ``--video`` it also renders the result.

    blender <character>.blend --background \
        --python scripts/blender/convert_motion_to_character.py -- <motion> <urdf> [--video out.mp4]

Open the character's ``.blend`` file, not the robot project: the script takes the first armature
it finds as the retarget target. Reading the motion needs rerun-sdk inside Blender's own Python:

    <blender>/python/bin/python.exe -m pip install rerun-sdk
"""

import argparse
import os
import sys

import bpy

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from mikumotion import blender, blender_scene, presets  # noqa: E402


def main():
    parser = argparse.ArgumentParser(prog="convert_motion_to_character.py")
    parser.add_argument("motion", help="motion name in the store")
    parser.add_argument("urdf", help="source robot URDF")
    parser.add_argument("--video", help="also render an mp4 here")
    args = parser.parse_args(blender.script_args())

    # before the robot rig exists, or "the first armature" would find that one instead
    target = next(obj for obj in bpy.data.objects if obj.type == "ARMATURE")
    print("target armature:", target.name)

    motion, source = blender.motion_to_robot_armature(args.motion, args.urdf, with_meshes=False)

    root_target, root_source = presets.LITE_PRO_TO_VROID_TRANSLATION_ROOT
    blender.retarget_armature(
        source, target, presets.LITE_PRO_TO_VROID_BONE_MAP,
        translation_root_target=root_target,
        translation_root_source=root_source,
        frame_start=1,
        frame_end=motion.num_frames,
    )
    source.hide_set(True)
    source.hide_render = True

    blender_scene.dress_scene(motion, "TEXTURE")

    if args.video:
        blender_scene.render_animation(args.video)


if __name__ == "__main__":
    main()
