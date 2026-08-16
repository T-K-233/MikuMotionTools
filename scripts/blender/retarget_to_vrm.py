"""
Direction 2, robot -> animation: replay a robot motion on a VRM/VRoid character.

The robot's armature is built from its URDF and driven by the motion, then each VRM
humanoid bone is baked from the robot link that carries that segment's world orientation
(``presets.LITE_PRO_TO_VROID_BONE_MAP``).

    blender <character>.blend --background \
        --python scripts/blender/retarget_to_vrm.py -- <motion> <urdf> [--video out.mp4]

Needs rerun-sdk inside Blender's own Python to read the motion:

    <blender>/python/bin/python.exe -m pip install rerun-sdk
"""

import os
import sys

import bpy

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from mikumotion import blender, blender_scene, presets, urdf  # noqa: E402
from mikumotion.rrd_io import MotionStore  # noqa: E402


def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []

    import argparse
    parser = argparse.ArgumentParser(prog="retarget_to_vrm")
    parser.add_argument("motion", help="motion name in the store")
    parser.add_argument("urdf", help="source robot URDF")
    parser.add_argument("--video", help="also render an mp4 here")
    parser.add_argument("--ffmpeg", default="ffmpeg", help="encoder used for --video")
    return parser.parse_args(argv)


def main():
    args = parse_args()
    motion = MotionStore().read_motion(args.motion)
    print(f"{args.motion}: {motion!r}")

    target = next(obj for obj in bpy.data.objects if obj.type == "ARMATURE")
    print("target armature:", target.name)

    # the robot rig is only a carrier for the motion, so it needs no meshes
    robot = urdf.RobotModel.from_file(args.urdf)
    tree = robot.to_armature_tree()
    source = blender.build_robot_from_urdf(robot, name="retarget_source", with_meshes=False)
    blender.load_motion_to_armature(motion, source, tree)

    root_target, root_source = presets.LITE_PRO_TO_VROID_TRANSLATION_ROOT
    blender.bake_retarget(
        source, target, presets.LITE_PRO_TO_VROID_BONE_MAP,
        translation_root_target=root_target,
        translation_root_source=root_source,
        frame_start=1,
        frame_end=motion.num_frames,
    )
    source.hide_set(True)
    source.hide_render = True

    blender_scene.add_ground()
    blender_scene.add_lighting()
    blender_scene.follow_camera(motion, frame_start=1)
    blender_scene.configure_render("TEXTURE")

    if args.video:
        blender_scene.render_animation(args.video, args.ffmpeg)


if __name__ == "__main__":
    main()
