"""
Direction 2, robot -> animation: build the robot's armature from its URDF and play a
stored motion on it.

    blender --background --factory-startup \
        --python scripts/blender/animate_robot.py -- <motion> <urdf> [--video out.mp4]

The motion is named, not a path: it is read from the store (see mikumotion.motion_sequence), which
needs rerun-sdk inside Blender's own Python:

    <blender>/python/bin/python.exe -m pip install rerun-sdk
"""

import os
import sys

import bpy

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from mikumotion import blender, blender_scene, urdf  # noqa: E402
from mikumotion.motion_sequence import MotionStore  # noqa: E402


def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []

    import argparse
    parser = argparse.ArgumentParser(prog="animate_robot")
    parser.add_argument("motion", help="motion name in the store")
    parser.add_argument("urdf", help="robot URDF")
    parser.add_argument("--video", help="also render an mp4 here")
    parser.add_argument("--ffmpeg", default="ffmpeg", help="encoder used for --video")
    return parser.parse_args(argv)


def build_scene(motion, urdf_path):
    """Build the robot at its rest pose and key the motion onto it. Returns the armature."""
    if "Cube" in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects["Cube"], do_unlink=True)

    robot = urdf.RobotModel.from_file(urdf_path)
    tree = robot.to_armature_tree()
    armature = blender.build_robot_from_urdf(robot, name=robot.name, with_meshes=True)
    blender.armature_from_motion(motion, armature, tree)
    return armature


def main():
    args = parse_args()
    motion = MotionStore().read_motion(args.motion)
    print(f"{args.motion}: {motion!r}")

    build_scene(motion, args.urdf)
    blender_scene.add_ground()
    blender_scene.add_lighting()
    blender_scene.follow_camera(motion, frame_start=1)
    blender_scene.configure_render("MATERIAL")

    if args.video:
        blender_scene.render_animation(args.video, args.ffmpeg)


if __name__ == "__main__":
    main()
