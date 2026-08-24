"""
Direction 2, robot -> animation, step 2 of 3: a motion becomes the robot's own animation.

Builds the robot's armature from its URDF, with its visual meshes, and plays a stored motion on
it. With ``--video`` it also renders that animation.

    blender --background --factory-startup \
        --python scripts/blender/convert_motion_to_robot.py -- <motion> <urdf> [--video out.mp4]

The motion is named, not a path: it is read from the store (see mikumotion.motion_sequence),
which needs rerun-sdk inside Blender's own Python:

    <blender>/python/bin/python.exe -m pip install rerun-sdk
"""

import argparse
import os
import sys

import bpy

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from mikumotion import blender, blender_scene  # noqa: E402


def main():
    parser = argparse.ArgumentParser(prog="convert_motion_to_robot.py")
    parser.add_argument("motion", help="motion name in the store")
    parser.add_argument("urdf", help="robot URDF")
    parser.add_argument("--video", help="also render an mp4 here")
    args = parser.parse_args(blender.script_args())

    if "Cube" in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects["Cube"], do_unlink=True)

    motion, _ = blender.motion_to_robot_armature(args.motion, args.urdf, with_meshes=True)
    blender_scene.dress_scene(motion, "MATERIAL")

    if args.video:
        blender_scene.render_animation(args.video)


if __name__ == "__main__":
    main()
