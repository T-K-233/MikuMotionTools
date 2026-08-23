"""
The ``mikumotion`` command line: one entry point for working with motion files.

    mikumotion import <mcap> <mjcf> <urdf>            a robot log becomes a motion
    mikumotion view <name>                            watch a motion in the Rerun viewer
    mikumotion retarget <name> <mjcf> <urdf> <map>    solve a robot's joints for a motion
    mikumotion list                                   show the motions in the store

You address a motion by name, not by path. The layout under ``--root`` decides which stage
lands in which layer (see :mod:`mikumotion.motion_sequence`).
"""

import argparse
import subprocess
import sys
from pathlib import Path

from .motion_sequence import REFERENCE, MotionStore


def run_import(args):
    """A logged motion is both stages at once: the poses are the export, the joints the solve.

    This command solves nothing, because the log already carries the joint angles a robot
    reached. Forward kinematics turns those angles into the poses, and both layers come from
    the one sequence.
    """
    from .forward_kinematics import robot_log_to_motion

    store = MotionStore(args.root)
    name = args.name or Path(args.mcap).stem
    motion = robot_log_to_motion(args.mcap, args.mjcf)
    store.write_reference_motion(name, motion)
    store.write_robot_motion(name, motion, args.urdf)

    print(f"{name}: {motion!r}")
    for path in store.layer_paths(name):
        print(f"  {path}")


def run_view(args):
    """Hand the motion's layers to the viewer, which pools them by recording id."""
    paths = MotionStore(args.root).layer_paths(args.name)
    rerun = str(Path(sys.executable).parent / "rerun")
    subprocess.run([rerun] + [str(path) for path in paths], check=True)


def run_retarget(args):
    from .motion_retargeting import MotionRetargetingIK
    from .presets import RETARGET_MAPS

    store = MotionStore(args.root)
    mapping = RETARGET_MAPS[args.mapping]
    MotionRetargetingIK(args.name, args.mjcf, store, mapping, args.urdf).run(args.view)


def run_list(args):
    store = MotionStore(args.root)
    for path in sorted((store.root / REFERENCE).glob("*.rrd")):
        motion = store.read_reference_motion(path.stem)
        solved = store.robots(path.stem)
        print(f"{path.stem:30s} {motion!r}  solved for: {', '.join(solved) or '-'}")


def main():
    parser = argparse.ArgumentParser(prog="mikumotion", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="data/motions", help="motion store directory")
    commands = parser.add_subparsers(dest="command", required=True)

    importer = commands.add_parser("import", help="convert a robot .mcap log into a motion")
    importer.add_argument("mcap", help="ROS2 .mcap log")
    importer.add_argument("mjcf", help="robot MJCF, used for forward kinematics")
    importer.add_argument("urdf", help="robot URDF, whose geometry goes in the robot layer")
    importer.add_argument("--name", help="motion name (defaults to the .mcap file name)")
    importer.set_defaults(handler=run_import)

    viewer = commands.add_parser("view", help="open a motion in the Rerun viewer")
    viewer.add_argument("name")
    viewer.set_defaults(handler=run_view)

    retarget = commands.add_parser("retarget", help="solve a robot's joints for a motion")
    retarget.add_argument("name")
    retarget.add_argument("mjcf", help="target robot MJCF, used for IK")
    retarget.add_argument("urdf", help="target robot URDF, whose geometry goes in the layer")
    retarget.add_argument("mapping", help="body map from mikumotion.presets.RETARGET_MAPS")
    retarget.add_argument("--view", action="store_true", help="watch the solve in a MuJoCo window")
    retarget.set_defaults(handler=run_retarget)

    listing = commands.add_parser("list", help="show the motions in the store")
    listing.set_defaults(handler=run_list)

    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
