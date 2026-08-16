"""
The ``mikumotion`` command line: one entry point for working with motion files.

    mikumotion import <mcap> <mjcf>    a robot log becomes a motion (robot -> animation)
    mikumotion view <name>             watch a motion in the Rerun viewer
    mikumotion retarget <name> <mjcf>  solve a robot's joints for a motion (animation -> robot)
    mikumotion list                    show the motions in the store

Motions are addressed by name, not by path: the layout under ``--root`` (see
:mod:`mikumotion.motion_sequence`) decides where each layer lives.
"""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

from .motion_sequence import MotionStore


def compose_for_viewing(store, name, destination):
    """
    Merge a motion's layers into one recording the viewer can show.

    Rerun composes data only within a single recording, and the robot layer is shared by
    every motion, so it is rewritten onto this motion's recording id before merging.
    """
    paths = store.layer_paths(name)
    rerun = str(Path(sys.executable).parent / "rerun")
    routed = destination.parent / "robot_routed.rrd"

    robot_layers = [path for path in paths if path.suffix == ".rrd" and path.parent.name == "robot"]
    motion_layers = [path for path in paths if path.suffix == ".rrd" and path.parent.name != "robot"]

    for robot_layer in robot_layers:
        subprocess.run([rerun, "rrd", "route", "--recording-id", name,
                        "-o", str(routed), str(robot_layer)], check=True)
        motion_layers.append(routed)

    subprocess.run([rerun, "rrd", "merge", "-o", str(destination)]
                   + [str(path) for path in motion_layers], check=True)
    return [path for path in paths if path.suffix == ".rbl"]


def run_import(args):
    from .forward_kinematics import robot_log_to_motion

    store = MotionStore(args.root)
    name = args.name or Path(args.mcap).stem
    motion = robot_log_to_motion(args.mcap, args.mjcf)
    store.write_motion(name, motion)
    store.write_preview(name, motion, args.urdf)
    print(f"{name}: {motion!r}")
    for path in store.layer_paths(name):
        print(f"  {path}")


def run_view(args):
    store = MotionStore(args.root)
    with tempfile.TemporaryDirectory() as scratch:
        composed = Path(scratch) / f"{args.name}.rrd"
        blueprints = compose_for_viewing(store, args.name, composed)
        rerun = str(Path(sys.executable).parent / "rerun")
        subprocess.run([rerun, str(composed)] + [str(path) for path in blueprints], check=True)


def run_retarget(args):
    from .motion_retargeting import MotionRetargetingIK
    from .presets import RETARGET_MAPS

    store = MotionStore(args.root)
    mapping = RETARGET_MAPS[args.mapping]
    MotionRetargetingIK(args.name, args.mjcf, store, mapping).run(args.view)


def run_list(args):
    store = MotionStore(args.root)
    for path in sorted((store.root / "base").glob("*.rrd")):
        motion = store.read_motion(path.stem)
        print(f"{path.stem:40s} {motion!r}")


def main():
    parser = argparse.ArgumentParser(prog="mikumotion", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="data/motions", help="motion store directory")
    commands = parser.add_subparsers(dest="command", required=True)

    importer = commands.add_parser("import", help="convert a robot .mcap log into a motion")
    importer.add_argument("mcap", help="ROS2 .mcap log")
    importer.add_argument("mjcf", help="robot MJCF, used for forward kinematics")
    importer.add_argument("urdf", help="robot URDF, used to build the preview layer")
    importer.add_argument("--name", help="motion name (defaults to the .mcap file name)")
    importer.set_defaults(handler=run_import)

    viewer = commands.add_parser("view", help="open a motion in the Rerun viewer")
    viewer.add_argument("name")
    viewer.set_defaults(handler=run_view)

    retarget = commands.add_parser("retarget", help="solve a robot's joints for a motion")
    retarget.add_argument("name")
    retarget.add_argument("mjcf", help="target robot MJCF")
    retarget.add_argument("mapping", help="body map from mikumotion.presets.RETARGET_MAPS")
    retarget.add_argument("--view", action="store_true", help="watch the solve in a MuJoCo window")
    retarget.set_defaults(handler=run_retarget)

    listing = commands.add_parser("list", help="show the motions in the store")
    listing.set_defaults(handler=run_list)

    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
