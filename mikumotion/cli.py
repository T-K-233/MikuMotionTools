"""
The ``mikumotion`` command line: one entry point for working with motion files.

    mikumotion import <mcap> <mjcf> <urdf>            a robot log becomes a motion
    mikumotion view <name>                            watch a motion in the Rerun viewer
    mikumotion retarget <name> <mjcf> <urdf> <map>    solve a robot's joints for a motion
    mikumotion list                                   show the motions in the store
    mikumotion push <name> ...                        send motions to the remote dataset
    mikumotion pull <name> ...                        fetch motions from the remote dataset

You address a motion by name, not by path. The layout under ``--root`` decides which stage
lands in which layer (see :mod:`mikumotion.motion_sequence`).

``push`` and ``pull`` name motions the same way, and with no name they move the whole store.
``MIKUMOTION_REMOTE`` says where (see :mod:`mikumotion.motion_remote`), so an environment is
configured once instead of on every command.
"""

import argparse
import subprocess
import sys
from pathlib import Path

from .motion_remote import motion_layers, motion_paths, uri_to_remote
from .motion_sequence import REFERENCE, MotionStore


def run_import(args):
    """A logged motion is both stages at once: the poses are the export, the joints the solve.

    This command solves nothing, because the log already carries the joint angles a robot
    reached. Forward kinematics turns those angles into the poses, and both layers come from
    the one sequence: the export in ``reference/``, and the same poses as the robot layer's
    goal, because a log reached its own goal.
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
    """Show the store, or the remote dataset with ``--remote``.

    Both listings group the same way, by the layers a file name appears in, so a robot layer
    on its own still counts as a motion — which is the point of the layer being
    self-contained. The remote one stops there, because reading a frame count would mean
    downloading the file this command exists to avoid downloading.
    """
    if args.remote:
        for name, layers in motion_layers(uri_to_remote().paths()).items():
            print(f"{name:30s} {', '.join(layers)}")
        return

    store = MotionStore(args.root)
    for name, layers in motion_layers(store.paths()).items():
        robots = [layer for layer in layers if layer != REFERENCE]
        motion = (store.read_reference_motion(name) if REFERENCE in layers
                  else store.read_robot_motion(name, robots[0]))
        print(f"{name:30s} {motion!r}  solved for: {', '.join(robots) or '-'}")


def run_push(args):
    """Upload the motions named, or the whole store, then print what moved."""
    store = MotionStore(args.root)
    remote = uri_to_remote()
    paths = motion_paths(store.paths(), args.name)
    remote.push(paths, store.root)
    for path in paths:
        print(path)


def run_pull(args):
    """Download the motions named, or the whole dataset, then print what moved."""
    store = MotionStore(args.root)
    remote = uri_to_remote()
    paths = motion_paths(remote.paths(), args.name)
    remote.pull(paths, store.root)
    for path in paths:
        print(path)


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
    listing.add_argument("--remote", action="store_true", help="list the remote dataset instead")
    listing.set_defaults(handler=run_list)

    push = commands.add_parser("push", help="send motions to the remote dataset")
    push.add_argument("name", nargs="*", help="motion names; none means the whole store")
    push.set_defaults(handler=run_push)

    pull = commands.add_parser("pull", help="fetch motions from the remote dataset")
    pull.add_argument("name", nargs="*", help="motion names; none means the whole dataset")
    pull.set_defaults(handler=run_pull)

    args = parser.parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
