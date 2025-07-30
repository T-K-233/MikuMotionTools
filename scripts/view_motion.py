"""
Script to view a motion file.
"""

import argparse

import matplotlib

from mikumotion.motion_sequence import MotionSequence
from mikumotion.viewers import MatplotViewer

parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str, required=True, help="Motion file")
parser.add_argument(
    "--render-scene",
    action="store_true",
    default=False,
    help=(
        "Whether the scene (space occupied by the skeleton during movement) is rendered instead of a reduced view"
        " of the skeleton."
    ),
)
parser.add_argument("--matplotlib-backend", type=str, default="TkAgg", help="Matplotlib interactive backend")
args, _ = parser.parse_known_args()

# https://matplotlib.org/stable/users/explain/figure/backends.html#interactive-backends
matplotlib.use(args.matplotlib_backend)

motion = MotionSequence.load(args.file)

g1_frames = ["pelvis", "left_rubber_hand", "right_rubber_hand", "left_ankle_roll_link", "right_ankle_roll_link"]

viewer = MatplotViewer(motion, render_scene=args.render_scene, show_frames=g1_frames)
viewer.show()
