"""
Script to view a motion file.
"""

import argparse

from mikumotion.motion_sequence import MotionSequence


parser = argparse.ArgumentParser()
parser.add_argument("--motion", type=str, required=True, help="Motion file")

args, _ = parser.parse_known_args()

motion = MotionSequence.load(args.motion)

print("Dof names:", motion.dof_names)
print("Body names:", motion.body_names)
