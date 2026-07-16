"""
Convert a policy/tracking motion log (``.mcap``) into a ``MotionSequence`` (``.npz``)
of per-link world poses, using MuJoCo forward kinematics.

Example:

    uv run scripts/policy_log_to_motion.py \
        --mcap "D:/Downloads/Lite-Pro-Tracking_....mcap" \
        --mjcf data/robots/berkeley_humanoids/lite_pro/mjcf/lite_pro.xml \
        --out  data/motions/lite_pro_tracking.npz
"""

import argparse
import os

from mikumotion.forward_kinematics import motion_from_policy_log


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=str, required=True, help="Input .mcap motion log")
    parser.add_argument("--mjcf", type=str, required=True, help="Robot MJCF for forward kinematics")
    parser.add_argument("--out", type=str, required=True, help="Output .npz MotionSequence")
    parser.add_argument("--base-body", type=str, default="pelvis", help="Floating-base body name")
    args = parser.parse_args()

    assert os.path.isfile(args.mcap), f"mcap not found: {args.mcap}"
    assert os.path.isfile(args.mjcf), f"mjcf not found: {args.mjcf}"

    motion = motion_from_policy_log(args.mcap, args.mjcf, base_body=args.base_body)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    motion.save(args.out)
    print(f"Saved MotionSequence -> {args.out}")
    print(f"  frames={motion.num_frames}  bodies={motion.num_bodies}  joints={motion.num_joints}  fps={motion.fps}")


if __name__ == "__main__":
    main()
