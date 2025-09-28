import argparse

from mikumotion.motion_retargeting import MotionRetargeting
from mikumotion.presets import MMD_YYB_TO_G1_CFG


parser = argparse.ArgumentParser()
parser.add_argument("--motion", type=str, help="Source motion file")
parser.add_argument("--mapping", type=str, help="Mapping config")
parser.add_argument("--real-time", action="store_true", default=False, help="Visualize in realtime")
args = parser.parse_args()


if __name__ == "__main__":
    motion_file = args.motion

    match args.mapping:
        case "MMD_YYB_TO_G1_CFG":
            robot_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5.xml"
            mapping_table = MMD_YYB_TO_G1_CFG
        case _:
            raise ValueError(f"Unknown mapping config: {args.mapping}")

    retargeting = MotionRetargeting(motion_file, robot_xml, mapping_table)
    retargeting.run(realtime=args.real_time)
