import argparse

from mikumotion.motion_retargeting import MotionRetargeting
from mikumotion.presets import G1_MMD_YYB_MAPPING


parser = argparse.ArgumentParser()
parser.add_argument("--motion", type=str, help="Source motion file")
parser.add_argument("--mapping", type=str, help="Mapping table")
parser.add_argument("--realtime", action="store_true", default=False, help="Visualize in realtime")
args = parser.parse_args()


if __name__ == "__main__":
    motion_file = args.motion

    match args.mapping:
        case "G1_MMD_YYB_MAPPING":
            robot_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5_mocap.xml"
            mapping_table = G1_MMD_YYB_MAPPING
        case _:
            raise ValueError(f"Unknown mapping table: {args.mapping}")

    retargeting = MotionRetargeting(motion_file, robot_xml, mapping_table)
    retargeting.run(realtime=args.realtime)
