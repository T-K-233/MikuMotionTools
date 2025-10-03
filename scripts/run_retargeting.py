import argparse

from mikumotion.motion_retargeting import MotionRetargeting
from mikumotion.presets import PRESETS


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion", type=str, help="Source motion file")
    parser.add_argument("--mapping", type=str, help="Mapping config")
    parser.add_argument("--real-time", action="store_true", default=False, help="Visualize in realtime")
    args = parser.parse_args()

    motion_file = args.motion

    mapping_table = PRESETS.get(args.mapping)

    assert mapping_table, f"Mapping config {args.mapping} not found"

    match args.mapping:
        case "ACCAD_TO_G1_CFG":
            robot_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5.xml"
        case "ACTORCORE_TO_G1_CFG":
            robot_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5.xml"
        case "MMD_YYB_TO_G1_CFG":
            robot_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5.xml"
        case _:
            raise ValueError(f"Does not support mapping config: {args.mapping}")

    retargeting = MotionRetargeting(motion_file, robot_xml, mapping_table)
    retargeting.run(realtime=args.real_time)
