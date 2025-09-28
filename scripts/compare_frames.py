"""
Script to compare the frames of two motions, used to adjust mapping table.

An example usage is as follows:
```bash
blender ./blender-projects/G1_USD.blend --python ./scripts/examples/export_g1_reset_pose.py
blender ./blender-projects/G1_Zamuza.blend --python ./scripts/examples/export_mmd_reset_pose.py
uv run ./scripts/compare_frames.py
```
"""

import argparse

import mujoco
import mujoco.viewer
import numpy as np

from mikumotion.math import quat_mul
from mikumotion.mujoco_utils import create_empty_scene, add_body_frames
from mikumotion.motion_sequence import MotionSequence
from mikumotion.presets import PRESETS


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=str, required=True, help="Source motion file")
    parser.add_argument("--target", type=str, required=True, help="Target motion file")
    parser.add_argument("--mapping", type=str, help="Mapping config")
    args = parser.parse_args()

    xml = create_empty_scene(show_world_frame=True)

    source_armature = MotionSequence.load(args.source)
    target_armature = MotionSequence.load(args.target)

    # Create the XML content
    xml = add_body_frames(xml, source_armature.body_names, prefix="source_", center_color=(0.0, 1.0, 1.0))
    xml = add_body_frames(xml, target_armature.body_names, prefix="target_", center_color=(1.0, 0.0, 1.0))

    # Load the model from XML string
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    if args.mapping:
        mapping = PRESETS.get(args.mapping)
    else:
        mapping = None
        print("No mapping config provided, showing the default transformation.")

    # move the source frame mocap body
    for body_name in source_armature.body_names:
        position = source_armature.body_positions[0, source_armature.get_body_indices([body_name])[0]]
        orientation = source_armature.body_rotations[0, source_armature.get_body_indices([body_name])[0]]
        mocap_id = model.body(f"source_{body_name}_frame").mocapid[0]
        if mapping:
            for entry in mapping.values():
                if entry["source"] == body_name:
                    break
            if entry.get("offset") and entry["offset"].get("position"):
                position += np.array(entry["offset"]["position"])

            if entry.get("offset") and entry["offset"].get("orientation"):
                orientation = quat_mul(orientation, np.array(entry["offset"]["orientation"]))

        data.mocap_pos[mocap_id] = position
        data.mocap_quat[mocap_id] = orientation

    # move the target frame mocap body
    for body_name in target_armature.body_names:
        position = target_armature.body_positions[0, target_armature.get_body_indices([body_name])[0]]
        orientation = target_armature.body_rotations[0, target_armature.get_body_indices([body_name])[0]]
        mocap_id = model.body(f"target_{body_name}_frame").mocapid[0]
        data.mocap_pos[mocap_id] = position
        data.mocap_quat[mocap_id] = orientation

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("MuJoCo scene with robot coordinate frames loaded.")
        print("Red axis: X-axis")
        print("Green axis: Y-axis")
        print("Blue axis: Z-axis")
        print("Gray sphere: Frame origin")
        print("Close the viewer window to exit.")

        # Keep the viewer running
        while viewer.is_running():
            # Step the simulation (even though it's static)
            mujoco.mj_step(model, data)
            viewer.sync()
