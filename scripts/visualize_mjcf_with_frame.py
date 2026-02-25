"""
Script to load an MJCF file and visualize coordinate frames of all links/bodies.

Usage:
```bash
uv run scripts/visualize_mjcf_with_frame.py --mjcf path/to/robot.xml
```
"""

import argparse

import numpy as np
import mujoco
import mujoco.viewer

from mikumotion.mujoco_utils import add_body_frames


def update_frames(model, data, body_names: list[str], prefix: str = "viz_"):
    """Recompute FK and reposition mocap frame bodies to match current body poses."""
    mujoco.mj_forward(model, data)
    for body_name in body_names:
        body_id = model.body(body_name).id
        mocap_id = model.body(f"{prefix}{body_name}_frame").mocapid[0]
        data.mocap_pos[mocap_id] = data.xpos[body_id]
        data.mocap_quat[mocap_id] = data.xquat[body_id]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize MJCF model with coordinate frames on all links.")
    parser.add_argument("--mjcf", type=str, required=True, help="Path to the MJCF XML file")
    args = parser.parse_args()

    xml = open(args.mjcf).read()

    # first pass: load by path so MuJoCo can resolve relative mesh references
    tmp_model = mujoco.MjModel.from_xml_path(args.mjcf)
    body_names = []

    body_names = [
        "pelvis",
        "torso",
        "head",
        "left_hand",
        "right_hand",
        "left_foot",
        "right_foot",

        "left_shoulder_roll",
        "right_shoulder_roll",
        "left_elbow_pitch",
        "right_elbow_pitch",
        "left_hip_pitch",
        "right_hip_pitch",
        "left_knee_pitch",
        "right_knee_pitch",
    ]

    if not body_names:
        for i in range(1, tmp_model.nbody):
            body_names.append(tmp_model.body(i).name)

    print(f"Found {len(body_names)} bodies: {body_names}")

    # xml = add_body_frames(xml, ["world"], prefix="", center_color=(1.0, 0.0, 0.0))
    xml = add_body_frames(xml, body_names, prefix="viz_", center_color=(0.8, 0.8, 0.8))

    # write augmented XML next to the original so relative mesh paths still resolve
    frames_xml_path = args.mjcf.replace(".xml", "_frames.xml")
    open(frames_xml_path, "w").write(xml)

    model = mujoco.MjModel.from_xml_path(frames_xml_path)
    data = mujoco.MjData(model)

    joint_positions = {
        "left_elbow_pitch": -0.2,
        "right_elbow_pitch": 0.2,
        "left_knee_pitch": 0.2,
        "right_knee_pitch": 0.2,
    }
    joint_positions = np.zeros(tmp_model.nu)
    # joint_positions[3] = -0.2  # left knee
    # joint_positions[10] = 0.2  # right knee
    # joint_positions[20] = -0.2  # left elbow
    # joint_positions[47] = 0.2  # right elbow

    data.qpos[7:] = joint_positions
    mujoco.mj_forward(model, data)

    update_frames(model, data, body_names)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Red axis:   X-axis")
        print("Green axis: Y-axis")
        print("Blue axis:  Z-axis")
        print("Close the viewer window to exit.")

        while viewer.is_running():
            update_frames(model, data, body_names)
            viewer.sync()

            mujoco.mj_forward(model, data)
