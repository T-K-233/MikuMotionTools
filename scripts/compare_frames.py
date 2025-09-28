"""
Script to compare the frames of two motions, used to adjust mapping table.

An example usage is as follows:
```bash
blender ./blender-projects/G1_USD.blend --python ./scripts/examples/export_g1_reset_pose.py
blender ./blender-projects/G1_Zamuza.blend --python ./scripts/examples/export_mmd_reset_pose.py
uv run ./scripts/compare_frames.py
```
"""

import mujoco
import mujoco.viewer
import numpy as np

from mikumotion.mujoco_utils import create_empty_scene, add_body_frames
from mikumotion.motion_sequence import MotionSequence, translate_motion


def main():
    """
    Main function to create and display the empty scene with coordinate frames.
    """
    xml = create_empty_scene(show_world_frame=True)

    source_armature = MotionSequence.load("./data/motions/mmd_reset_pose.npz")
    target_armature = MotionSequence.load("./data/motions/g1_reset_pose.npz")
    # move G1 armature to stand on the ground
    target_armature = translate_motion(target_armature, np.array([0.0, 0.0, 0.78]))

    # Create the XML content
    xml = add_body_frames(xml, source_armature, prefix="source_", center_color=(0.0, 1.0, 1.0))
    xml = add_body_frames(xml, target_armature, prefix="target_", center_color=(1.0, 0.0, 1.0))

    # Load the model from XML string
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

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


if __name__ == "__main__":
    main()
