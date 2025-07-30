"""
Ubuntu:
```bash
blender ./blender-projects/G1_Zamuza.blend --python ./scripts/examples/extract_g1_zamuza.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\G1_Zamuza.blend --python scripts\examples\extract_g1_zamuza.py
```
"""

import sys
import os
import bpy
import importlib

""" Include the pose library """

blend_path = os.path.dirname(bpy.data.filepath)
mikumotion_path = os.getcwd()

if blend_path not in sys.path:
   sys.path.append(blend_path)

if mikumotion_path not in sys.path:
   sys.path.append(mikumotion_path)

from mikumotion import blender

importlib.reload(blender)

C = bpy.context
D = bpy.data
O = bpy.ops


""" Everything else follows """

import numpy as np
from mikumotion.presets import UnitreeG1Mapping
from mikumotion.blender import (
    set_scene_animation_range,
    build_body_motion_data,
    # set_armature_to_rest,
    set_armature_to_pose,
)
from mikumotion.math import quat_mul, quat_from_euler_xyz


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

miku_to_g1_scaling = 0.83

motion_section = (0, 1632)

set_scene_animation_range(motion_section[0], motion_section[1])

source_armature = D.objects.get("YYB式初音ミクv1.02_arm")

# set_armature_to_rest(source_armature)
set_armature_to_pose(source_armature)

motion = build_body_motion_data(source_armature, mapping=UnitreeG1Mapping.mmd_yyb, scaling_ratio=miku_to_g1_scaling)


# Post-process the motion data to align the frames

pelvis_idx = motion.get_body_index(["pelvis"])[0]
left_hand_idx = motion.get_body_index(["left_rubber_hand"])[0]
right_hand_idx = motion.get_body_index(["right_rubber_hand"])[0]
left_foot_idx = motion.get_body_index(["left_ankle_roll_link"])[0]
right_foot_idx = motion.get_body_index(["right_ankle_roll_link"])[0]

# pelvis (root)
motion._body_rotations[:, pelvis_idx, :] = quat_mul(motion._body_rotations[:, pelvis_idx, :], quat_from_euler_xyz(np.deg2rad(90), 0, np.deg2rad(-90)))
motion._body_rotations[:, pelvis_idx, :] = quat_mul(motion._body_rotations[:, pelvis_idx, :], quat_from_euler_xyz(0, np.deg2rad(-16), 0))
# left hand
motion._body_rotations[:, left_hand_idx, :] = quat_mul(motion._body_rotations[:, left_hand_idx, :], quat_from_euler_xyz(np.deg2rad(90), np.deg2rad(90), 0))
# right hand
motion._body_rotations[:, right_hand_idx, :] = quat_mul(motion._body_rotations[:, right_hand_idx, :], quat_from_euler_xyz(np.deg2rad(90), np.deg2rad(90), 0))
# left feet
motion._body_rotations[:, left_foot_idx, :] = quat_mul(motion._body_rotations[:, left_foot_idx, :], quat_from_euler_xyz(np.deg2rad(90), 0, np.deg2rad(-90)))
motion._body_rotations[:, left_foot_idx, :] = quat_mul(motion._body_rotations[:, left_foot_idx, :], quat_from_euler_xyz(0, np.deg2rad(65), 0))
# right feet
motion._body_rotations[:, right_foot_idx, :] = quat_mul(motion._body_rotations[:, right_foot_idx, :], quat_from_euler_xyz(np.deg2rad(90), 0, np.deg2rad(-90)))
motion._body_rotations[:, right_foot_idx, :] = quat_mul(motion._body_rotations[:, right_foot_idx, :], quat_from_euler_xyz(0, np.deg2rad(65), 0))

save_path = "./data/motions/g1_zamuza_0_1632_body_only.npz"
motion.save(save_path)
print(f"Results saved to {save_path}")
