"""

# Usage

1. Export the motion from Blender:

Ubuntu:
```bash
blender ./blender-projects/Zamuza.blend --python ./scripts/examples/export_zamuza.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\Zamuza.blend --python scripts\examples\export_zamuza.py
```

2. After getting the source motion, run the retargeting script:

```bash
uv run ./scripts/run_retargeting.py --motion ./data/motions/zamuza_0_1632.npz --mapping MMD_YYB_TO_G1_CFG --real-time
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
from mikumotion.blender import (
    build_body_motion_data,
    set_armature_to_pose,
    set_armature_to_rest,
    set_scene_animation_range,
)

C = bpy.context
D = bpy.data
O = bpy.ops


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

armature = D.objects.get("Armature")


# we need data from these bones to match SMPL keypoints
bone_names = [
    "pelvis",           # 0
    "left_upper_leg",   # 1
    "right_upper_leg",  # 2
    "torso",            # 3
    "left_lower_leg",   # 4
    "right_lower_leg",  # 5
    "torso",            # 6
    "left_foot",        # 7
    "right_foot",       # 8
    "head",             # 9
    "left_upper_arm",   # 10
    "right_upper_arm",  # 11
    "left_lower_arm",   # 12
    "right_lower_arm",  # 13
    "left_hand",        # 14
    "right_hand",       # 15
]


# # blender is +Y forward, we need to rotate to +X forward
# rotate_z_angle = 0


def export_reset_pose():
    set_scene_animation_range(0, 1)
    set_armature_to_rest(armature)

    motion = build_body_motion_data(armature, bone_names)

    # blender is +Y forward, we need to rotate to +X forward
    # motion = rotate_motion(motion, rotate_z_angle)

    save_path = "./data/motions/zamuza_reset.npz"
    motion.save(save_path)
    print(f"Reset pose motion saved to {save_path}")


def export_motion():
    motion_section = (0, 400)
    set_scene_animation_range(motion_section[0], motion_section[1])

    set_armature_to_pose(armature)

    motion = build_body_motion_data(armature, bone_names)
    # motion = rotate_motion(motion, rotate_z_angle)

    save_path = f"./data/motions/zamuza_{motion_section[0]}_{motion_section[1]}.npz"
    motion.save(save_path)
    print(f"Result motion saved to {save_path}")


export_reset_pose()
export_motion()

exit()
