"""

# Usage

1. Export the motion from Blender:

Ubuntu:
```bash
blender ./blender-projects/ACCAD-Female1-WalkTurnChangeDirection.blend --python ./scripts/examples/export_accad.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\ACCAD-Female1-WalkTurnChangeDirection.blend --python scripts\examples\export_accad.py
```

2. After getting the source motion, run the retargeting script:

```bash
uv run ./scripts/run_retargeting.py --motion ./data/motions/accad_female1_walk_turn_change_direction_0_293.npz --mapping ACCAD_TO_G1_CFG --real-time
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
from mikumotion.motion_sequence import rotate_motion, translate_motion

C = bpy.context
D = bpy.data
O = bpy.ops


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

armature = D.objects.get("armature")

# we need data from these bones to match SMPL keypoints
bone_names = [
   "Hips",            # 0
   "LeftUpLeg",       # 1
   "RightUpLeg",      # 2
   "ToSpine",         # 3
   "LeftLeg",         # 4
   "RightLeg",        # 5
   "Spine",           # 6
   "LeftFoot",        # 7
   "RightFoot",       # 8
   "Spine1",          # 9
   "LeftToeBase",     # 10
   "RightToeBase",    # 11
   "Neck",            # 12
   "LeftShoulder",    # 13
   "RightShoulder",   # 14
   "Head",            # 15
   "LeftArm",         # 16
   "RightArm",        # 17
#    "LeftForearm",     # 18  # somehow these two bones are buggy
#    "RightForearm",    # 19
   "LeftHand",        # 20, 22
   "RightHand",       # 21, 23
]

scaling_ratio = 0.8

# motion is -Y forward, we need to rotate to +X forward
rotate_z_angle = -np.pi / 2


def export_reset_pose():
    set_scene_animation_range(0, 1)
    set_armature_to_rest(armature)

    motion = build_body_motion_data(armature, bone_names, scaling_ratio=scaling_ratio)
    motion = rotate_motion(motion, rotate_z_angle)

    save_path = "./data/motions/accad_reset.npz"
    motion.save(save_path)
    print(f"Reset pose motion saved to {save_path}")


def export_motion():
    motion_section = (0, 652)
    set_scene_animation_range(motion_section[0], motion_section[1])

    set_armature_to_pose(armature)

    motion = build_body_motion_data(armature, bone_names, scaling_ratio=scaling_ratio)
    motion = rotate_motion(motion, rotate_z_angle)

    save_path = f"./data/motions/accad_female1_walk_turn_change_direction_{motion_section[0]}_{motion_section[1]}.npz"
    motion.save(save_path)
    print(f"Result motion saved to {save_path}")


export_reset_pose()
export_motion()

exit()
