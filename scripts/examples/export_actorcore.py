"""

# Usage

1. Export the motion from Blender:

Ubuntu:
```bash
blender ./blender-projects/ActorCore-Walk-Relaxed.blend --python ./scripts/examples/export_actorcore.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\ActorCore-Walk-Relaxed.blend --python scripts\examples\export_actorcore.py
```

2. After getting the source motion, run the retargeting script:

```bash
uv run ./scripts/run_retargeting.py --motion ./data/motions/actorcore_walk_relaxed_0_293.npz --mapping ACTORCORE_TO_G1_CFG --real-time
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
from mikumotion.motion_sequence import rotate_motion

C = bpy.context
D = bpy.data
O = bpy.ops


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

armature = D.objects.get("Armature")

# we need data from these bones to match SMPL keypoints
bone_names = [
   "CC_Base_Pelvis",       # 0
   "CC_Base_L_Thigh",      # 1
   "CC_Base_R_Thigh",      # 2
   "CC_Base_Spine01",      # 3
   "CC_Base_L_Calf",       # 4
   "CC_Base_R_Calf",       # 5
   "CC_Base_Spine02",      # 6, 9
   "CC_Base_L_Foot",       # 7
   "CC_Base_R_Foot",       # 8
   "CC_Base_L_ToeBaseShareBone",    # 10
   "CC_Base_R_ToeBaseShareBone",    # 11
   "CC_Base_NeckTwist02",           # 12
   "CC_Base_L_Clavicle",   # 13
   "CC_Base_R_Clavicle",   # 14
   "CC_Base_Head",         # 15
   "CC_Base_L_Upperarm",   # 16
   "CC_Base_R_Upperarm",   # 17
   "CC_Base_L_Forearm",    # 18
   "CC_Base_R_Forearm",    # 19
   "CC_Base_L_Hand",       # 20, 22
   "CC_Base_R_Hand",       # 21, 23
]

scaling_ratio = 0.7 * 0.01

# blender is +Y forward, we need to rotate to +X forward
rotate_z_angle = np.pi / 2


def export_reset_pose():
    set_scene_animation_range(0, 1)
    set_armature_to_rest(armature)

    motion = build_body_motion_data(armature, bone_names, scaling_ratio=scaling_ratio)

    # blender is +Y forward, we need to rotate to +X forward
    motion = rotate_motion(motion, rotate_z_angle)

    save_path = "./data/motions/actorcore_reset.npz"
    motion.save(save_path)
    print(f"Reset pose motion saved to {save_path}")


def export_motion():
    motion_section = (0, 293)
    set_scene_animation_range(motion_section[0], motion_section[1])

    set_armature_to_pose(armature)

    motion = build_body_motion_data(armature, bone_names, scaling_ratio=scaling_ratio)
    motion = rotate_motion(motion, rotate_z_angle)

    save_path = f"./data/motions/actorcore_walk_relaxed_{motion_section[0]}_{motion_section[1]}.npz"
    motion.save(save_path)
    print(f"Result motion saved to {save_path}")


export_reset_pose()
export_motion()

exit()
