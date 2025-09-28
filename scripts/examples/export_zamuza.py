"""
Ubuntu:
```bash
blender ./blender-projects/Zamuza.blend --python ./scripts/examples/export_zamuza.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\Zamuza.blend --python scripts\examples\export_zamuza.py
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

armature = D.objects.get("YYB式初音ミクv1.02_arm")

# we need data from these bones to match SMPL keypoints
bone_names = [
   "下半身",      # 0
   "足.L",        # 1
   "足.R",        # 2
   "上半身",      # 3
   "ひざ.L",      # 4
   "ひざ.R",      # 5
   "上半身2",     # 6, 9
   "足首.L",      # 7
   "足首.R",      # 8
   "足先EX.L",    # 10
   "足先EX.R",    # 11
   "首",          # 12
   "肩.L",        # 13
   "肩.R",        # 14
   "頭",          # 15
   "腕.L",        # 16
   "腕.R",        # 17
   "ひじ.L",      # 18
   "ひじ.R",      # 19
   "手首.L",      # 20, 22
   "手首.R",      # 21, 23
]

scaling_ratio = 0.85

# blender is +Y forward, we need to rotate to +X forward
rotate_z_angle = np.pi / 2


def export_reset_pose():
    set_scene_animation_range(0, 1)
    set_armature_to_rest(armature)

    motion = build_body_motion_data(armature, bone_names, scaling_ratio=scaling_ratio)

    # blender is +Y forward, we need to rotate to +X forward
    motion = rotate_motion(motion, rotate_z_angle)

    save_path = "./data/motions/zamuza_reset.npz"
    motion.save(save_path)
    print(f"Reset pose motion saved to {save_path}")


def export_motion():
    motion_section = (0, 1632)
    set_scene_animation_range(motion_section[0], motion_section[1])

    set_armature_to_pose(armature)

    motion = build_body_motion_data(armature, bone_names, scaling_ratio=scaling_ratio)
    motion = rotate_motion(motion, rotate_z_angle)

    save_path = f"./data/motions/zamuza_{motion_section[0]}_{motion_section[1]}.npz"
    motion.save(save_path)
    print(f"Result motion saved to {save_path}")


export_reset_pose()
export_motion()

exit()
