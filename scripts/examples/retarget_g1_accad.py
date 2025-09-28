"""
Ubuntu:
```bash
blender ./blender-projects/ACCAD_Female1_WalkTurnChangeDirection.blend --python ./scripts/examples/extract_accad.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\ACCAD_Female1_B03_Walk1.blend --python scripts\examples\extract_accad.py
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
from mikumotion.presets import GenericKeypointMapping
from mikumotion.blender import (
    set_scene_animation_range,
    build_body_motion_data,
    set_armature_to_rest,
    set_armature_to_pose,
)
from mikumotion.math import quat_mul, quat_from_euler_xyz


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

motion_section = (1, 652)

set_scene_animation_range(motion_section[0], motion_section[1])

source_armature = D.objects.get("armature")

# set_armature_to_rest(source_armature)
set_armature_to_pose(source_armature)

scaling_ratio = 1.0

motion = build_body_motion_data(source_armature, mapping=GenericKeypointMapping.accad, scaling_ratio=scaling_ratio)

save_path = f"./data/motions/accad_{motion_section[0]}_{motion_section[1]}_body_only.npz"
motion.save(save_path)
print(f"Results saved to {save_path}")

exit()
