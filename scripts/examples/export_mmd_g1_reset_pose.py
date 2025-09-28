"""
Ubuntu:
```bash
blender ./blender-projects/Zamuza.blend --python ./scripts/examples/export_mmd_g1_reset_pose.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\Zamuza.blend --python scripts\examples\export_mmd_g1_reset_pose.py
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
from mikumotion.presets import MMD_YYB_TO_G1_CFG
from mikumotion.blender import set_armature_to_rest, set_scene_animation_range, build_body_motion_data
from mikumotion.motion_sequence import rotate_motion

C = bpy.context
D = bpy.data
O = bpy.ops

armature = D.objects.get("YYB式初音ミクv1.02_arm")

set_armature_to_rest(armature)

set_scene_animation_range(0, 1)

motion = build_body_motion_data(armature, config=MMD_YYB_TO_G1_CFG)

# blender is +Y forward, we need to rotate to +X forward
motion = rotate_motion(motion, np.pi / 2)

save_path = "./data/motions/mmd_g1_reset_pose.npz"
motion.save(save_path)
print(f"Results saved to {save_path}")
exit()
