"""
Ubuntu:
```bash
blender ./blender-projects/G1_Zamuza.blend --python ./scripts/examples/retarget_g1_zamuza.py
uv run ./scripts/compute_dof_ik.py --motion ./data/motions/g1_zamuza_0_1632_body_only.npz --mapping G1_MMD_YYB_MAPPING
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\G1_Zamuza.blend --python scripts\examples\retarget_g1_zamuza.py
uv run ./scripts/compute_dof_ik.py --motion ./data/motions/g1_zamuza_0_1632.npz --mapping G1_MMD_YYB_MAPPING
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
from mikumotion.presets import G1_MMD_YYB_MAPPING
from mikumotion.blender import (
    set_scene_animation_range,
    build_body_motion_data,
    set_armature_to_pose,
)
from mikumotion.motion_sequence import rotate_motion


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

motion_section = (0, 1632)
# motion_section = (0, 600)

set_scene_animation_range(motion_section[0], motion_section[1])

source_armature = D.objects.get("YYB式初音ミクv1.02_arm")

set_armature_to_pose(source_armature)

scaling_ratio = 0.85
motion = build_body_motion_data(source_armature, mapping=G1_MMD_YYB_MAPPING, scaling_ratio=scaling_ratio)

# blender is +Y forward, we need to rotate to +X forward
motion = rotate_motion(motion, np.pi / 2)

save_path = f"./data/motions/g1_zamuza_{motion_section[0]}_{motion_section[1]}_body_only.npz"
motion.save(save_path)
print(f"Results saved to {save_path}")

exit()
