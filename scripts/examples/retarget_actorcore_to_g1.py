"""
Ubuntu:
```bash
blender ./blender-projects/ActorCore-Walk-Relaxed.blend --python ./scripts/examples/retarget_g1_actorcore.py
uv run ./scripts/compute_dof_ik.py --motion ./data/motions/g1_actorcore_0_1632_body_only.npz --mapping G1_ACTORCORE_MAPPING
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\ActorCore-Walk-Relaxed.blend --python scripts\examples\retarget_g1_actorcore.py
uv run ./scripts/compute_dof_ik.py --motion ./data/motions/g1_actorcore_0_1632_body_only.npz --mapping G1_ACTORCORE_MAPPING
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
from mikumotion.presets import G1_ACTORCORE_MAPPING
from mikumotion.blender import (
    build_body_motion_data,
    set_armature_to_pose,
)
from mikumotion.motion_sequence import rotate_motion


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

source_armature = D.objects.get("Armature")

set_armature_to_pose(source_armature)


# convert from cm to m
scaling_ratio = 0.01 * 0.8

motion = build_body_motion_data(source_armature, mapping=G1_ACTORCORE_MAPPING, scaling_ratio=scaling_ratio)

# blender source motion is +Y forward, we need to rotate to +X forward
motion = rotate_motion(motion, np.pi / 2)

save_path = "./data/motions/g1_actorcore_walk_relaxed_keypoint.npz"
motion.save(save_path)
print(f"Results saved to {save_path}")

exit()
