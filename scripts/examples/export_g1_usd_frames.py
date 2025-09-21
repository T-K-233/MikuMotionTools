"""
Ubuntu:
```bash
blender ./blender-projects/G1_USD.blend --python ./scripts/examples/export_g1_usd_frames.py
```

Windows:
```powershell
D:\Documents\Blender\blender.exe .\blender-projects\G1_USD.blend --python scripts\examples\export_g1_usd_frames.py
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

from mikumotion.presets import G1_MMD_YYB_MAPPING
from mikumotion.motion_sequence import MotionSequence


C = bpy.context
D = bpy.data
O = bpy.ops

usd_object = D.objects.get("g1_29dof_mode_6")

g1_frames = []
for value in G1_MMD_YYB_MAPPING.values():
    g1_frames.append(value["target"])

motion_sequence = MotionSequence(
    num_frames=1,
    dof_names=[],
    body_names=g1_frames,
    fps=1,
)

for frame in usd_object.children:
    if frame.name in g1_frames:
        print(frame.name)
        motion_index = motion_sequence.get_body_indices([frame.name])[0]
        motion_sequence.body_positions[0, motion_index] = frame.location
        motion_sequence.body_rotations[0, motion_index] = frame.rotation_quaternion

motion_sequence.save(f"./data/motions/g1_reset_pose.npz")
print(f"Results saved to ./data/motions/g1_reset_pose.npz")

exit()
