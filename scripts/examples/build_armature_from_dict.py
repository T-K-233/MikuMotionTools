"""
```bash
blender ./blender-projects/test.blend --python ./scripts/tests/build_armature.py
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
from mikumotion.armature_tree import ArmatureTree


np.set_printoptions(precision=3, suppress=True)

tree = ArmatureTree(
    body_names=[
        "pelvis",  # 0
        "torso",  # 1
        "head",  # 2
        "left_upper_arm",  # 3
        "left_lower_arm",  # 4
        "left_hand",  # 5
        "right_upper_arm",  # 6
        "right_lower_arm",  # 7
        "right_hand",  # 8
        "left_upper_leg",  # 9
        "left_lower_leg",  # 10
        "left_foot",  # 11
        "right_upper_leg",  # 12
        "right_lower_leg",  # 13
        "right_foot",  # 14
    ],
    body_parent_indices=np.array([
        -1,
        0,
        1,
        1,  # left_upper_arm
        3,
        4,
        1,  # right_upper_arm
        6,
        7,
        0,  # left_upper_leg
        9,
        10,
        0,  # right_upper_leg
        12,
        13,
    ], dtype=np.int32),
    local_translations=np.array([
        [0, 0, 0.000],
        [0, 0, 0.120],
        [0, 0, 0.320],
        [0, 0.090, 0.260],
        [0, 0, 0.200],
        [0, 0, 0.180],
        [0, -0.090, 0.260],
        [0, 0, 0.200],
        [0, 0, 0.180],
        [0, 0.080, 0],
        [0, 0, 0.420],
        [0, 0, 0.400],
        [0, -0.080, 0],
        [0, 0, 0.420],
        [0, 0, 0.400],
    ], dtype=np.float32),
    local_rotations=np.array([
        [1, 0, 0, 0],
        [1, 0, 0, 0],
        [1, 0, 0, 0],
        [0.707, -0.707, 0, 0],
        [1, 0, 0, 0],
        [1, 0, 0, 0],
        [0.707, 0.707, 0, 0],
        [1, 0, 0, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0.707, 0, 0.707, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0.707, 0, 0.707, 0],
    ], dtype=np.float32),
)

print(tree)

blender.build_armature(tree)
