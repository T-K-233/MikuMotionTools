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

xml_path = "/home/tk/Desktop/Pianist/source/pianist/data/robots/ude_better_dummy/mjcf/ude_dummy.xml"

tree = ArmatureTree.from_mjcf(xml_path)
print(tree)

blender.build_armature(tree)
