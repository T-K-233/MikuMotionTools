"""
Build a Blender armature + visual meshes for the Berkeley-Humanoids `lite_pro` robot
from its URDF.

The lite_pro description is not bundled with this repository. Obtain it from
https://github.com/Berkeley-Humanoids/Lite-Description and place it under:

    data/robots/berkeley_humanoids/lite_pro/
        urdf/lite_pro.urdf
        meshes/visual/*.stl

Then run from the repository root:

```bash
blender ./blender-projects/test.blend --python ./scripts/examples/build_lite_pro_armature.py
```

This creates an armature named "lite_pro" with one bone per link and all visual
meshes imported, placed, colored, and parented to their bones (the robot at its
zero / rest pose).
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

from mikumotion import blender, urdf

importlib.reload(urdf)
importlib.reload(blender)

C = bpy.context
D = bpy.data
O = bpy.ops


""" Everything else follows """

import numpy as np

np.set_printoptions(precision=3, suppress=True)

# Path to the lite_pro URDF (resolved relative to the current working directory,
# i.e. the repository root when launched as shown above).
URDF_PATH = os.path.join(
    mikumotion_path,
    "data", "robots", "berkeley_humanoids", "lite_pro", "urdf", "lite_pro.urdf",
)

assert os.path.isfile(URDF_PATH), (
    f"URDF not found: {URDF_PATH}\n"
    "Download the lite_pro description into data/robots/berkeley_humanoids/lite_pro/ "
    "(see the docstring at the top of this file)."
)

robot = urdf.RobotModel.from_file(URDF_PATH)
print(robot)
print("root link:", robot.root_link())

blender.build_robot_from_urdf(robot, name="lite_pro", with_meshes=True)
