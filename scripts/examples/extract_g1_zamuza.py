"""
blender ./blender-projects/G1_Zamuza.blend --python scripts/examples/g1_zamuza.py
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
from mikumotion.presets import UnitreeG1Mapping
from mikumotion.blender import (
    set_scene_animation_range,
    build_motion_data,
    export_motion_data,
)
from mikumotion.math import quat_mul


assert C.scene.render.fps == 50, f"Detected FPS is {C.scene.render.fps}, expected to be 50"

yyb_scaling = 0.85

section_1 = (0, 1632)

set_scene_animation_range(section_1[0], section_1[1])

source_armature = D.objects.get("YYB式初音ミクv1.02_arm")

motion_data = build_motion_data(source_armature, mapping=UnitreeG1Mapping.mmd_yyb, scaling_ratio=yyb_scaling)


# realign the frame on hand
for f in range(motion_data["body_rotations"].shape[0]):
    # pelvis (root)
    motion_data["body_rotations"][f, 0, :] = quat_mul(motion_data["body_rotations"][f, 0, :], np.array([0.5, 0.5, 0.5, -0.5]))
    motion_data["body_rotations"][f, 0, :] = quat_mul(motion_data["body_rotations"][f, 0, :], np.array([0.985, 0, -0.174, 0]))
    # left hand
    motion_data["body_rotations"][f, 3, :] = quat_mul(motion_data["body_rotations"][f, 3, :], np.array([0.5, 0.5, 0.5, 0.5]))
    # right hand
    motion_data["body_rotations"][f, 6, :] = quat_mul(motion_data["body_rotations"][f, 6, :], np.array([0.5, 0.5, 0.5, 0.5]))
    # left feet
    motion_data["body_rotations"][f, 10, :] = quat_mul(motion_data["body_rotations"][f, 10, :], np.array([0.5, 0.5, 0.5, -0.5]))
    motion_data["body_rotations"][f, 10, :] = quat_mul(motion_data["body_rotations"][f, 10, :], np.array([0.991, 0., -0.131, 0.]))
    # right feet
    motion_data["body_rotations"][f, 13, :] = quat_mul(motion_data["body_rotations"][f, 13, :], np.array([0.5, 0.5, 0.5, -0.5]))
    motion_data["body_rotations"][f, 13, :] = quat_mul(motion_data["body_rotations"][f, 13, :], np.array([0.991, 0., -0.131, 0.]))

motion_data["body_positions"][:, 10, :] -= 0.1
motion_data["body_positions"][:, 13, :] -= 0.1


export_motion_data("./data/motions/g1_zamuza_0_960.npz", motion_data)
