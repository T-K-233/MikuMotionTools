"""
Retarget motion from Miku source armature to Zamuza target armature.

Usage:
    blender ./blender-projects/test.blend --python ./scripts/examples/retarget_zamuza.py
"""

import os
import sys

import bpy
import importlib
from mathutils import Vector

# Add project to path
blend_path = os.path.dirname(bpy.data.filepath)
mikumotion_path = os.getcwd()
for p in (blend_path, mikumotion_path):
    if p and p not in sys.path:
        sys.path.insert(0, p)

import mikumotion.blender as blender_module
importlib.reload(blender_module)

from mikumotion.blender import (
    WM_OT_modal_retarget_motion,
    RetargetConfig,
    set_retarget_config,
)

# ============================================================
# CONFIG
# ============================================================

CONFIG = RetargetConfig(
    source_armature_name="YYB式初音ミクv1.02_arm",
    target_armature_name="Armature",
    bone_map={
        "pelvis": "下半身",
        "torso": "上半身2",
        "left_upper_arm": "腕.L",
        "left_lower_arm": "ひじ.L",
        "left_hand": "手首.L",
        "right_upper_arm": "腕.R",
        "right_lower_arm": "ひじ.R",
        "right_hand": "手首.R",
        "left_upper_leg": "足.L",
        "left_lower_leg": "ひざ.L",
        "left_foot": "足首.L",
        "right_upper_leg": "足.R",
        "right_lower_leg": "ひざ.R",
        "right_foot": "足首.R",
    },
    translation_root_source="センター",
    translation_root_target="pelvis",
    auto_map_same_names=False,
    ignore_twist=False,
    bone_axis_local=Vector((0.0, 1.0, 0.0)),
    use_scene_frame_range=True,
    frame_start=1,
    frame_end=250,
    insert_keyframes=True,
    clear_existing_keys=True,
    frames_per_tick=1,
    timer_step_sec=0.0,
)

# ============================================================
# REGISTER / RUN
# ============================================================

classes = (WM_OT_modal_retarget_motion,)

def register():
    for c in classes:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(classes):
        bpy.utils.unregister_class(c)


# Safe re-register for repeated script runs in Blender text editor
try:
    unregister()
except Exception:
    pass
register()

# Set config and run (UI stays responsive, ESC cancels)
set_retarget_config(CONFIG)
bpy.ops.wm.modal_retarget_motion('INVOKE_DEFAULT')
