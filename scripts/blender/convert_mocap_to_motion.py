"""
Direction 1, animation -> robot, step 2 of 3: a mocap rig's animation becomes a motion.

Writes two motions per rig: ``<preset>_reset`` (the rest pose, which the retarget measures its
offset against) and ``<preset>`` (the animation). Which armature, which bones, and how to scale
and turn the rig all come from ``mikumotion.presets.MOCAP_RIGS``.

    blender <file>.blend --python scripts/blender/convert_mocap_to_motion.py -- <preset>

for example:

    blender blender-projects/zamuza.blend \
        --python scripts/blender/convert_mocap_to_motion.py -- zamuza

Then solve a robot's joints for it with
`mikumotion retarget <preset> <robot.xml> <robot.urdf> <map>`.
"""

import os
import sys

import bpy

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from mikumotion.blender import (  # noqa: E402
    armature_to_motion,
    script_args,
    set_armature_to_pose,
    set_armature_to_rest,
    set_scene_animation_range,
)
from mikumotion.motion_sequence import MotionStore, rotate_motion  # noqa: E402
from mikumotion.presets import MOCAP_RIGS  # noqa: E402


def export(armature, rig, name, store):
    """Read the current frame range off the armature and store it as ``name``."""
    motion = armature_to_motion(armature, rig["bones"])
    motion.body_positions *= rig["scale"]
    motion.body_linear_velocities *= rig["scale"]

    # Blender rigs are +Y forward; the robot convention is +X forward
    motion = rotate_motion(motion, rig["rotate_z"])

    print(f"{name}: {motion!r}")
    print(f"  {store.write_reference_motion(name, motion)}")


def main():
    names = script_args()
    assert len(names) == 1, f"expected one preset name, one of {sorted(MOCAP_RIGS)}"
    preset_name = names[0]
    rig = MOCAP_RIGS[preset_name]

    scene_fps = bpy.context.scene.render.fps
    assert scene_fps == 50, f"scene is {scene_fps} fps, expected 50"

    armature = bpy.data.objects[rig["armature"]]
    store = MotionStore()

    set_scene_animation_range(0, 1)
    set_armature_to_rest(armature)
    export(armature, rig, f"{preset_name}_reset", store)

    set_scene_animation_range(*rig["frames"])
    set_armature_to_pose(armature)
    export(armature, rig, preset_name, store)


if __name__ == "__main__":
    main()
