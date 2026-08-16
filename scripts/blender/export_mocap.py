"""
Direction 1, animation -> robot: export a Blender mocap armature as a motion.

Writes two motions per rig: ``<preset>_reset`` (the rest pose, used as the retarget
reference) and ``<preset>`` (the animation). Which armature, which bones, and how to
scale and turn the rig all come from ``mikumotion.presets.MOCAP_EXPORTS``.

    blender <file>.blend --python scripts/blender/export_mocap.py -- <preset>

for example:

    blender ./blender-projects/Zamuza.blend --python scripts/blender/export_mocap.py -- zamuza

Then solve a robot's joints for it with `mikumotion retarget <preset> <robot.xml>`.
"""

import os
import sys

import bpy

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from mikumotion.blender import (  # noqa: E402
    motion_from_armature,
    set_armature_to_pose,
    set_armature_to_rest,
    set_scene_animation_range,
)
from mikumotion.motion_sequence import rotate_motion  # noqa: E402
from mikumotion.presets import MOCAP_EXPORTS  # noqa: E402
from mikumotion.motion_sequence import MotionStore  # noqa: E402


def export(armature, preset, name, store):
    """Read the current frame range off the armature and store it as ``name``."""
    motion = motion_from_armature(armature, preset["bones"])
    motion.body_positions *= preset["scale"]
    motion.body_linear_velocities *= preset["scale"]

    # Blender rigs are +Y forward; the robot convention is +X forward
    motion = rotate_motion(motion, preset["rotate_z"])

    print(f"{name}: {motion!r}")
    print(f"  {store.write_motion(name, motion)}")


def main():
    argv = sys.argv
    names = argv[argv.index("--") + 1:] if "--" in argv else []
    assert len(names) == 1, f"expected one preset name, one of {sorted(MOCAP_EXPORTS)}"
    preset_name = names[0]
    preset = MOCAP_EXPORTS[preset_name]

    scene_fps = bpy.context.scene.render.fps
    assert scene_fps == 50, f"scene is {scene_fps} fps, expected 50"

    armature = bpy.data.objects[preset["armature"]]
    store = MotionStore()

    set_scene_animation_range(0, 1)
    set_armature_to_rest(armature)
    export(armature, preset, f"{preset_name}_reset", store)

    set_scene_animation_range(*preset["frames"])
    set_armature_to_pose(armature)
    export(armature, preset, preset_name, store)


if __name__ == "__main__":
    main()
