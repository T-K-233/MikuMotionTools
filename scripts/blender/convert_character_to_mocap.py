"""
Direction 1, animation -> robot, step 1 of 3: a character's animation becomes a mocap rig's.

A character arrives on the armature its authoring tool gave it, under that tool's bone names and
axis conventions: MMD, Mixamo, ActorCore, Meshcapade. ``convert_mocap_to_motion.py`` reads the
small mocap rig instead. This script transfers the animation from one to the other, one keyframe
per frame, so the mocap rig holds it on its own and the character's armature is not needed again.
It saves the ``.blend`` file.

Nothing here is specific to one format. The armature to read, the bone map, and the bone that
places the rig in the world all come from ``mikumotion.presets.CHARACTER_RIGS``, so a new source
format is an entry in that table rather than another script.

Skip this step for a rig that carries its own ``<body>.frame`` helper bones, as the accad and
actorcore rigs do; those export directly.

    blender <file>.blend --background \
        --python scripts/blender/convert_character_to_mocap.py -- <preset>

for example:

    blender blender-projects/zamuza.blend --background \
        --python scripts/blender/convert_character_to_mocap.py -- zamuza

The transfer is the same ``blender.retarget_armature`` that bakes a robot rig onto a VRM
character. Both jobs put one rig's motion onto a differently proportioned rig.

The bake also converts the frame rate. The character's keys sit on its own rate, ``fps`` in
``CHARACTER_RIGS`` (30 for MMD), and setting the scene to 50 fps does not move them. So the bake
samples the character at 30/50 of a frame per scene frame, and the mocap rig carries one key per
50 fps frame. The scene's frame range counts the character's own frames, as its import set it.
"""

import math
import os
import sys

import bpy

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from mikumotion.blender import BAKED_FPS, retarget_armature, scene_fps, script_args  # noqa: E402
from mikumotion.presets import CHARACTER_RIGS, MOCAP_RIGS  # noqa: E402


def main():
    names = script_args()
    assert len(names) == 1, f"expected one preset name, one of {sorted(CHARACTER_RIGS)}"
    preset_name = names[0]
    character = CHARACTER_RIGS[preset_name]

    source = bpy.data.objects[character["armature"]]
    target = bpy.data.objects[MOCAP_RIGS[preset_name]["armature"]]
    scene = bpy.context.scene
    fps = scene_fps()
    assert fps == 50, f"scene is {fps} fps, expected 50"

    time_scale = character["fps"] / fps
    frames = retarget_armature(source, target, character["bone_map"], "pelvis",
                               character["translation_root"],
                               math.ceil(scene.frame_start / time_scale),
                               math.floor(scene.frame_end / time_scale), time_scale)
    target[BAKED_FPS] = fps
    print(f"baked {frames} frames of {source.name!r} ({character['fps']} fps) onto "
          f"{target.name!r} ({fps} fps)")

    bpy.ops.wm.save_mainfile()
    print(f"saved {bpy.data.filepath}")


if __name__ == "__main__":
    main()
