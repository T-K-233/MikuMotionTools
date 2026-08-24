"""
Scene dressing and rendering for the Blender-side scripts: ground, light, a camera that
follows the motion, and a video render.

Blender is often built without FFmpeg, and the build bundled here is one of those. Its RNA
enum still lists ``FFMPEG``, so no flag tells you whether the encoder is there. This module
therefore always renders a PNG sequence and hands it to an external encoder.

``MIKUMOTION_FFMPEG`` names that encoder when it is not simply ``ffmpeg`` on the path. It is a
property of the machine, not of a render, so it is an environment variable rather than a flag
on every script.
"""

import os
import subprocess

import bpy
from mathutils import Vector

FFMPEG = os.environ.get("MIKUMOTION_FFMPEG", "ffmpeg")
RESOLUTION = (720, 900)
CAMERA_OFFSET = (2.6, -5.4, 1.3)
CAMERA_LOOK_HEIGHT = 0.5  # aim low enough that crouching and prone poses stay framed


def add_ground():
    """Add a large matte plane at z = 0, which makes contact with the floor readable."""
    bpy.ops.mesh.primitive_plane_add(size=40.0, location=(0, 0, 0))
    ground = bpy.context.active_object
    ground.name = "Ground"

    material = bpy.data.materials.new("Ground")
    material.use_nodes = True
    material.diffuse_color = (0.30, 0.30, 0.33, 1.0)
    ground.data.materials.append(material)
    return ground


def add_lighting():
    """Add a single sun, and darken the world background."""
    bpy.ops.object.light_add(type="SUN", location=(4, -4, 8))
    sun = bpy.context.active_object
    sun.data.energy = 3.0
    sun.rotation_euler = (0.6, 0.1, 0.5)

    world = bpy.data.worlds.new("World")
    bpy.context.scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.05, 0.05, 0.06, 1.0)
    return sun


def follow_camera(motion, frame_start):
    """Add a camera, keyframed to track the root body's position across the motion."""
    root = motion.body_positions[:, 0]

    target = bpy.data.objects.new("LookAt", None)
    target.empty_display_size = 0.2
    bpy.context.scene.collection.objects.link(target)

    camera_data = bpy.data.cameras.new("FollowCam")
    camera_data.lens = 40.0
    camera = bpy.data.objects.new("FollowCam", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    bpy.context.scene.camera = camera

    track = camera.constraints.new(type="TRACK_TO")
    track.target = target
    track.track_axis = "TRACK_NEGATIVE_Z"
    track.up_axis = "UP_Y"

    for index in range(motion.num_frames):
        frame = frame_start + index
        position = Vector((float(root[index, 0]), float(root[index, 1]), CAMERA_LOOK_HEIGHT))
        target.location = position
        target.keyframe_insert("location", frame=frame)
        camera.location = position + Vector(CAMERA_OFFSET)
        camera.keyframe_insert("location", frame=frame)
    return camera


def configure_render(shading):
    """
    Set up the Workbench renderer at a fixed resolution.

    Pass MATERIAL for ``shading`` to render a robot, or TEXTURE to render a character.
    """
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = shading
    scene.display.shading.light = "STUDIO"
    scene.display.shading.show_shadows = True
    scene.render.resolution_x, scene.render.resolution_y = RESOLUTION
    scene.render.film_transparent = False
    return scene


def dress_scene(motion, shading):
    """
    Add the ground, the light and a camera that follows the motion, then set up the renderer.

    Both ``convert_motion_to_*`` scripts want the same scene and differ only in shading:
    MATERIAL for a robot's flat link colours, TEXTURE for a character's skin. Frame 1 is where
    ``motion_to_armature`` puts the motion's first frame.
    """
    add_ground()
    add_lighting()
    follow_camera(motion, frame_start=1)
    configure_render(shading)


def render_animation(out_video):
    """Render the scene's frame range to ``out_video`` through a PNG sequence."""
    scene = bpy.context.scene
    frames_dir = os.path.join(os.path.dirname(os.path.abspath(out_video)), "_frames")
    os.makedirs(frames_dir, exist_ok=True)

    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = os.path.join(frames_dir, "f_")
    print(f"[render] frames {scene.frame_start}..{scene.frame_end} -> {frames_dir}")
    bpy.ops.render.render(animation=True)

    command = [
        FFMPEG, "-y", "-framerate", str(scene.render.fps),
        "-start_number", "1", "-i", os.path.join(frames_dir, "f_%04d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out_video,
    ]
    print("[render]", " ".join(command))
    subprocess.run(command, check=True)
    print("[render] saved", out_video)
