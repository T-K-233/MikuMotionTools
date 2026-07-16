"""
Stage 2+3 end-to-end (Blender side): build the lite_pro armature from URDF, drive it
from a MotionSequence (.npz produced by scripts/policy_log_to_motion.py), set up a
follow-camera + ground + lighting, and render a video (or test stills).

Launch headless, e.g.:

    blender --background --factory-startup --python scripts/examples/animate_lite_pro_from_mcap.py -- \
        --npz data/motions/lite_pro_tracking.npz \
        --urdf data/robots/berkeley_humanoids/lite_pro/urdf/lite_pro.urdf \
        --out-blend blender-projects/lite_pro_motion.blend \
        --out-video renders/lite_pro_motion.mp4

Args (after the `--`):
    --npz, --urdf         : inputs (required)
    --out-blend PATH      : also save a .blend
    --out-video PATH      : render an mp4 to PATH
    --still-frames "a,b"  : render individual PNGs at those motion frames (test mode)
    --stride N            : sample every Nth frame (fps scaled by 1/N to keep speed)
    --max-frames N        : only use the first N motion frames (0 = all)
    --res-x, --res-y      : render resolution (default 720x900)
"""

import os
import sys

import bpy
from mathutils import Vector

REPO = r"C:\Users\TK\Desktop\MikuMotionTools"
if REPO not in sys.path:
    sys.path.append(REPO)

from mikumotion import blender, urdf  # noqa: E402
from mikumotion.motion_sequence import MotionSequence  # noqa: E402


def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--npz", required=True)
    p.add_argument("--urdf", required=True)
    p.add_argument("--out-blend", default=None)
    p.add_argument("--out-video", default=None)
    p.add_argument("--still-frames", default=None)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--res-x", type=int, default=720)
    p.add_argument("--res-y", type=int, default=900)
    p.add_argument("--ffmpeg", default=None,
                   help="Path to an ffmpeg binary, used to encode the video when this "
                        "Blender build lacks FFmpeg support (falls back to 'ffmpeg' on PATH).")
    return p.parse_args(argv)


def _blender_has_ffmpeg(scene) -> bool:
    prop = scene.render.image_settings.bl_rna.properties["file_format"]
    return any(e.identifier == "FFMPEG" for e in prop.enum_items)


def _encode_png_seq(ffmpeg_bin, frames_dir, fps, out_video):
    """Encode f_%04d.png in frames_dir to an H.264 mp4 via an external ffmpeg binary."""
    import subprocess
    cmd = [
        ffmpeg_bin, "-y", "-framerate", str(int(fps)),
        "-start_number", "1", "-i", os.path.join(frames_dir, "f_%04d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out_video,
    ]
    print("ENCODE:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def render_video(scene, args, frame_start, n):
    """Render the animation to args.out_video, handling FFmpeg-less Blender builds."""
    import shutil
    os.makedirs(os.path.dirname(os.path.abspath(args.out_video)), exist_ok=True)
    scene.frame_start = frame_start
    scene.frame_end = frame_start + n - 1

    if _blender_has_ffmpeg(scene):
        scene.render.image_settings.file_format = "FFMPEG"
        scene.render.ffmpeg.format = "MPEG4"
        scene.render.ffmpeg.codec = "H264"
        scene.render.ffmpeg.constant_rate_factor = "MEDIUM"
        scene.render.filepath = args.out_video
        print(f"RENDERING VIDEO (Blender FFmpeg) {scene.frame_start}..{scene.frame_end} -> {args.out_video}")
        bpy.ops.render.render(animation=True)
        print("SAVED VIDEO:", args.out_video)
        return

    # this Blender build has no FFmpeg: render a PNG sequence then encode externally
    frames_dir = os.path.join(os.path.dirname(os.path.abspath(args.out_video)), "_frames")
    os.makedirs(frames_dir, exist_ok=True)
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = os.path.join(frames_dir, "f_")
    print(f"RENDERING PNG SEQ {scene.frame_start}..{scene.frame_end} -> {frames_dir}")
    bpy.ops.render.render(animation=True)

    ffmpeg_bin = args.ffmpeg or shutil.which("ffmpeg")
    if ffmpeg_bin:
        _encode_png_seq(ffmpeg_bin, frames_dir, scene.render.fps, args.out_video)
        print("SAVED VIDEO:", args.out_video)
    else:
        print(f"NO ffmpeg available; PNG frames left in {frames_dir}. "
              f"Encode with: ffmpeg -framerate {scene.render.fps} -i f_%04d.png "
              f"-c:v libx264 -pix_fmt yuv420p {args.out_video}")


def clean_scene():
    """Remove the factory Cube; keep the scene otherwise empty."""
    for name in ("Cube",):
        if name in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)


def add_ground():
    bpy.ops.mesh.primitive_plane_add(size=40.0, location=(0, 0, 0))
    ground = bpy.context.active_object
    ground.name = "Ground"
    mat = blender.get_or_create_material("Ground", (0.30, 0.30, 0.33, 1.0))
    ground.data.materials.clear()
    ground.data.materials.append(mat)
    return ground


def add_lighting():
    bpy.ops.object.light_add(type="SUN", location=(4, -4, 8))
    sun = bpy.context.active_object
    sun.data.energy = 3.0
    sun.rotation_euler = (0.6, 0.1, 0.5)
    # world background
    world = bpy.context.scene.world
    if world is None:
        world = bpy.data.worlds.new("World")
        bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.05, 0.05, 0.06, 1.0)
        bg.inputs[1].default_value = 1.0


def setup_follow_camera(motion, frames, frame_start, off=(1.6, -3.2, 0.7), look_z=0.8):
    """Camera + look-at empty, both keyframed to follow the pelvis in XY."""
    pi = motion.body_names.index("pelvis")
    pos = motion.body_positions

    look = bpy.data.objects.new("LookAt", None)
    look.empty_display_size = 0.2
    bpy.context.scene.collection.objects.link(look)

    cam_data = bpy.data.cameras.new("FollowCam")
    cam_data.lens = 40.0
    cam = bpy.data.objects.new("FollowCam", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    bpy.context.scene.camera = cam

    track = cam.constraints.new(type="TRACK_TO")
    track.target = look
    track.track_axis = "TRACK_NEGATIVE_Z"
    track.up_axis = "UP_Y"

    for out_i, f in enumerate(frames):
        bl = frame_start + out_i
        px, py = float(pos[f, pi, 0]), float(pos[f, pi, 1])
        look.location = (px, py, look_z)
        look.keyframe_insert("location", frame=bl)
        cam.location = (px + off[0], py + off[1], off[2] + look_z + 0.5)
        cam.keyframe_insert("location", frame=bl)
    return cam


def main():
    args = parse_args()
    assert os.path.isfile(args.npz), f"npz not found: {args.npz}"
    assert os.path.isfile(args.urdf), f"urdf not found: {args.urdf}"

    clean_scene()

    # --- build the faithful armature + meshes from URDF (Stage 1) ---
    robot = urdf.RobotModel.from_file(args.urdf)
    tree = robot.to_armature_tree()
    armature = blender.build_robot_from_urdf(robot, name="lite_pro", with_meshes=True,
                                             show_axes=False, show_names=False)

    # --- drive it from the motion (Stage 3) ---
    motion = MotionSequence.load(args.npz)
    if args.max_frames and args.max_frames < motion.num_frames:
        # shallow clip: reuse arrays up to max_frames
        clip = MotionSequence(num_frames=args.max_frames, joint_names=motion.joint_names,
                              body_names=motion.body_names, fps=motion.fps)
        clip._joint_positions[:] = motion._joint_positions[:args.max_frames]
        clip._joint_velocities[:] = motion._joint_velocities[:args.max_frames]
        clip._body_positions[:] = motion._body_positions[:args.max_frames]
        clip._body_rotations[:] = motion._body_rotations[:args.max_frames]
        clip._body_linear_velocities[:] = motion._body_linear_velocities[:args.max_frames]
        clip._body_angular_velocities[:] = motion._body_angular_velocities[:args.max_frames]
        motion = clip

    frame_start = 1
    stride = max(1, args.stride)
    n = blender.load_motion_to_armature(motion, armature, tree,
                                        frame_start=frame_start, frame_stride=stride)
    frames = list(range(0, motion.num_frames, stride))

    # --- scene dressing ---
    add_ground()
    add_lighting()
    setup_follow_camera(motion, frames, frame_start)

    # --- render config: fast Workbench, solid material colors + shadows ---
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    sh = scene.display.shading
    sh.color_type = "MATERIAL"
    sh.light = "STUDIO"
    sh.show_shadows = True
    sh.show_cavity = True
    scene.render.resolution_x = args.res_x
    scene.render.resolution_y = args.res_y
    scene.render.film_transparent = False

    if args.out_blend:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_blend)), exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=args.out_blend)
        print("SAVED BLEND:", args.out_blend)

    # --- test stills ---
    if args.still_frames:
        want = [int(x) for x in args.still_frames.split(",") if x.strip() != ""]
        for f in want:
            out_i = f // stride
            bl = frame_start + out_i
            scene.frame_set(bl)
            outp = os.path.join(REPO, "renders", f"still_{f:05d}.png")
            os.makedirs(os.path.dirname(outp), exist_ok=True)
            scene.render.filepath = outp
            bpy.ops.render.render(write_still=True)
            print("STILL:", outp)

    # --- video ---
    if args.out_video:
        render_video(scene, args, frame_start, n)


if __name__ == "__main__":
    main()
