"""
Stage 4: retarget the lite_pro robot motion onto a VRM/VRoid humanoid armature and
let the VRM replay it.

Pipeline:
  1. open a VRM .blend (the character; its armature is the retarget TARGET),
  2. build the lite_pro armature (bones only) and drive it from the motion .npz
     (Stage 2/3) -> retarget SOURCE,
  3. bake_retarget the robot's per-link world orientations onto the VRM humanoid
     bones (mikumotion.presets.LITE_PRO_TO_VROID_BONE_MAP),
  4. dress the scene (ground, sun, follow-camera) and render a video.

Launch headless, e.g.:

    blender --background blender-projects/nai_model.blend \
        --python scripts/examples/retarget_lite_pro_to_vrm.py -- \
        --npz  data/motions/lite_pro_tracking.npz \
        --urdf data/robots/berkeley_humanoids/lite_pro/urdf/lite_pro.urdf \
        --out-blend blender-projects/nai_model_motion.blend \
        --out-video renders/nai_model_motion.mp4 \
        --ffmpeg .venv/lib/site-packages/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe
"""

import os
import sys
import types

import bpy

REPO = r"C:\Users\TK\Desktop\MikuMotionTools"
if REPO not in sys.path:
    sys.path.append(REPO)
# reuse the Stage-3 scene/render helpers
sys.path.append(os.path.join(REPO, "scripts", "examples"))

from mikumotion import blender, urdf, presets  # noqa: E402
from mikumotion.motion_sequence import MotionSequence  # noqa: E402
import animate_lite_pro_from_mcap as animate  # noqa: E402


def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--npz", required=True)
    p.add_argument("--urdf", required=True)
    p.add_argument("--target-armature", default=None,
                   help="Name of the VRM armature object (default: first ARMATURE in the file).")
    p.add_argument("--out-blend", default=None)
    p.add_argument("--out-video", default=None)
    p.add_argument("--still-frames", default=None)
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--res-x", type=int, default=720)
    p.add_argument("--res-y", type=int, default=900)
    p.add_argument("--ffmpeg", default=None)
    p.add_argument("--ignore-twist", action="store_true")
    p.add_argument("--shading", default="TEXTURE", choices=["TEXTURE", "MATERIAL", "SINGLE"])
    return p.parse_args(argv)


def clip_motion(motion, max_frames):
    if not max_frames or max_frames >= motion.num_frames:
        return motion
    clip = MotionSequence(num_frames=max_frames, joint_names=motion.joint_names,
                          body_names=motion.body_names, fps=motion.fps)
    clip._joint_positions[:] = motion._joint_positions[:max_frames]
    clip._joint_velocities[:] = motion._joint_velocities[:max_frames]
    clip._body_positions[:] = motion._body_positions[:max_frames]
    clip._body_rotations[:] = motion._body_rotations[:max_frames]
    clip._body_linear_velocities[:] = motion._body_linear_velocities[:max_frames]
    clip._body_angular_velocities[:] = motion._body_angular_velocities[:max_frames]
    return clip


def main():
    args = parse_args()
    assert os.path.isfile(args.npz), f"npz not found: {args.npz}"
    assert os.path.isfile(args.urdf), f"urdf not found: {args.urdf}"

    # --- locate the VRM target armature (already loaded from the opened .blend) ---
    if args.target_armature:
        target = bpy.data.objects.get(args.target_armature)
    else:
        target = next((o for o in bpy.data.objects if o.type == "ARMATURE"), None)
    assert target is not None and target.type == "ARMATURE", "no VRM target armature found"
    print("TARGET ARMATURE:", target.name)

    # --- build the robot source armature (bones only) and drive it (Stage 2/3) ---
    robot = urdf.RobotModel.from_file(args.urdf)
    tree = robot.to_armature_tree()
    source = blender.build_robot_from_urdf(robot, name="lite_pro_src", with_meshes=False,
                                           show_axes=False, show_names=False)

    motion = clip_motion(MotionSequence.load(args.npz), args.max_frames)
    frame_start = 1
    n = blender.load_motion_to_armature(motion, source, tree, frame_start=frame_start)
    frames = list(range(0, motion.num_frames))

    # --- retarget robot -> VRM (bake keyframes onto the VRM humanoid bones) ---
    tt, ts = presets.LITE_PRO_TO_VROID_TRANSLATION_ROOT
    blender.bake_retarget(
        source, target, presets.LITE_PRO_TO_VROID_BONE_MAP,
        translation_root_target=tt, translation_root_source=ts,
        frame_start=frame_start, frame_end=frame_start + n - 1,
        ignore_twist=args.ignore_twist,
        # robot URDF link frames and VRM bone frames share the T-pose rest but not
        # local axis conventions -> transfer WORLD rotation deltas, not rest-relative
        # ones; and track the root position directly (robot pelvis rest is at the origin
        # while the VRM hips rest is at standing height).
        use_rest_orientation_offsets=False,
        track_root_world_position=True,
    )

    # hide the source robot from the render
    source.hide_set(True)
    source.hide_render = True

    # --- scene dressing + render config ---
    animate.add_ground()
    animate.add_lighting()
    # pull the camera back + look lower so both standing and low/prone poses stay framed
    animate.setup_follow_camera(motion, frames, frame_start, off=(2.6, -5.4, 1.3), look_z=0.5)

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    sh = scene.display.shading
    sh.color_type = args.shading
    sh.light = "STUDIO"
    sh.show_shadows = True
    scene.render.resolution_x = args.res_x
    scene.render.resolution_y = args.res_y
    scene.frame_start = frame_start
    scene.frame_end = frame_start + n - 1

    if args.out_blend:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_blend)), exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=args.out_blend)
        print("SAVED BLEND:", args.out_blend)

    if args.still_frames:
        for f in [int(x) for x in args.still_frames.split(",") if x.strip()]:
            scene.frame_set(frame_start + f)
            outp = os.path.join(REPO, "renders", f"vrm_still_{f:05d}.png")
            os.makedirs(os.path.dirname(outp), exist_ok=True)
            scene.render.image_settings.file_format = "PNG"
            scene.render.filepath = outp
            bpy.ops.render.render(write_still=True)
            print("STILL:", outp)

    if args.out_video:
        ns = types.SimpleNamespace(out_video=args.out_video, ffmpeg=args.ffmpeg)
        animate.render_video(scene, ns, frame_start, n)


if __name__ == "__main__":
    main()
