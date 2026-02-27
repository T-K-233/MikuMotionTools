"""
Replay retargeted motion in MuJoCo and export as MP4 video.

Usage:
```bash
uv run scripts/examples/visuialize_retargeting_result_mujoco.py \
    --motion ./data/motions/zamuza_0_2000_retargeted.npz \
    --robot /home/tk/Desktop/onshape-to-robot-workflow/data/miku/mjcf/miku.xml \
    --output ./data/motions/zamuza_0_2000_retargeted.mp4
```
"""

import argparse
import subprocess

import numpy as np
import mujoco

from mikumotion.motion_sequence import MotionSequence


def main():
    parser = argparse.ArgumentParser(description="Replay retargeted motion in MuJoCo and export as MP4.")
    parser.add_argument("--motion", type=str, required=True, help="Path to retargeted motion .npz file")
    parser.add_argument(
        "--robot", type=str,
        default="/home/tk/Desktop/onshape-to-robot-workflow/data/miku/mjcf/miku.xml",
        help="Path to the robot MJCF XML file",
    )
    parser.add_argument("--output", type=str, default=None, help="Output MP4 path (default: same as motion with .mp4)")
    parser.add_argument("--width", type=int, default=1920, help="Video width in pixels")
    parser.add_argument("--height", type=int, default=1080, help="Video height in pixels")
    args = parser.parse_args()

    if args.output is None:
        args.output = args.motion.replace(".npz", ".mp4")

    motion = MotionSequence.load(args.motion)
    fps = int(np.asarray(motion.fps).flat[0])

    model = mujoco.MjModel.from_xml_path(args.robot)
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)

    pelvis_body_idx = motion.get_body_indices(["pelvis"])[0]

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.distance = 2.5
    camera.elevation = -15.0
    camera.azimuth = 135.0

    ffmpeg_cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo",
        "-pixel_format", "rgb24",
        "-video_size", f"{args.width}x{args.height}",
        "-framerate", str(fps),
        "-i", "pipe:0",
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-crf", "18",
        "-preset", "medium",
        args.output,
    ]
    ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    duration = float(np.asarray(motion.duration).flat[0])
    print(f"Rendering {motion.num_frames} frames at {fps} FPS ({duration:.1f}s) ...")

    for frame_idx in range(motion.num_frames):
        root_pos = motion.body_positions[frame_idx, pelvis_body_idx]
        root_quat = motion.body_rotations[frame_idx, pelvis_body_idx]

        data.qpos[0:3] = root_pos
        data.qpos[3:7] = root_quat
        data.qpos[7:] = motion.joint_positions[frame_idx]

        mujoco.mj_forward(model, data)

        camera.lookat[:] = data.xpos[model.body("pelvis").id]

        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()

        ffmpeg_proc.stdin.write(pixels.tobytes())

        if (frame_idx + 1) % 500 == 0 or frame_idx == motion.num_frames - 1:
            print(f"  frame {frame_idx + 1}/{motion.num_frames}")

    ffmpeg_proc.stdin.close()
    ffmpeg_proc.wait()

    if ffmpeg_proc.returncode != 0:
        stderr = ffmpeg_proc.stderr.read().decode()
        raise RuntimeError(f"ffmpeg failed (code {ffmpeg_proc.returncode}):\n{stderr}")

    renderer.close()
    print(f"Video saved to {args.output}")


if __name__ == "__main__":
    main()
