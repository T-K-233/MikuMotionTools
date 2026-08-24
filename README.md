# MikuMotionTools

MikuMotionTools converts motion data between character animation and humanoid robot
reinforcement learning (RL). It sits between *animation-world* motion (MMD dances, mocap takes,
Blender rigs) and *robotics-world* motion (a robot's joint angles and link poses), and it
translates in both directions:

- **animation → robot** — build reference motions for RL training from a Blender armature.
- **robot → animation** — show what a trained policy did, as a character animation.

One data structure holds everything: a [Rerun](https://rerun.io) `.rrd` motion file. Every other
module converts into or out of that format.

## Install

```bash
git clone https://github.com/T-K-233/MikuMotionTools.git
cd ./MikuMotionTools/
uv pip install -e .
uv sync
uv sync --extra examples
uv sync --extra hf
```

Blender runs the Blender-side scripts with its own Python, which needs `rerun-sdk`:

```bash
<blender>/python/bin/python.exe -m pip install rerun-sdk
```

## Example

You address a motion by **name**, not by path. The store layout decides where each stage lands.

```bash
# robot -> animation: a policy rollout becomes a motion, then a video
mikumotion import ./rollout.mcap <robot.xml> <robot.urdf> --name lite_pro_tracking
mikumotion view lite_pro_tracking
blender --background --python ./scripts/blender/convert_motion_to_robot.py -- lite_pro_tracking <robot.urdf>
```

```bash
# animation -> robot: a Blender rig becomes a robot reference motion
blender ./blender-projects/zamuza.blend --background --python ./scripts/blender/convert_character_to_mocap.py -- zamuza
blender ./blender-projects/zamuza.blend --python ./scripts/blender/convert_mocap_to_motion.py -- zamuza
mikumotion retarget zamuza <robot.xml> <robot.urdf> ZAMUZA_TO_LITE_PRO
```

Each Blender script is one step of a chain, named for it: `character -> mocap -> motion -> robot`
going in, `motion -> character` coming back out. A new source format — MMD, Mixamo, ActorCore —
is an entry in `presets.CHARACTER_RIGS`, not another script. They need `bpy`, so Blender runs them.

## Dataset remote

A motion is written on a workstation that has Blender, an MMD source and a robot asset. A
training run or a deployment reads the `.rrd` files and needs none of those. A remote dataset
mirrors the store's layout one for one, so moving a motion is a file transfer:

```bash
export MIKUMOTION_REMOTE=hf://<owner>/mikumotion-motions
mikumotion push zamuza    # on the workstation that solved it
mikumotion pull zamuza    # on the training or the deployment machine
```

Hugging Face is the backend, `hf://<owner>/<name>[@<revision>]`, and a revision pins a training
run to one version of the dataset. Each direction is one call to `upload_folder` or
`snapshot_download`, so a repeated transfer resumes and skips what has not changed, and Xet
storage keeps one copy of the robot geometry that every robot layer repeats. An S3 bucket,
`s3://<bucket>/<prefix>`, is the same three methods over `boto3`. See
[the dataset remote](https://t-k-233.github.io/MikuMotionTools/#the-dataset-remote).

## License

MIT. See [LICENSE](./LICENSE).

The license does not cover the Blender project files or the MMD motions, and does not permit
their redistribution, so `mikumotion push` creates a private dataset. Get them from their
original creators;
[this note](./data/motions/MMD-Motion-Sources.md) links to the original authors. Internal
developers can use the
[Google Drive](https://drive.google.com/drive/folders/1sFQmo_UvkY5xSIZKLjXLxlAOpLdI_1jz?usp=drive_link)
mirror.
