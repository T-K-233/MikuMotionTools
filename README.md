# MikuMotionTools

MikuMotionTools converts motion data for humanoid robot reinforcement learning (RL). It sits between
*animation-world* motion (MMD/MikuMikuDance dances, mocap, Blender rigs) and *robotics-world* motion
(a robot's joint angles and link poses), and translates in both directions:

- **animation → robot** — make reference motions for RL training from Blender armature animation.
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
```

The last line is only needed for the example code.

Blender runs the Blender-side scripts with its own Python, which needs `rerun-sdk`:

```bash
<blender>/python/bin/python.exe -m pip install rerun-sdk
```


## Commands

You address a motion by **name**, not by path. The store layout decides where each stage lands.

```bash
mikumotion import <log.mcap> <robot.xml> <robot.urdf>        # robot log -> reference/ + <robot>/
mikumotion view <name>                                       # watch every layer in Rerun
mikumotion retarget <name> <robot.xml> <robot.urdf> <map>    # reference/ -> <robot>/
mikumotion list                                              # motions, and the robots each is solved for
```

```bash
# animation -> robot: export a mocap armature as a motion
blender ./blender-projects/zamuza.blend --python ./scripts/blender/export_mocap.py -- zamuza

# robot -> animation: play a motion on the robot's own rig
blender --background --python ./scripts/blender/animate_robot.py -- lite_pro_tracking <robot.urdf>

# robot -> animation: replay it on a VRM character
blender ./character.blend --background --python ./scripts/blender/retarget_to_vrm.py -- lite_pro_tracking <robot.urdf>
```


## The two pipelines

Conversions are named **`a_to_b`**, so inverses pair up on sight: `armature_to_motion` against
`motion_to_armature`. Use `from_` only for constructors, as in `RobotModel.from_file(path)`, and
`to_` for methods that convert what they belong to, as in `robot.to_armature_tree()`.

| Step | animation → robot | robot → animation |
|---|---|---|
| 1. read the source | *(Blender opens the .blend)* | `mcap_io.read_robot_log` |
| 2. **into the hub** | `blender.armature_to_motion` | `forward_kinematics.robot_log_to_motion` |
| 3. store the export | `MotionStore.write_reference_motion` → `reference/` | `MotionStore.write_reference_motion` → `reference/` |
| 4. retarget | `MotionRetargetingIK` solves the robot's joints | `blender.retarget_armature` bakes the robot rig onto a character rig |
| 5. store the solve | `MotionStore.write_robot_motion` → `<robot>/` | *(read it back with `read_reference_motion`)* |
| 6. **out of the hub** | *(the robot's joints are the product)* | `blender.motion_to_armature` |
| entry point | `scripts/blender/export_mocap.py` | `scripts/blender/animate_robot.py`, `retarget_to_vrm.py` |

Step 4 is where the two directions differ. A robot needs inverse kinematics (IK), because a
character's limb proportions do not map onto a robot's joints. A character needs only a rotation
transfer.


## Workflow

1. Create a Blender project and import the source motion. Set the animation to the target policy
   frequency, usually 50 Hz.

2. Export the armature with `export_mocap.py`. It writes `<preset>`, the motion itself, and
   `<preset>_reset`, the rig's rest pose. The retarget needs the rest pose to measure how far each
   bone's frame sits from the link that the bone drives.

3. Write a keypoint mapping between the source armature and the target robot. See
   [presets.py](./mikumotion/presets.py) for examples.

4. Run `mikumotion retarget` to solve the robot's joints by IK. The result lands beside the export,
   as `<robot>/<motion>.rrd`.

Match the frame rate to the policy update rate of the training environment. A mismatch forces
expensive interpolation.


## Motion format

A motion is a Rerun `.rrd` recording, stored as one directory per pipeline stage and one file per
motion, following the [Rerun dataset convention](https://huggingface.co/datasets/rerun/arkitscenes-rrd):

```
data/motions/
  reference/  <motion>.rrd  body poses as exported, robot-agnostic
  <robot>/    <motion>.rrd  that robot's joints, and the poses its solve reached
  blueprints/ <robot>.rbl   viewer layout for that robot
```

Read a motion with `MotionStore(root).read_reference_motion(name)`, or
`read_reference_motion(name, robot)` to carry that robot's solved joints on the same sequence.

[`motion_sequence.py`](./mikumotion/motion_sequence.py) documents the rest: the field shapes and
units, the entity paths, and the frame rules that a producer must follow.


## Viewing

`mikumotion view` only resolves paths and hands the layer files to the `rerun` binary. Any machine
with `rerun` and a copy of the files can do the same, with no Python:

```bash
rerun data/motions/*/zamuza.rrd data/motions/blueprints/lite_pro.rbl
```

The layers share an `application_id` and a `recording_id`, and [the viewer pools data by those two
ids](https://rerun.io/docs/concepts/apps-and-recordings), so several files open as one recording.
Each layer also embeds its own blueprint, so any one file opens sensibly alone. Blueprints do not
merge and the last one loaded wins, so pass `blueprints/<robot>.rbl` last.


## Directories

```
blender-projects/  Blender project files
mikumotion/        the library
scripts/blender/   the scripts Blender runs
data/motions/      converted motions
data/robots/       robot assets for the IK solve
```

The license does not permit redistribution of the Blender project files or the MMD motions. Get them
from their original creators; [this note](./data/motions/MMD-Motion-Sources.md) links to the
original authors. Internal developers can use the
[Google Drive](https://drive.google.com/drive/folders/1sFQmo_UvkY5xSIZKLjXLxlAOpLdI_1jz?usp=drive_link)
mirror.


## Conventions

Name generic joints after the
[SMPL-X joint names](https://github.com/vchoutas/smplx/blob/main/smplx/joint_names.py#L244C21-L268C18).

Install the [MMD Tools](https://extensions.blender.org/add-ons/mmd-tools/) add-on in Blender to
import and convert MMD motions.
