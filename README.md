# MikuMotionTools

MikuMotionTools converts motion data for humanoid robot RL. It sits between *animation-world*
motion (MMD/MikuMikuDance dances, mocap, Blender rigs) and *robotics-world* motion (a robot's
joint angles and link poses). It translates in both directions:

- **animation → robot** — make reference motions for RL training from Blender armature animation.
- **robot → animation** — show what a trained policy did, as a character animation.

One data structure holds everything: a [Rerun](https://rerun.io) `.rrd` motion file. Every other
module converts into or out of that format. Because motions are `.rrd` files, you can keep them on
a server, preview one by streaming down a single recording, and train from the same files. A
training environment reads only the columns it needs.


## Getting Started

Clone the repository:

```bash
git clone https://github.com/T-K-233/MikuMotionTools.git
cd ./MikuMotionTools/
uv pip install -e .
```

Install the dependencies:

```bash
uv sync
```

Install the extra dependencies that the example code needs:

```bash
uv sync --extra examples
```


## The two directions

Conversions are named **`a_to_b`**. A name reads as the transformation it performs, and inverses
pair up on sight: `armature_to_motion` against `motion_to_armature`. Use `from_` only for
constructors that build the thing they are called on, as in `RobotModel.from_file(path)`. Use `to_`
for methods that convert what they belong to, as in `robot.to_armature_tree()`.

The two pipelines mirror each other, step for step:

| Step | animation → robot | robot → animation |
|---|---|---|
| 1. read the source | *(Blender opens the .blend)* | `mcap_io.read_robot_log` |
| 2. **into the hub** | `blender.armature_to_motion` | `forward_kinematics.robot_log_to_motion` |
| 3. store the export | `MotionStore.write_reference_motion` → `reference/` | `MotionStore.write_reference_motion` → `reference/` |
| 4. retarget | `MotionRetargetingIK` solves the robot's joints | `blender.retarget_armature` bakes the robot rig onto a character rig |
| 5. store the solve | `MotionStore.write_robot_motion` → `<robot>/` | *(read it back with `read_reference_motion`)* |
| 6. **out of the hub** | *(the robot's joints are the product)* | `blender.motion_to_armature` |
| entry point | `scripts/blender/export_mocap.py` | `scripts/blender/animate_robot.py`, `retarget_to_vrm.py` |

Read across a row to see which step matches which. Read down a column to see one pipeline. Step 4 is
where the two directions differ. A robot needs IK, because a character's limb proportions do not map
onto a robot's joints. A character needs only a rotation transfer.


## Commands

Motions are addressed by **name**, not by path. The store layout decides where each stage lands.

```bash
mikumotion import <log.mcap> <robot.xml> <robot.urdf>        # robot log -> reference/ + <robot>/
mikumotion view <name>                                       # watch every layer in Rerun
mikumotion retarget <name> <robot.xml> <robot.urdf> <map>    # reference/ -> <robot>/
mikumotion list                                              # motions, and the robots each is solved for
```

Blender runs the Blender-side scripts. They read and write the same store, so Blender's own Python
needs `rerun-sdk`:

```bash
<blender>/python/bin/python.exe -m pip install rerun-sdk
```

```bash
# animation -> robot: export a mocap armature as a motion
blender ./blender-projects/zamuza.blend --python ./scripts/blender/export_mocap.py -- zamuza

# robot -> animation: play a motion on the robot's own rig
blender --background --python ./scripts/blender/animate_robot.py -- lite_pro_tracking <robot.urdf>

# robot -> animation: replay it on a VRM character
blender ./character.blend --background --python ./scripts/blender/retarget_to_vrm.py -- lite_pro_tracking <robot.urdf>
```


### Viewing without this package

`mikumotion view` only resolves paths. It gives the layer files to the `rerun` binary, and that is
all it does. Any machine with `rerun` installed and a copy of the files can do the same, with no
Python. Dragging the files into the viewer also works:

```bash
rerun data/motions/*/zamuza.rrd data/motions/blueprints/lite_pro.rbl
```

The layers share an `application_id` and a `recording_id`. [The viewer pools data by those two
ids](https://rerun.io/docs/concepts/apps-and-recordings), so several files open as one recording.
You do not need to merge them first.

Each layer also embeds its own blueprint, so any one file opens sensibly on its own. Blueprints do
not merge, and the last one loaded wins. When you open several files together, pass
`blueprints/<robot>.rbl` last to be sure of the robot layout.


## General Workflow

1. Create a Blender project and import the source motion. Set the animation to the target policy
   frequency, usually 50 Hz.

2. Export the armature's key frame positions and orientations with `export_mocap.py`. This writes
   two motions: `<preset>`, labelled with the source bone names and with no joint data, and
   `<preset>_reset`, the rig's rest pose. The retarget needs the rest pose. It measures how far each
   bone's frame sits from the link that the bone will drive.

3. Write a keypoint mapping between the source armature and the target robot. See
   [presets.py](./mikumotion/presets.py) for examples.

4. Run `mikumotion retarget` to remap the motion and solve the robot's joints by IK. The result
   lands beside the export, as `<robot>/<motion>.rrd`.


## Directory Structure

`blender-projects/` holds the Blender project files.

`mikumotion/` holds the Python source of the library.

`scripts/blender/` holds the scripts that Blender runs.

`data/motions/` holds the converted motions.

`data/robots/` holds the robot assets that the IK solve uses.

Note: licensing prevents redistribution of the Blender project files and the MMD motions. Get those
files from their original creators. [This note](./data/motions/MMD-Motion-Sources.md) links to the
original authors' MMD motions. Internal developers can use the
[Google Drive](https://drive.google.com/drive/folders/1sFQmo_UvkY5xSIZKLjXLxlAOpLdI_1jz?usp=drive_link)
mirror of this directory.


## Motion Format

A motion is a Rerun `.rrd` recording. The fields follow IsaacLab's
[MotionLoader](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/direct/humanoid_amp/motions/motion_loader.py#L12).
For `D` joints, `B` links and `F` frames:

- `fps`: an int64 number, the frame rate of the motion data.
- `joint_names`: a list of length `D`, the name of each joint.
- `body_names`: a list of length `B`, the name of each link.
- `joint_positions`: an array of shape `(F, D)`, the rotational positions of the joints, in `rad`.
- `joint_velocities`: an array of shape `(F, D)`, the angular velocities of the joints, in `rad/s`.
- `body_positions`: an array of shape `(F, B, 3)`, each body's location in **world frame**, in `m`.
- `body_rotations`: an array of shape `(F, B, 4)`, each body's rotation in **world frame**, as the
  quaternion `(qw, qx, qy, qz)`.
- `body_linear_velocities`: an array of shape `(F, B, 3)`, each body's linear velocity in
  **world frame**, in `m/s`.
- `body_angular_velocities`: an array of shape `(F, B, 3)`, each body's angular velocity in
  **world frame**, in `rad/s`.

Read the export stage with `MotionStore(root).read_reference_motion(name)`. Pass a robot as well,
`read_reference_motion(name, robot)`, to carry that robot's solved joints on the same sequence. Both
calls return a `MotionSequence` that holds those fields as numpy arrays. The body poses are always
the reference rig's, because that is what a training run tracks. The robot layer keeps the poses the
robot itself reached, beside its joints at `/<robot>/body/poses/<link>`.

Rerun's native types carry the fields. The entity paths group by the thing, and the leaf names the
quantity:

```
/<layer>/body/poses/<name>        Transform3D, one entity per body, one row per frame
/<layer>/body/frames/<name>       a gizmo bound to the frame that pose names, static
/<layer>/body/linear_velocities   Arrows3D    one row per frame, every body an instance
/<layer>/body/angular_velocities  Arrows3D
/<robot>/joint/positions          Scalars     one row per frame, every joint an instance
/<robot>/joint/velocities         Scalars
/<robot>/tf                       Transform3D per joint, plus the root link's placement
/<robot>/tf_static                the URDF's fixed transforms
/<robot>/visual_geometries/...    the URDF's geometry, named by Rerun's URDF loader
```

`<layer>` is `reference` or a robot's name. Both layers store bodies the same way, so one writer and
one reader serve both.

Names follow one rule: **`body` and `joint` name the domain, and anything holding many things is
plural.** The rule covers both a container of per-item entities (`poses`, `frames`) and a leaf that
batches every body or joint of a frame into one row (`linear_velocities`, `positions`). Rerun's own
URDF loader names `visual_geometries` and `collision_geometries` the same way. Those two are not ours
to rename: the Rust loader emits them, and exposes no knob for the names.

Quaternions are stored in Rerun's `xyzw` order, and converted back to `wxyz` on read.

Each pose names a coordinate frame, and a red/green/blue gizmo is bound to that frame. A motion
therefore shows each body's orientation, not only its position, and it costs no per-frame bytes. The
gizmos and a robot layer's URDF geometry share one transform graph, so the viewer can draw the solved
robot with its target frames on top.

Frame names carry the layer, as `reference/body/<name>` and `<robot>/body/<link>`. A robot's own link
frames are `<robot>/<link>`, which the joint chain in `/<robot>/tf` defines. Its body poses must name
something else, because a frame can have only one parent.

**Every entity drawn in 3D must name a frame**, even one that needs no transform. The velocity arrows
are world-frame vectors, and say so with a static `CoordinateFrame`. An entity with no frame is an
orphan. The viewer invents `tf#<entity path>` for it, finds no route to the view's root, and draws an
error in place of the entity. Scalars are exempt, because the viewer plots them against time rather
than placing them in the scene.

**Body names must be unique.** They are entity paths, so two bodies with one name would overwrite
each other. Both writers refuse. Names that need escaping, such as Japanese MMD bones or names
with spaces, pass through `rr.new_entity_path` and survive the round trip.

`fill_body_velocities` always derives the body velocities from the pose trajectory. No producer
computes them alone. The angular part must come from the rotation between frames, because
Euler-angle rates are not an angular velocity, and it must take that rotation the short way round.
A quaternion that flips sign otherwise reads as a near-full turn and spikes at `2*pi*fps`.

### Layout

Each layer is a directory, and each motion is a file in it. This follows the
[Rerun dataset convention](https://huggingface.co/datasets/rerun/arkitscenes-rrd). **A layer is a
pipeline stage.** A file is one motion segment: a dance, an episode, or a take.

```
data/motions/
  reference/  <motion>.rrd  body poses as exported, robot-agnostic
  lite_pro/   <motion>.rrd  what the solve for lite_pro produced
  g1/         <motion>.rrd  the same motion solved for another robot
  blueprints/ <robot>.rbl   viewer layout for that robot
```

`reference/` is the product of the export step. It holds the world-frame body poses and velocities a
solve aims at. It has no joints and no robot model, so anyone can consume it.

Each robot layer is the product of a solve step. It holds that robot's joint positions and
velocities, and the body poses the solve reached. It also holds the robot's URDF geometry and the
transforms that animate it, so the file shows the robot on its own.

Both layers hold body poses, and the two sets are different data rather than a duplicate.
`reference/` is what the IK aims at, and `<robot>/` is what it reached. The gap between them is the
retarget's error, frame by frame. A training run tracks the reference poses, and reads the joints
from a robot layer to reset the robot.

The layer is named `reference`, not `base`, because `base` already means the base link and the
floating base everywhere else in this codebase.

Filenames match across layers. `reference/zamuza.rrd` and `lite_pro/zamuza.rrd` are one segment at
two stages. Every layer shares an `application_id` and a `recording_id`, which is all the viewer needs to
open them as one recording. `rerun data/motions/*/zamuza.rrd` opens the source motion and every robot
solved from it.

A robot layer puts its entities under `/<robot>/`, and prefixes its URDF frames and static
transforms the same way. Two robots, or a robot and the reference rig, would otherwise write the same
entity paths and overwrite each other. Each motion's robot layer carries its own copy of the geometry. A shared file would need a
different `recording_id` and would arrive as a *separate* recording, which Rerun cannot compose
against the motion. [rerun-io/rerun#7316](https://github.com/rerun-io/rerun/issues/7316) tracks
reuse of shared data across recordings.

Match the frame rate to the policy update rate of the training environment. A mismatch forces
expensive interpolation.


### Generic Joint Names

Use the [SMPL-X joint names](https://github.com/vchoutas/smplx/blob/main/smplx/joint_names.py#L244C21-L268C18)
as the generic naming convention.

```
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
```


## Working with MMD

Install the [MMD Tools](https://extensions.blender.org/add-ons/mmd-tools/) add-on in Blender to
import and convert MMD motions.
