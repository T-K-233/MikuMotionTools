# MikuMotionTools

MikuMotionTools is a motion-data conversion library for humanoid robot RL. It sits between *animation-world* motion (MMD/MikuMikuDance dances, mocap, Blender rigs) and *robotics-world* motion (a robot's joint angles and link poses), and translates in both directions:

- **animation → robot** — produce reference motions for RL training from Blender armature animation.
- **robot → animation** — render what a trained policy actually did as a character animation.

Everything is organised around one central data structure, a [Rerun](https://rerun.io) `.rrd` motion file; every other module is a converter into or out of it. Storing motions as `.rrd` means they can live on a server, be previewed by streaming down only the selected recording, and be consumed by a training environment that reads only the columns it needs.


## Getting Started

First, clone the repository from Github

```bash
git clone https://github.com/T-K-233/MikuMotionTools.git
cd ./MikuMotionTools/
uv pip install -e .
```

and install the dependencies

```bash
uv sync
```

To install extra dependencies that is required by the example code, do

```bash
uv sync --extra examples
```


## The two directions

Conversions are named **`a_to_b`**, so a name reads as the transformation it performs and inverses pair up on sight: `armature_to_motion` against `motion_to_armature`. `from_` is reserved for constructors that build the thing they are called on — `RobotModel.from_file(path)` — and methods that convert what they belong to use `to_`, as in `robot.to_armature_tree()`.

The two pipelines mirror each other step for step:

| Step | animation → robot | robot → animation |
|---|---|---|
| 1. read the source | *(Blender opens the .blend)* | `mcap_io.read_robot_log` |
| 2. **into the hub** | `blender.armature_to_motion` | `forward_kinematics.robot_log_to_motion` |
| 3. store | `MotionStore.write_motion` | `MotionStore.write_motion` |
| 4. retarget | `MotionRetargetingIK` — IK solves the robot's joints | `blender.retarget_armature` — bakes the robot rig onto a character rig |
| 5. **out of the hub** | *(the robot's joints are the product)* | `blender.motion_to_armature` |
| entry point | `scripts/blender/export_mocap.py` | `scripts/blender/animate_robot.py`, `retarget_to_vrm.py` |

Reading it across a row tells you which step corresponds to which; reading down a column is one pipeline. Step 4 is where the directions genuinely differ: going *to* a robot needs IK, because a character's limb proportions do not map onto a robot's joints, while coming *from* a robot only needs a rotation transfer.


## Commands

Motions are addressed by **name**, not by path; the store layout decides where each layer lives.

```bash
mikumotion import <log.mcap> <robot.xml> <robot.urdf>   # robot log -> motion
mikumotion view <name>                                  # watch it in the Rerun viewer
mikumotion retarget <name> <robot.xml>                  # solve a robot's joints for a motion
mikumotion list                                         # what is in the store
```

The Blender-side scripts are run by Blender itself. They read and write the same store, which needs `rerun-sdk` inside Blender's bundled Python:

```bash
<blender>/python/bin/python.exe -m pip install rerun-sdk
```

```bash
# animation -> robot: export a mocap armature as a motion
blender ./blender-projects/Zamuza.blend --python ./scripts/blender/export_mocap.py -- zamuza

# robot -> animation: play a motion on the robot's own rig
blender --background --python ./scripts/blender/animate_robot.py -- lite_pro_tracking <robot.urdf>

# robot -> animation: replay it on a VRM character
blender ./character.blend --background --python ./scripts/blender/retarget_to_vrm.py -- lite_pro_tracking <robot.urdf>
```


## General Workflow

1. Create a Blender project and import the source motion. Adjust the animation in Blender to match the target policy frequency (typically 50 Hz).

2. Export the source armature's key frame positions and orientations with `export_mocap.py`. This writes a motion labelled with the source bone names and no joint data yet.

3. Create a keypoint mapping configuration between the source motion armature and target robot. Some examples are in [presets.py](./mikumotion/presets.py).

4. Run `mikumotion retarget` to remap and IK-solve for the robot's joints, then write the result as a new motion.


## Directory Structure

`blender-projects/` stores the blender project files. 

`mikumotion/` stores the Python source file of the library.

`scripts/blender/` stores the scripts that Blender itself runs.

`data/motions/` stores the converted motions.

`data/robots/` stores the robot asset file used during inverse kinematic solving.

Note: Due to licensing restrictions, the Blender project files and MMD motions cannot be redistributed here. To access them, please obtain the files directly from their original creators. For your convenience, we’ve included links to the original authors’ MMD motions in [this note](./data/motions/MMD-Motion-Sources.md). For internal developers, the mirror of this directory is stored at [Google Drive](https://drive.google.com/drive/folders/1sFQmo_UvkY5xSIZKLjXLxlAOpLdI_1jz?usp=drive_link).


## Motion Format

A motion is a Rerun `.rrd` recording. The fields follow IsaacLab's [MotionLoader](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/direct/humanoid_amp/motions/motion_loader.py#L12); assuming `D` joints, `B` links and `F` frames:

- `fps`: an int64 number representing the frame rate of the motion data.
- `joint_names`: a list of length `D` containing the names of each joint.
- `body_names`: a list of length `B` containing the names of each link.
- `joint_positions`: a numpy array of shape `(F, D)` containing the rotational positions of the joints in `rad`.
- `joint_velocities`: a numpy array of shape `(F, D)` containing the rotational (angular) velocities of the joints in `rad/s`.
- `body_positions`: a numpy array of shape `(F, B, 3)` containing the locations of each body in **world frame**, in `m`.
- `body_rotations`: a numpy array of shape `(F, B, 4)` containing the rotations of each body in **world frame**, in quaternion `(qw, qx, qy, qz)`.
- `body_linear_velocities`: a numpy array of shape `(F, B, 3)` containing the linear velocities of each body in **world frame**, in `m/s`.
- `body_angular_velocities`: a numpy array of shape `(F, B, 3)` containing the rotational (angular) velocities of each body in **world frame**, in `rad/s`.

Read one with `MotionStore(root).read_motion(name)`, which returns a `MotionSequence` holding exactly those fields as numpy arrays.

Where Rerun has a native representation, the fields use it: a body's position and rotation are one `Transform3D`, velocities are `Arrows3D`, and joint angles are `Scalars` so they plot on the timeline. Quaternions are stored in Rerun's `xyzw` order and converted back to `wxyz` on read.

### Layout

Following the Rerun dataset convention, each layer is a directory and each motion a file within it:

```
data/motions/
  base/       <name>.rrd   the motion sequence — this is what training consumes
  preview/    <name>.rrd   joint transforms that animate the robot in the viewer
  robot/      <robot>.rrd  URDF geometry + static transforms, shared by every motion
  blueprints/ <robot>.rbl  viewer layout
```

`base/` stands alone and needs no robot model, so a motion exported from a mocap armature is storable before any robot exists. The other layers exist only so a human can watch the motion, which is why the geometry is stored once per robot rather than once per motion. Layers share an `application_id` and `recording_id`, so `mikumotion view` merges them into a single recording.

The converted motion file is targeted for one particular robot skeleton structure.

To ensure best performance, also make sure that the frame rate matches the training environment policy update rate to avoid expensive interpolations.


### Generic Joint Names

We follow the [SMPL-X joint name](https://github.com/vchoutas/smplx/blob/main/smplx/joint_names.py#L244C21-L268C18) as a generic joint naming convention.

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

To import and convert MMD motions in Blender, the [MMD Tools](https://extensions.blender.org/add-ons/mmd-tools/) plugin needs to be installed to Blender.
