# MikuMotionTools

MikuMotionTools contains various functions for converting MMD (MikuMikuDance) motions and other motion file formats into armature motion format that can be used in the Isaac Lab RL training environment.


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


## General Workflow

The general workflow of retargeting is listed as follows:

1. Create a Blender project and import the source motion. Adjust the animation in Blender to match the target policy frequency (typically 50 Hz).

2. Create a Python script to read the key frame position and orientations of the source armature motion in the Blender project. This will generate a MotionSequence labeled with source motion bone names.

3. Create a keypoint mapping configuration between the source motion armature and target motion armature. Some examples are in [presets.py](./mikumotion/presets.py).

4. Use [run_retargeting.py](./scripts/run_retargeting.py) to perform motion retargeting. This will first read from the source MotionSeuqence, perform the remapping translation, rotation, and scaling according to the mapping config, perform IK solving and get the joint positions, update the body pose and velocities according to the solved FK solutions, and write the result as a new MotionSequence file.


## Running examples

Export motion from Blender.

This script only exports the body pose data. The joint data will be empty, and need to be filled in with the following retargeting script.

```bash
blender ./blender-projects/Zamuza.blend --python ./scripts/examples/export_zamuza.py
```

View motion with matplotlib:

```bash
uv run ./scripts/view_motion.py --motion ./data/motions/zamuza_0_1632.npz
```

Run retargeting logic to solve for joint data.

```bash
uv run ./scripts/run_retargeting.py --motion ./data/motions/zamuza_0_1632.npz --mapping MMD_YYB_TO_G1_CFG --real-time
```

When adding new mapping config, the following script might be helpful:

```bash
# export the reset pose of target robot
blender ./blender-projects/G1-USD.blend --python ./scripts/examples/export_g1_reset_pose.py

# compare the frames between source motion and target robot armature
uv run ./scripts/compare_frames.py --source ./data/motions/actorcore_reset.npz --target ./data/motions/g1_reset_pose.npz --mapping ACTORCORE_TO_G1_CFG
```


## Directory Structure

`blender-projects/` stores the blender project files. 

`mikumotion/` stores the Python source file of the library.

`data/motions/` stores the converted motions.

`data/robots/` stores the robot asset file used during inverse kinematic solving.

Note: Due to licensing restrictions, the Blender project files and MMD motions cannot be redistributed here. To access them, please obtain the files directly from their original creators. For your convenience, we’ve included links to the original authors’ MMD motions in [this note](./data/motions/MMD-Motion-Sources.md). For internal developers, the mirror of this directory is stored at [Google Drive](https://drive.google.com/drive/folders/1sFQmo_UvkY5xSIZKLjXLxlAOpLdI_1jz?usp=drive_link).


## Motion Format

This library uses the motion file format defined in IsaacLab [MotionLoader](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/direct/humanoid_amp/motions/motion_loader.py#L12).

Each motion file is a numpy dictionary with the following fields. Here, we assume the robot has `D` number of joints and `B` number of linkages, and the motion file has `F` frames.

- `fps`: an int64 number representing the frame rate of the motion data.
- `dof_names`: a list of length `D` containing the names of each joint.
- `body_names`: a list of length `B` containing the names of each link.
- `dof_positions`: a numpy array of shape `(F, D)` containing the rotational positions of the joints in `rad`.
- `dof_velocities`: a numpy array of shape `(F, D)` containing the rotational (angular) velocities of the joints in `rad/s`.
- `body_positions`: a numpy array of shape `(F, B, 3)` containing the locations of each body in **world frame**, in `m`.
- `body_rotations`: a numpy array of shape `(F, B, 4)` containing the rotations of each body in **world frame**, in quaternion `(qw, qx, qy, qz)`.
- `body_linear_velocities`: a numpy array of shape `(F, B, 3)` containing the linear velocities of each body in **world frame**, in `m/s`.
- `body_angular_velocities`: a numpy array of shape `(F, B, 3)` containing the rotational (angular) velocities of each body in **world frame**, in `rad/s`.

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
