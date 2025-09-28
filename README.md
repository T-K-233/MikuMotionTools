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

1. Create a Blender project and import the source motion.

2. Create a keypoint map between the source motion and target motion. Some examples are in [presets.py](./mikumotion/presets.py).

3. Create a Python script to read the key frame position from the Blender project, and perform body retargeting. This will extract all the keypoint body positions and rotations, perform the remapping translation and rotation, and finally also calculate the body linear and rotation velocities.

4. Use [compute_dof_ik.py](./scripts/compute_dof_ik.py) to perform IK solving and get the joint positions. This script will also move the body frames to the solved FK positions.


## Running examples

Export motion from Blender.

This script only exports the body pose data. The joint data will be empty, and need to be filled in with the following retargeting script.

```bash
blender ./blender-projects/G1_Zamuza.blend --python ./scripts/examples/retarget_g1_zamuza.py
```

View motion with matplotlib:

```bash
uv run ./scripts/view_motion.py --motion ./data/motions/g1_zamuza_0_1632.npz
```

Run retargeting logic to solve for joint data.

```bash
uv run ./scripts/examples/retarget_g1.py --motion ./data/motions/g1_zamuza_0_1632.npz --robot unitree_g1 --realtime
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
