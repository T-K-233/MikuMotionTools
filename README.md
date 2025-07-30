# MikuMotionTools

MikuMotionTools contains various functions for converting MMD (MikuMikuDance) motions and other motion file formats into armature motion format that can be used in the Isaac Lab RL training environment.


## Getting Started

### Create the environment

```bash
uv sync
```

To run the examples, do

```bash
uv pip install -e .
```

### Install the library

From pip (coming soon)

```bash
uv pip install mikumotion
```

From source
```bash
git clone https://github.com/T-K-233/MikuMotionTools.git
cd ./MikuMotionTools/
uv pip install -e .
```


## Directory Structure

`blender-projects/` stores the blender project files. 

`mikumotion/` stores the Python source file of the library.

`data/motions/` stores the converted motions.

`data/robots/` stores the robot asset file used during inverse kinematic solving.

Note: Due to licensing issue, the Blender project files and MMD motions should be retrieved from the original authors. For internal developers, the mirror of this directory is stored at [Google Drive]().


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


## Working with MMD

To import and convert MMD motions in Blender, the [MMD Tools](https://extensions.blender.org/add-ons/mmd-tools/) plugin needs to be installed to Blender.
