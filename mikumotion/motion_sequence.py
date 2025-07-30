# Modified from Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# SPDX-License-Identifier: BSD-3-Clause

import os

import numpy as np


class MotionSequence:
    """
    A sequence of motion data.

    Modified from https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/direct/humanoid_amp/motions/motion_loader.py.

    This class is modified to use numpy instead of torch to load the motion data, suitable for CPU-only environment.
    """

    @classmethod
    def load(cls, path: str) -> "MotionSequence":
        """Load a motion file and return a MotionSequence object.

        Args:
            path: Motion file path to load.

        Raises:
            AssertionError: If the specified motion file doesn't exist.
        """
        assert os.path.isfile(path), f"Invalid file path: {path}"
        data = np.load(path)

        num_frames = np.max([data["dof_positions"].shape[0], data["body_positions"].shape[0]])

        motion = cls(
            num_frames=num_frames,
            dof_names=data["dof_names"].tolist(),
            body_names=data["body_names"].tolist(),
            fps=data["fps"],
        )

        motion._dof_positions[:] = data["dof_positions"]     # joint positions, rad
        motion._dof_velocities[:] = data["dof_velocities"]   # joint velocities, rad/s
        motion._body_positions[:] = data["body_positions"]   # link positions, m
        motion._body_rotations[:] = data["body_rotations"]   # link rotations, (qw, qx, qy, qz) quaternion
        motion._body_linear_velocities[:] = data["body_linear_velocities"]       # link linear velocities, m/s
        motion._body_angular_velocities[:] = data["body_angular_velocities"]     # link angular velocities, rad/s

        print(f"Motion loaded ({path}): duration: {motion._duration} sec, # of frames: {motion._num_frames}, FPS: {motion._fps}")

        return motion

    def __init__(self, num_frames: int, dof_names: list[str], body_names: list[str], fps: int = 50) -> None:
        """Initialize a MotionSequence object.
        
        Args:
            num_frames: Number of frames.
            dof_names: List of joint names.
            body_names: List of rigid body names.
            fps: the FPS of the motion data.
        """
        self._fps = fps                                         # frame rate
        self._dt = 1.0 / self._fps                              # per frame time step, s
        self._num_frames = num_frames                           # total number of frames
        self._duration = self._dt * (self._num_frames - 1)      # duration, s
        self._dof_names = dof_names                             # joint names
        self._body_names = body_names                           # rigid body names

        # joint positions, rad
        self._dof_positions = np.zeros((self._num_frames, len(self._dof_names)), dtype=np.float32)
        # joint velocities, rad/s
        self._dof_velocities = np.zeros((self._num_frames, len(self._dof_names)), dtype=np.float32)
        # link positions, m
        self._body_positions = np.zeros((self._num_frames, len(self._body_names), 3), dtype=np.float32)
        # link rotations, (qw, qx, qy, qz) quaternion
        self._body_rotations = np.zeros((self._num_frames, len(self._body_names), 4), dtype=np.float32)
        # link linear velocities, m/s
        self._body_linear_velocities = np.zeros((self._num_frames, len(self._body_names), 3), dtype=np.float32)
        # link angular velocities, rad/s
        self._body_angular_velocities = np.zeros((self._num_frames, len(self._body_names), 3), dtype=np.float32)

        # ensure correct default quaternion representation
        self._body_rotations[:, :, 0] = 1.0

    @property
    def fps(self) -> int:
        """Frames per second."""
        return self._fps

    @property
    def dt(self) -> float:
        """Time step."""
        return self._dt

    @property
    def num_frames(self) -> int:
        """Number of frames."""
        return self._num_frames

    @property
    def duration(self) -> float:
        """Duration."""
        return self._duration

    @property
    def dof_names(self) -> list[str]:
        """Joint names."""
        return self._dof_names

    @property
    def body_names(self) -> list[str]:
        """Rigid body names."""
        return self._body_names

    @property
    def num_dofs(self) -> int:
        """Number of joints."""
        return len(self._dof_names)

    @property
    def num_bodies(self) -> int:
        """Number of rigid bodies."""
        return len(self._body_names)

    @property
    def dof_positions(self) -> np.ndarray:
        """Joint positions."""
        return self._dof_positions

    @property
    def dof_velocities(self) -> np.ndarray:
        """Joint velocities."""
        return self._dof_velocities

    @property
    def body_positions(self) -> np.ndarray:
        """Rigid body positions."""
        return self._body_positions

    @property
    def body_rotations(self) -> np.ndarray:
        """Rigid body rotations."""
        return self._body_rotations

    @property
    def body_linear_velocities(self) -> np.ndarray:
        """Rigid body linear velocities."""
        return self._body_linear_velocities

    @property
    def body_angular_velocities(self) -> np.ndarray:
        """Rigid body angular velocities."""
        return self._body_angular_velocities

    def get_dof_index(self, dof_names: list[str]) -> list[int]:
        """Get joint indexes by joint names.

        Args:
            dof_names: List of joint names.

        Raises:
            AssertionError: If the specified joint name doesn't exist.

        Returns:
            List of joint indexes.
        """
        indexes = []
        for name in dof_names:
            assert name in self._dof_names, f"The specified joint name ({name}) doesn't exist in {self._dof_names}"
            indexes.append(self._dof_names.index(name))
        return indexes

    def get_body_index(self, body_names: list[str]) -> list[int]:
        """Get rigid body indexes by rigid body names.

        Args:
            body_names: List of rigid body names.

        Raises:
            AssertionError: If the specified rigid body name doesn't exist.

        Returns:
            List of rigid body indexes.
        """
        indexes = []
        for name in body_names:
            assert name in self._body_names, f"The specified body name ({name}) doesn't exist in {self._body_names}"
            indexes.append(self._body_names.index(name))
        return indexes

    def get_frames(
        self,
        frames: int | list[int] | np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get motion data by frame indices.
        This function will automatically clip the frame indices to the valid range. If the frame indices are out of range,
        the function will return the data of last frame (as upper bound) or first frame (as lower bound).

        Args:
            frames: List of frame indices to sample.

        Returns:
            Motion joint positions (with shape (N, num_dofs)), joint velocities (with shape (N, num_dofs)),
            rigid body positions (with shape (N, num_bodies, 3)), rigid body rotations (with shape (N, num_bodies, 4), as wxyz quaternion),
            rigid body linear velocities (with shape (N, num_bodies, 3)) and rigid body angular velocities (with shape (N, num_bodies, 3)).
        """
        frames = np.clip(frames, 0, self._num_frames - 1)

        return (
            self._dof_positions[frames],
            self._dof_velocities[frames],
            self._body_positions[frames],
            self._body_rotations[frames],
            self._body_linear_velocities[frames],
            self._body_angular_velocities[frames],
        )

    def save(self, path: str) -> None:
        """Save the motion data to a file.
        
        Args:
            path: The path to the output file.
        """
        fps = np.array([self._fps], dtype=np.int64)  # this needs to be int64
        dof_names = np.array(self._dof_names)
        body_names = np.array(self._body_names)

        np.savez(path,
            fps=fps,
            dof_names=dof_names,
            body_names=body_names,
            dof_positions=self._dof_positions,
            dof_velocities=self._dof_velocities,
            body_positions=self._body_positions,
            body_rotations=self._body_rotations,
            body_linear_velocities=self._body_linear_velocities,
            body_angular_velocities=self._body_angular_velocities,
        )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, required=True, help="Motion file")
    args, _ = parser.parse_known_args()

    motion = MotionSequence(args.file)

    print("- number of frames:", motion.num_frames)
    print("- number of DOFs:", motion.num_dofs)
    print("- number of bodies:", motion.num_bodies)
