# Modified from Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# SPDX-License-Identifier: BSD-3-Clause

import matplotlib
import matplotlib.animation
import matplotlib.pyplot as plt
import numpy as np

from mikumotion.motion_sequence import MotionSequence


class MatplotViewer:
    """
    Helper class to visualize motion data from NumPy-file format.
    """

    def __init__(
        self,
        motion: MotionSequence,
        render_scene: bool = False,
        show_velocity: bool = False,
        show_frames: list[str] = [],
    ) -> None:
        """Load a motion file and initialize the internal variables.

        Args:
            motion_file: Motion file path to load.
            device: The device to which to load the data.
            render_scene: Whether the scene (space occupied by the skeleton during movement)
                is rendered instead of a reduced view of the skeleton.

        Raises:
            AssertionError: If the specified motion file doesn't exist.
        """
        self._figure = None
        self._figure_axes = None
        self._render_scene = render_scene
        self._show_velocity = show_velocity
        self._show_frames = show_frames

        # drawing parameters
        self._velocity_scale = 0.2  # Scale factor for velocity arrows
        self._frame_length = 0.1  # Length of the coordinate frame axes

        # load motions
        self._motion = motion

        self._current_frame = 0

        print("\nBody")
        for i, name in enumerate(self._motion.body_names):
            minimum = np.min(self._motion.body_positions[:, i], axis=0).round(decimals=2)
            maximum = np.max(self._motion.body_positions[:, i], axis=0).round(decimals=2)
            print(f"  |-- [{name}] minimum position: {minimum}, maximum position: {maximum}")

    def _quaternion_to_rotation_matrix(self, q):
        """Convert quaternion (w, x, y, z) to rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*x*x - 2*z*z, 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*x*x - 2*y*y]
        ])

    def _drawing_callback(self, frame: int) -> None:
        """Drawing callback called each frame"""
        # get current motion frame
        # get data
        vertices = self._motion.body_positions[self._current_frame]
        velocities = self._motion.body_linear_velocities[self._current_frame]
        rotations = self._motion.body_rotations[self._current_frame]
        # draw skeleton state
        self._figure_axes.clear()

        # Draw keypoints as dots
        self._figure_axes.scatter(*vertices.T, color="black", depthshade=False)

        # Draw coordinate frames for specified bodies
        for name in self._show_frames:
            idx = self._motion.body_names.index(name)
            frame_pos = vertices[idx]
            quat = rotations[idx]  # (w, x, y, z)

            # Convert quaternion to rotation matrix
            R = self._quaternion_to_rotation_matrix(quat)

            # Define unit vectors for X, Y, Z axes
            x_axis = np.array([1, 0, 0])
            y_axis = np.array([0, 1, 0])
            z_axis = np.array([0, 0, 1])

            # Rotate the axes using the rotation matrix
            x_rotated = R @ x_axis
            y_rotated = R @ y_axis
            z_rotated = R @ z_axis

            # Scale the rotated axes
            x_rotated *= self._frame_length
            y_rotated *= self._frame_length
            z_rotated *= self._frame_length

            # Draw X-axis (red)
            self._figure_axes.quiver(
                frame_pos[0], frame_pos[1], frame_pos[2],
                x_rotated[0], x_rotated[1], x_rotated[2],
                color='red', arrow_length_ratio=0.2)
            # Draw Y-axis (green)
            self._figure_axes.quiver(
                frame_pos[0], frame_pos[1], frame_pos[2],
                y_rotated[0], y_rotated[1], y_rotated[2],
                color='green', arrow_length_ratio=0.2)
            # Draw Z-axis (blue)
            self._figure_axes.quiver(
                frame_pos[0], frame_pos[1], frame_pos[2],
                z_rotated[0], z_rotated[1], z_rotated[2],
                color='blue', arrow_length_ratio=0.2)

        # Draw velocity vectors for all points
        for i, (pos, vel) in enumerate(zip(vertices, velocities)):
            # Only draw velocity if it's not zero
            if np.linalg.norm(vel) > 1e-6:
                self._figure_axes.quiver(
                    pos[0], pos[1], pos[2],
                    vel[0] * self._velocity_scale, vel[1] * self._velocity_scale, vel[2] * self._velocity_scale,
                    color="orange", alpha=0.7, arrow_length_ratio=0.3
                )

        # adjust exes according to motion view
        # - scene
        if self._render_scene:
            # compute axes limits
            minimum = np.min(self._motion.body_positions.reshape(-1, 3), axis=0)
            maximum = np.max(self._motion.body_positions.reshape(-1, 3), axis=0)
            center = 0.5 * (maximum + minimum)
            diff = 0.75 * (maximum - minimum)
        # - skeleton
        else:
            # compute axes limits
            minimum = np.min(vertices, axis=0)
            maximum = np.max(vertices, axis=0)
            center = 0.5 * (maximum + minimum)
            diff = np.array([0.75 * np.max(maximum - minimum).item()] * 3)
        # scale view
        self._figure_axes.set_xlim((center[0] - diff[0], center[0] + diff[0]))
        self._figure_axes.set_ylim((center[1] - diff[1], center[1] + diff[1]))
        self._figure_axes.set_zlim((center[2] - diff[2], center[2] + diff[2]))
        self._figure_axes.set_box_aspect(aspect=diff / diff[0])
        # plot ground plane
        x, y = np.meshgrid([center[0] - diff[0], center[0] + diff[0]], [center[1] - diff[1], center[1] + diff[1]])
        self._figure_axes.plot_surface(x, y, np.zeros_like(x), color="green", alpha=0.2)
        # print metadata
        self._figure_axes.set_xlabel("X")
        self._figure_axes.set_ylabel("Y")
        self._figure_axes.set_zlabel("Z")
        self._figure_axes.set_title(f"frame: {self._current_frame}/{self._motion.num_frames}")
        # increase frame counter
        self._current_frame += 1
        if self._current_frame >= self._motion.num_frames:
            self._current_frame = 0

    def show(self) -> None:
        """Show motion"""
        # create a 3D figure
        self._figure = plt.figure()
        self._figure_axes = self._figure.add_subplot(projection="3d")
        # matplotlib animation (the instance must live as long as the animation will run)
        self._animation = matplotlib.animation.FuncAnimation(
            fig=self._figure,
            func=self._drawing_callback,
            frames=self._motion.num_frames,
            interval=1000 * self._motion.dt,
        )
        plt.show()
