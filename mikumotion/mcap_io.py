"""
Read a robot-motion log from an ``.mcap`` file into plain numpy arrays.

The logs produced by the Lite-Pro tracking pipeline are **ROS2 (CDR-encoded)** MCAP
files with two synchronized channels sampled at a fixed rate:

- ``/odom``          — ``nav_msgs/msg/Odometry``    : floating-base pose (+ twist) in ``world``.
- ``/joint_states``  — ``sensor_msgs/msg/JointState``: per-joint angles (+ velocities).

``read_motion_mcap`` decodes both channels, aligns them by timestamp, converts the
base orientation from ROS ``xyzw`` to the ``wxyz`` convention used everywhere else in
this codebase, and returns everything as dense numpy arrays. It is deliberately
free of any ``mujoco``/``bpy`` dependency so it can run anywhere.

Requires ``mcap`` and ``mcap-ros2-support`` (declared in ``pyproject.toml``).
"""

from __future__ import annotations

from typing import Dict

import numpy as np

# topic / schema names expected in the log
ODOM_TOPIC = "/odom"
JOINT_TOPIC = "/joint_states"


def read_motion_mcap(path: str, *, odom_topic: str = ODOM_TOPIC, joint_topic: str = JOINT_TOPIC) -> Dict:
    """Read a ROS2 motion ``.mcap`` into aligned numpy arrays.

    Args:
        path: Path to the ``.mcap`` file.
        odom_topic: Topic carrying ``nav_msgs/Odometry`` base pose.
        joint_topic: Topic carrying ``sensor_msgs/JointState`` joint angles.

    Returns:
        A dict with keys:
            ``joint_names``            : list[str], length D (joint order as logged).
            ``fps``                    : int, inferred from the median sample period.
            ``times``                  : (N,) float seconds, starting at 0.
            ``base_positions``         : (N, 3) float, base position in world.
            ``base_quaternions``       : (N, 4) float, base orientation as (w, x, y, z).
            ``joint_positions``        : (N, D) float, joint angles [rad].
            ``joint_velocities``       : (N, D) float, joint velocities [rad/s].
            ``base_linear_velocities`` : (N, 3) float, base twist linear [m/s].
            ``base_angular_velocities``: (N, 3) float, base twist angular [rad/s].

    Raises:
        ImportError: If ``mcap-ros2-support`` is not installed.
        ValueError: If neither expected topic carries any messages.
    """
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as e:  # pragma: no cover - dependency guard
        raise ImportError(
            "reading ROS2 .mcap logs requires 'mcap-ros2-support' "
            "(add it to your environment: `uv pip install mcap-ros2-support`)."
        ) from e

    # bucket messages per topic, keyed by log_time (ns) so we can align the two streams
    odom: Dict[int, dict] = {}
    joints: Dict[int, dict] = {}
    joint_names: list[str] | None = None

    for msg in read_ros2_messages(path):
        topic = msg.channel.topic
        ros = msg.ros_msg
        t = int(msg.log_time_ns)  # raw log time in nanoseconds

        if topic == odom_topic:
            p = ros.pose.pose.position
            q = ros.pose.pose.orientation  # ROS order: x, y, z, w
            tl = ros.twist.twist.linear
            ta = ros.twist.twist.angular
            odom[t] = {
                "pos": (p.x, p.y, p.z),
                "quat_wxyz": (q.w, q.x, q.y, q.z),  # -> w, x, y, z
                "lin": (tl.x, tl.y, tl.z),
                "ang": (ta.x, ta.y, ta.z),
            }
        elif topic == joint_topic:
            if joint_names is None:
                joint_names = list(ros.name)
            joints[t] = {
                "position": np.asarray(ros.position, dtype=np.float64),
                "velocity": np.asarray(ros.velocity, dtype=np.float64) if len(ros.velocity) else None,
            }

    if not odom or not joints:
        raise ValueError(
            f"expected messages on '{odom_topic}' and '{joint_topic}', "
            f"found {len(odom)} odom / {len(joints)} joint_states"
        )
    if joint_names is None:
        raise ValueError("no joint names found in joint_states messages")

    # align the two streams on their common timestamps
    common = sorted(set(odom) & set(joints))
    if not common:
        raise ValueError("odom and joint_states share no common timestamps")
    dropped = (len(odom) - len(common)) + (len(joints) - len(common))
    if dropped:
        print(f"[mcap_io] WARNING: {dropped} unmatched messages dropped during alignment")

    n = len(common)
    d = len(joint_names)

    times = np.array(common, dtype=np.float64)
    times = (times - times[0]) / 1e9  # ns -> s, start at 0

    base_positions = np.zeros((n, 3), dtype=np.float64)
    base_quaternions = np.zeros((n, 4), dtype=np.float64)
    base_linear_velocities = np.zeros((n, 3), dtype=np.float64)
    base_angular_velocities = np.zeros((n, 3), dtype=np.float64)
    joint_positions = np.zeros((n, d), dtype=np.float64)
    joint_velocities = np.zeros((n, d), dtype=np.float64)

    for i, t in enumerate(common):
        o = odom[t]
        j = joints[t]
        base_positions[i] = o["pos"]
        base_quaternions[i] = o["quat_wxyz"]
        base_linear_velocities[i] = o["lin"]
        base_angular_velocities[i] = o["ang"]
        joint_positions[i] = j["position"]
        if j["velocity"] is not None:
            joint_velocities[i] = j["velocity"]

    # normalize base quaternions defensively
    norms = np.linalg.norm(base_quaternions, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    base_quaternions /= norms

    # infer fps from the median sample period
    if n > 1:
        dt = float(np.median(np.diff(times)))
        fps = int(round(1.0 / dt)) if dt > 0 else 50
    else:
        fps = 50

    print(
        f"[mcap_io] {path}: {n} frames, {d} joints, ~{fps} fps, "
        f"{times[-1]:.2f}s"
    )

    return {
        "joint_names": joint_names,
        "fps": fps,
        "times": times,
        "base_positions": base_positions,
        "base_quaternions": base_quaternions,
        "joint_positions": joint_positions,
        "joint_velocities": joint_velocities,
        "base_linear_velocities": base_linear_velocities,
        "base_angular_velocities": base_angular_velocities,
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Inspect a motion .mcap file.")
    parser.add_argument("--mcap", type=str, required=True)
    args = parser.parse_args()

    data = read_motion_mcap(args.mcap)
    print("joint_names:", data["joint_names"])
    print("base_positions[0]:", data["base_positions"][0])
    print("base_quaternions[0] (wxyz):", data["base_quaternions"][0])
    print("joint_positions[0][:6]:", data["joint_positions"][0][:6])
