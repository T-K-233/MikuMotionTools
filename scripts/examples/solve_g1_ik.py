import time

import numpy as np
import mujoco
import mujoco.viewer
from loop_rate_limiters import RateLimiter

import mink

_XML = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5_mocap.xml"


def get_dof_index(motion, dof_names: list[str]) -> list[int]:
    indexes = []
    for name in dof_names:
        assert name in motion["dof_names"], f"The specified DOF name ({name}) doesn't exist: {motion['dof_names']}"
        indexes.append(motion["dof_names"].index(name))
    return indexes


def get_body_index(motion, body_names: list[str]) -> list[int]:
    indexes = []
    for name in body_names:
        assert name in motion["body_names"], f"The specified body name ({name}) doesn't exist: {motion['body_names']}"
        indexes.append(motion["body_names"].index(name))
    return indexes


if __name__ == "__main__":
    realtime = False
    pose_names = [
        "pelvis",
        "left_rubber_hand", "right_rubber_hand",
        "left_ankle_roll_link", "right_ankle_roll_link",
    ]
    pole_names = [
        "left_shoulder_roll_link", "left_elbow_link",
        "right_shoulder_roll_link", "right_elbow_link",
        "left_hip_roll_link", "right_hip_roll_link",
        "left_knee_link", "right_knee_link",
    ]

    motion_file = "./data/motions/g1_zamuza_0_960.npz"
    motion = dict(np.load(motion_file))
    motion["dof_names"] = motion["dof_names"].tolist()
    motion["body_names"] = motion["body_names"].tolist()

    pose_indices = get_body_index(motion, pose_names)
    pole_indices = get_body_index(motion, pole_names)

    model = mujoco.MjModel.from_xml_path(_XML)
    configuration = mink.Configuration(model)

    pose_tasks = [
        mink.FrameTask(
            frame_name=name,
            frame_type="body",
            position_cost=2.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        ) for name in pose_names]
    pole_tasks = [
        mink.FrameTask(
            frame_name=name,
            frame_type="body",
            position_cost=1.0,
            orientation_cost=0.0,
            lm_damping=1.0,
        ) for name in pole_names]

    posture_task = mink.PostureTask(model, cost=1e-1)

    tasks = [
        posture_task,
        *pose_tasks,
        *pole_tasks,
    ]

    limits = [
        mink.ConfigurationLimit(model),
    ]

    model = configuration.model
    data = configuration.data
    solver = "daqp"

    num_frames = motion["body_positions"].shape[0]

    motion["dof_positions"] = np.zeros((num_frames, model.nu), dtype=np.float32)

    joint_names = [model.joint(1 + i).name for i in range(model.nu)]
    motion["dof_names"] = joint_names

    with mujoco.viewer.launch_passive(
        model=model, data=data  #, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize to the home keyframe.
        for task in tasks:
            task.set_target_from_configuration(configuration)

        rate = RateLimiter(frequency=int(motion["fps"][0]), warn=False)
        # while viewer.is_running():
        for frame_idx in range(num_frames):

            # Update task targets.
            pose_positions = motion["body_positions"][frame_idx, pose_indices]
            pose_rotations = motion["body_rotations"][frame_idx, pose_indices]
            pole_positions = motion["body_positions"][frame_idx, pole_indices]

            for pose_idx in range(len(pose_tasks)):
                pose_tasks[pose_idx].set_target(mink.SE3(wxyz_xyz=np.concatenate([
                    pose_rotations[pose_idx],
                    pose_positions[pose_idx]])))

            for pole_idx in range(len(pole_tasks)):
                pole_tasks[pole_idx].set_target(mink.SE3.from_translation(
                    translation=pole_positions[pole_idx]))

            for name in pose_names:
                mink.move_mocap_to_frame(model, data, f"{name}_target", name, "body")
            for name in pole_names:
                mink.move_mocap_to_frame(model, data, f"{name}_target", name, "body")

            vel = mink.solve_ik(
                configuration, tasks, rate.dt, solver, 1e-1, limits=limits
            )
            configuration.integrate_inplace(vel, rate.dt)

            motion["dof_positions"][frame_idx, :] = data.qpos[7:]

            # Visualize at fixed FPS.
            viewer.sync()
            if realtime:
                rate.sleep()

    np.savez(motion_file, **motion)
