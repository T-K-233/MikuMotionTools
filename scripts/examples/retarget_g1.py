import argparse

import numpy as np
import mujoco
import mujoco.viewer
from loop_rate_limiters import RateLimiter
import mink

from mikumotion.motion_sequence import MotionSequence


parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str)
parser.add_argument("--realtime", action="store_true", help="Visualize in realtime")
args = parser.parse_args()


def main():
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

    motion = MotionSequence.load(args.file)

    pose_indices = motion.get_body_index(pose_names)
    pole_indices = motion.get_body_index(pole_names)

    unitree_g1_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5_mocap.xml"

    model = mujoco.MjModel.from_xml_path(unitree_g1_xml)
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

    # write the dof names to the motion data
    motion._dof_names = [model.joint(1 + i).name for i in range(model.nu)]
    # override the dof positions with correct dimension
    motion._dof_positions = np.zeros((motion.num_frames, model.nu), dtype=np.float32)

    viewer = mujoco.viewer.launch_passive(
        model=model, data=data  #, show_left_ui=False, show_right_ui=False
    )
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    # Initialize to the home keyframe.
    for task in tasks:
        task.set_target_from_configuration(configuration)

    if args.realtime:
        rate = RateLimiter(frequency=motion.fps[0], warn=False)

    for frame_idx in range(motion.num_frames):
        # Update task targets.
        for i in range(len(pose_tasks)):
            pose_tasks[i].set_target(mink.SE3(
                wxyz_xyz=np.concatenate([
                    motion.body_rotations[frame_idx, pose_indices[i]],
                    motion.body_positions[frame_idx, pose_indices[i]]
                ])
            ))

        for i in range(len(pole_tasks)):
            pole_tasks[i].set_target(mink.SE3.from_translation(
                translation=motion.body_positions[frame_idx, pole_indices[i]]
            ))

        for i, frame_name in enumerate(pose_names):
            # move the current mocap to the frame
            mink.move_mocap_to_frame(model, data, f"{frame_name}_current", frame_name, "body")
            # move the target mocap to the frame
            mocap_id = model.body(f"{frame_name}_target").mocapid[0]
            data.mocap_pos[mocap_id] = motion.body_positions[frame_idx, pose_indices[i]]
            data.mocap_quat[mocap_id] = motion.body_rotations[frame_idx, pose_indices[i]]

        for i, frame_name in enumerate(pole_names):
            # move the current mocap to the frame
            mink.move_mocap_to_frame(model, data, f"{frame_name}_current", frame_name, "body")
            # move the target mocap to the frame
            mocap_id = model.body(f"{frame_name}_target").mocapid[0]
            data.mocap_pos[mocap_id] = motion.body_positions[frame_idx, pole_indices[i]]

        vel = mink.solve_ik(
            configuration, tasks, motion.dt, solver, 1e-1, limits=limits
        )
        configuration.integrate_inplace(vel, motion.dt)

        motion._dof_positions[frame_idx, :] = data.qpos[7:]

        # Visualize at fixed FPS.
        viewer.sync()
        if args.realtime:
            rate.sleep()

    viewer.close()

    motion_file_out = args.file.replace(".npz", "_retargeted.npz")
    motion.save(motion_file_out)
    print(f"Results saved to {motion_file_out}")


if __name__ == "__main__":
    main()
