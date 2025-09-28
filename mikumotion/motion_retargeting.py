from loop_rate_limiters import RateLimiter
import numpy as np
import mink
import mujoco
import mujoco.viewer


from mikumotion.math import quat_mul
from mikumotion.motion_sequence import MotionSequence
from mikumotion.mujoco_utils import add_body_frames


class MotionRetargeting:
    """
    The MotionRetargeting logic.

    It takes in a mapping table, a source motion file, and a target model file.
    It then retargets the source motion to the target model.

    The mapping table is a dictionary that maps the target body names to the source body
    information, containing the following keys:
        - body: the name of the body in the source model
        - weight: the weight / cost of each body in the IK solver, containing:
            - position: the position weight
            - orientation: the orientation weight
        - offset: the offset from the source body to the target body in the world frame, containing:
            - position: the position offset in (x, y, z) in meters
            - orientation: the orientation offset in (w, x, y, z) quaternion

    An example mapping table is shown below:
    ```python
    {
        "pelvis": {
            "source": "腰",
            "target": "pelvis_link",
            "offset": {
                "position": (0.0, 0.0, 0.0),
                "orientation": (1.0, 0.0, 0.0, 0.0),
            },
            "weight": {
                "position": 10.0,
                "orientation": 1.0,
            },
        },
        ...
    }
    ```

    Args:
        motion_file: The source motion file.
        robot_xml: The robot XML file used for IK solving.
        mapping_table: The mapping table between the source and target bodies.
        solver: The solver to use for the inverse kinematics.
        damping: The damping factor for the inverse kinematics.
        max_iter: The maximum number of iterations for the inverse kinematics.
    """
    def __init__(
        self,
        motion_file: str,
        robot_xml: str,
        config: dict,
        solver: str = "daqp",
        damping: float = 0.5,
        max_iter: int = 20,
    ):
        self.motion_file = motion_file
        self.robot_xml = robot_xml
        self.mapping_table = config
        self.solver = solver
        self.damping = damping
        self.max_iter = max_iter

        # load the source motion
        self.source_motion = MotionSequence.load(self.motion_file)
        self.num_frames = self.source_motion.num_frames
        self.fps = self.source_motion.fps[0]

        self.retargeted_bodies = []
        target_bone_names = []
        for body_name, entry in config.items():
            if not entry["source"] or not entry["target"]:
                continue
            self.retargeted_bodies.append(body_name)
            target_bone_names.append(entry["target"])

        # add coordinate frames to the robot XML
        xml = open(self.robot_xml).read()
        xml = add_body_frames(xml, self.retargeted_bodies, prefix="current_", center_color=(0.0, 1.0, 1.0))
        xml = add_body_frames(xml, self.retargeted_bodies, prefix="target_", center_color=(1.0, 0.0, 1.0))
        open(self.robot_xml.replace(".xml", "_frames.xml"), "w").write(xml)

        model = mujoco.MjModel.from_xml_path(self.robot_xml.replace(".xml", "_frames.xml"))
        self.configuration = mink.Configuration(model)

        # create a new motion sequence for the retargeted motion
        self.target_motion = MotionSequence(
            num_frames=self.num_frames,
            dof_names=[model.joint(1 + i).name for i in range(model.nu)],
            body_names=target_bone_names,
            fps=self.fps,
        )

        self.tasks = []
        self.frame_tasks = {}

        for body_name in self.retargeted_bodies:
            mapping_entry = self.mapping_table[body_name]
            target_bone_name = mapping_entry["target"]
            position_weight = mapping_entry["weight"]["position"]
            orientation_weight = mapping_entry["weight"]["orientation"]

            print(f"Adding task for {body_name} ({target_bone_name})")
            task = mink.FrameTask(
                frame_name=target_bone_name,
                frame_type="body",
                position_cost=position_weight,
                orientation_cost=orientation_weight,
                lm_damping=1.0,
            )
            self.frame_tasks[target_bone_name] = task
            self.tasks.append(task)

        # add a posture task to keep the body in a reasonable posture
        # this task acts like a low-priority regularizer, biasing the solution towards
        # the default joint configuration, which is particularly helpful to avoid joint
        # locking up by itself from gimbal lock problem
        posture_task = mink.PostureTask(model, cost=0.1)
        self.tasks.append(posture_task)

        self.limits = [
            mink.ConfigurationLimit(model),
        ]

        self.model = self.configuration.model
        self.data = self.configuration.data

        self.viewer = mujoco.viewer.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        mujoco.mjv_defaultFreeCamera(self.model, self.viewer.cam)

        # Initialize to the home keyframe.
        for task in self.tasks:
            task.set_target_from_configuration(self.configuration)

    def calculate_error(self) -> float:
        """
        Calculate the error of the tasks.
        """
        return np.linalg.norm(
                np.concatenate(
                    [task.compute_error(self.configuration) for task in self.tasks]
                )
            )

    def solve_ik(self) -> float:
        """
        Perform one iteration of the inverse kinematics solving.
        """
        dt = self.configuration.model.opt.timestep
        vel = mink.solve_ik(
            configuration=self.configuration,
            tasks=self.tasks,
            dt=dt,
            solver=self.solver,
            damping=self.damping,
            limits=self.limits,
        )
        self.configuration.integrate_inplace(vel, dt)
        return self.calculate_error()

    def run(self, realtime: bool = False):
        """
        Run the motion retargeting.
        """
        if realtime:
            self.rate = RateLimiter(frequency=self.fps, warn=False)

        for frame_idx in range(self.num_frames):
            # update task targets
            for body_name in self.retargeted_bodies:
                mapping_entry = self.mapping_table[body_name]
                source_bone_name = mapping_entry["source"]
                target_bone_name = mapping_entry["target"]

                # TODO: might be better to optimize the following logic with numpy vectorization
                source_body_index = self.source_motion.get_body_indices([source_bone_name])[0]

                source_position = self.source_motion.body_positions[frame_idx, source_body_index]
                # get the position offset in (x, y, z) in meters
                if mapping_entry.get("offset") and mapping_entry["offset"].get("position"):
                    source_position += np.array(mapping_entry["offset"]["position"])

                source_orientation = self.source_motion.body_rotations[frame_idx, source_body_index]
                if mapping_entry.get("offset") and mapping_entry["offset"].get("orientation"):
                    source_orientation = quat_mul(source_orientation, np.array(mapping_entry["offset"]["orientation"]))

                self.frame_tasks[target_bone_name].set_target(mink.SE3(
                    wxyz_xyz=np.concatenate([source_orientation, source_position])
                ))

                # move the frame mocap body to the current body pose
                mink.move_mocap_to_frame(self.model, self.data, f"current_{body_name}_frame", target_bone_name, "body")
                # move the target frame mocap body to the target pose
                mocap_id = self.model.body(f"target_{body_name}_frame").mocapid[0]
                self.data.mocap_pos[mocap_id] = source_position
                self.data.mocap_quat[mocap_id] = source_orientation

            prev_error = self.calculate_error()
            num_iter = 0

            error = self.solve_ik()
            while prev_error - error > 0.001 and num_iter < self.max_iter:
                prev_error = error
                error = self.solve_ik()
                num_iter += 1

            # forward kinematics to update body positions and orientations
            mujoco.mj_forward(self.model, self.data)

            # store the joint motion data
            self.target_motion._dof_positions[frame_idx, :] = self.data.qpos[7:]

            # extract body data from the MuJoCo robot after IK solving
            for i, body_name in enumerate(self.retargeted_bodies):
                mapping_entry = self.mapping_table[body_name]
                body_id = self.model.body(mapping_entry["target"]).id

                # body position and rotation in world frame
                self.target_motion._body_positions[frame_idx, i, :] = self.data.xpos[body_id]
                self.target_motion._body_rotations[frame_idx, i, :] = self.data.xquat[body_id]

                # body linear and angular velocities in world frame
                self.target_motion._body_linear_velocities[frame_idx, i, :] = self.data.cvel[body_id][3:6]
                self.target_motion._body_angular_velocities[frame_idx, i, :] = self.data.cvel[body_id][0:3]

            # visualize at fixed FPS
            self.viewer.sync()
            if realtime:
                self.rate.sleep()

        # compute the velocities
        self.target_motion._dof_velocities[1:] = np.diff(self.target_motion._dof_positions, axis=0) / (1. / self.fps)

        self.viewer.close()

        motion_file_out = self.motion_file.replace(".npz", "_retargeted.npz")
        self.target_motion.save(motion_file_out)
        print(f"Results saved to {motion_file_out}")
