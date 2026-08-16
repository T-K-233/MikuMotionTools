from pathlib import Path

from loop_rate_limiters import RateLimiter
import numpy as np
import mink
import mujoco
import mujoco.viewer
from tqdm import tqdm

from mikumotion.motion_sequence import MotionSequence, MotionStore
from mikumotion.mujoco_utils import add_body_frames

SOLVER = "daqp"
DAMPING = 0.5
MAX_ITER = 40
KNEE_BIAS = 0.2    # rad, keeps knees off the straight-leg singularity
ELBOW_BIAS = -0.2


class MotionRetargetingIK:
    """
    **animation -> robot, step 2 of 2: hub to hub.**
    Solves a robot's joints for a character motion, by IK. The mirror of
    :func:`mikumotion.blender.retarget_armature`, which retargets the other way.

    The MotionRetargetingIK logic.

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
        motion_name: Name of the source motion in the store.
        robot_xml: The robot XML file used for IK solving.
        store: Where the source motion is read from and ``<name>_retargeted`` is written.
    """
    def __init__(self, motion_name: str, robot_xml: str, store: MotionStore, mapping: dict):
        self.motion_name = motion_name
        self.store = store
        self.mapping = mapping

        self.source_motion = store.read_motion(motion_name)
        self.num_frames = self.source_motion.num_frames
        self.fps = self.source_motion.fps

        # source body -> target robot link, in a stable order
        self.retargeted_bodies = {source: target for source, (target, _) in mapping.items()}

        # draw where each body currently is and where it is being pulled to
        xml = Path(robot_xml).read_text()
        xml = add_body_frames(xml, self.retargeted_bodies, prefix="current_", center_color=(0.0, 1.0, 1.0))
        xml = add_body_frames(xml, self.retargeted_bodies, prefix="target_", center_color=(1.0, 0.0, 1.0))
        framed_xml = Path(robot_xml).with_name(Path(robot_xml).stem + "_frames.xml")
        framed_xml.write_text(xml)

        model = self.floating_base_model(str(framed_xml))
        self.configuration = mink.Configuration(model)
        self.model = self.configuration.model
        self.data = self.configuration.data

        # a floating base occupies the first 7 qpos entries; the rest are the joints we solve for
        self.num_base_coords = 7
        joint_names = [self.model.joint(j).name for j in range(self.model.njnt)
                       if self.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE]

        self.target_motion = MotionSequence(
            num_frames=self.num_frames,
            joint_names=joint_names,
            body_names=list(self.retargeted_bodies.values()),
            fps=self.fps,
        )

        self.tasks = []
        self.frame_tasks = {}
        for source, (target, orientation_cost) in mapping.items():
            print(f"Adding task for {source} -> {target}")
            task = mink.FrameTask(
                frame_name=target,
                frame_type="body",
                position_cost=1.0,
                orientation_cost=orientation_cost,
                lm_damping=1.0,
            )
            self.frame_tasks[target] = task
            self.tasks.append(task)

        # A low-priority regularizer biasing knees and elbows towards a slight bend, which
        # keeps them off the singularity where a straight limb can gimbal-lock.
        posture = self.data.qpos.copy()
        for index, name in enumerate(joint_names):
            posture[self.num_base_coords + index] = KNEE_BIAS if "knee" in name else (
                ELBOW_BIAS if "elbow" in name else posture[self.num_base_coords + index])
        posture_task = mink.PostureTask(self.model, cost=0.1)
        posture_task.set_target(posture)

        self.data.qpos = posture
        mujoco.mj_forward(self.model, self.data)
        self.limits = [mink.ConfigurationLimit(self.model)]

        for task in self.tasks:
            task.set_target_from_configuration(self.configuration)
        self.tasks.append(posture_task)

    def floating_base_model(self, xml_path: str):
        """
        Compile the robot with a floating base.

        IK has to place the whole robot in the world, not just bend its joints, and a
        description written for a fixed-base simulation (lite_pro) pins its root link.
        """
        spec = mujoco.MjSpec.from_file(xml_path)
        root = spec.bodies[1]  # bodies[0] is the implicit world body
        if not any(joint.type == mujoco.mjtJoint.mjJNT_FREE for joint in root.joints):
            root.add_freejoint()
        return spec.compile()

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
            solver=SOLVER,
            damping=DAMPING,
            limits=self.limits,
        )
        self.configuration.integrate_inplace(vel, dt)
        return self.calculate_error()

    def run(self, show_viewer: bool = False):
        """
        Solve every frame and write the result as ``<motion>_retargeted``.

        With ``show_viewer`` a passive MuJoCo window follows the solve, throttled to
        playback speed; without it the solve runs headless as fast as it can.
        """
        viewer = mujoco.viewer.launch_passive(
            model=self.model, data=self.data, show_left_ui=False, show_right_ui=False,
        ) if show_viewer else None
        if viewer is not None:
            mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
            rate = RateLimiter(frequency=self.fps / 4, warn=False)

        for frame_idx in tqdm(range(self.num_frames)):
            # update task targets
            for body_name, target_body_name in self.retargeted_bodies.items():
                # TODO: might be better to optimize the following logic with numpy vectorization
                source_body_index = self.source_motion.get_body_indices([body_name])[0]

                source_position = self.source_motion.body_positions[frame_idx, source_body_index].copy()

                # get the position offset in (x, y, z) in meters
                source_orientation = self.source_motion.body_rotations[frame_idx, source_body_index].copy()

                self.frame_tasks[target_body_name].set_target(mink.SE3(
                    wxyz_xyz=np.concatenate([source_orientation, source_position])
                ))

                # move the frame mocap body to the current body pose
                mink.move_mocap_to_frame(self.model, self.data, f"current_{body_name}_frame", body_name, "body")
                # move the target frame mocap body to the target pose
                mocap_id = self.model.body(f"target_{body_name}_frame").mocapid[0]
                self.data.mocap_pos[mocap_id] = source_position
                self.data.mocap_quat[mocap_id] = source_orientation

            prev_error = self.calculate_error()
            num_iter = 0

            error = self.solve_ik()
            while prev_error - error > 0.0001:
                if num_iter >= MAX_ITER:
                    print(f"Maximum number of iterations reached for frame {frame_idx}")
                    break
                prev_error = error
                error = self.solve_ik()
                num_iter += 1

            # forward kinematics to update body positions and orientations
            mujoco.mj_forward(self.model, self.data)

            # store the joint motion data, skipping the floating base's 7 coordinates
            self.target_motion.joint_positions[frame_idx, :] = self.data.qpos[self.num_base_coords:]

            # extract body data from the MuJoCo robot after IK solving. target_motion's
            # body_names are retargeted_bodies' values, so iterating it keeps them aligned.
            for i, target_body_name in enumerate(self.retargeted_bodies.values()):
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, target_body_name)

                # body position and rotation in world frame
                self.target_motion.body_positions[frame_idx, i, :] = self.data.xpos[body_id]
                self.target_motion.body_rotations[frame_idx, i, :] = self.data.xquat[body_id]

                # body linear and angular velocities in world frame
                velocity = np.empty(6)
                is_local = False
                mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body_id, velocity, is_local)
                self.target_motion.body_linear_velocities[frame_idx, i, :] = velocity[3:6]
                self.target_motion.body_angular_velocities[frame_idx, i, :] = velocity[0:3]

            if viewer is not None:
                viewer.sync()
                rate.sleep()

        # compute the velocities
        self.target_motion.joint_velocities[1:] = np.diff(self.target_motion.joint_positions, axis=0) * self.fps

        if viewer is not None:
            viewer.close()

        retargeted_name = f"{self.motion_name}_retargeted"
        self.store.write_motion(retargeted_name, self.target_motion)
        print(f"Results saved to {self.store.motion_file(retargeted_name)}")
