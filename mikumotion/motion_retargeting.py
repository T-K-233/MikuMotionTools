from pathlib import Path

from loop_rate_limiters import RateLimiter
import numpy as np
import mink
import mujoco
import mujoco.viewer
from tqdm import tqdm

from mikumotion.math import quat_mul
from mikumotion.motion_sequence import MotionSequence, MotionStore, fill_body_velocities
from mikumotion.mujoco_utils import add_body_frames

SOLVER = "daqp"
DAMPING = 0.5
MAX_ITER = 40
KNEE_BIAS = 0.2    # rad, keeps knees off the straight-leg singularity
ELBOW_BIAS = -0.2
BASE_COORDS = 7    # a floating base occupies the first 7 qpos entries
ROOT_BODY_ID = 1   # MuJoCo body 0 is the world, so the robot's root link is body 1


class MotionRetargetingIK:
    """
    **animation -> robot, step 2 of 2: hub to hub.**
    Solves a robot's joints for a character motion, by IK. It mirrors
    :func:`mikumotion.blender.retarget_armature`, which retargets the other way.

    ``mapping`` pairs a source body with a robot link and an orientation cost, as
    ``{"left_hand": ("left_hand", 0.5)}``. The IK always tracks position at cost 1.0. A high
    orientation cost pins a limb's pose. A low one lets the IK swing the segment freely, and
    uses it only to steer the elbow or knee.

    Args:
        motion_name: Name of the source motion in the store.
        robot_xml: The robot MJCF, used for IK.
        store: Where ``reference/<name>`` is read and ``<robot>/<name>`` is written.
        mapping: Source body -> (robot link, orientation cost), from ``presets.RETARGET_MAPS``.
        urdf_path: The robot URDF, whose geometry goes into the robot layer.
    """
    def __init__(self, motion_name: str, robot_xml: str, store: MotionStore, mapping: dict,
                 urdf_path: str):
        self.motion_name = motion_name
        self.store = store
        self.urdf_path = urdf_path

        self.source_motion = store.read_reference_motion(motion_name)
        self.num_frames = self.source_motion.num_frames
        self.fps = self.source_motion.fps

        # source body -> target robot link, in a stable order
        self.retargeted_bodies = {source: target for source, (target, _) in mapping.items()}

        # draw where each tracked link currently is and where it is being pulled to
        links = list(self.retargeted_bodies.values())
        xml = Path(robot_xml).read_text()
        xml = add_body_frames(xml, links, prefix="current_", center_color=(0.0, 1.0, 1.0))
        xml = add_body_frames(xml, links, prefix="target_", center_color=(1.0, 0.0, 1.0))
        framed_xml = Path(robot_xml).with_name(Path(robot_xml).stem + "_frames.xml")
        framed_xml.write_text(xml)

        model = self.floating_base_model(str(framed_xml))
        self.configuration = mink.Configuration(model)
        self.model = self.configuration.model
        self.data = self.configuration.data

        joint_names = [self.model.joint(j).name for j in range(self.model.njnt)
                       if self.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE]

        # What the solve reached. Only the root link's pose is stored, as the placement that
        # puts the robot in the world, so only the root link is kept: every other reached pose
        # is the forward kinematics of the joints beside it.
        self.target_motion = MotionSequence(
            num_frames=self.num_frames,
            joint_names=joint_names,
            body_names=[self.model.body(ROOT_BODY_ID).name],
            fps=self.fps,
        )

        self.orientation_offsets = self.rest_offsets()

        # Where the IK is aimed, under the robot's own link names and in their frame
        # convention: the pose the robot cannot reach, which survives nowhere else and is what
        # a tracking reward compares the robot against. It does not depend on the solve, so it
        # is built once, here, and the frame loop only reads rows out of it.
        sources = self.source_motion.get_body_indices(self.retargeted_bodies)
        offsets = np.stack([self.orientation_offsets[body] for body in self.retargeted_bodies])
        positions = self.source_motion.body_positions[:, sources].astype(np.float64)
        rotations = quat_mul(self.source_motion.body_rotations[:, sources], offsets)  # float64

        self.goal_motion = MotionSequence(
            num_frames=self.num_frames,
            joint_names=[],  # a goal is a body pose; the joints beside it are what was reached
            body_names=list(self.retargeted_bodies.values()),
            fps=self.fps,
        )
        self.goal_motion.body_positions[:] = positions
        self.goal_motion.body_rotations[:] = rotations
        fill_body_velocities(self.goal_motion)

        # The same goal as one (frames, bodies, 7) array, in the order mink's SE3 takes it.
        # It stays float64: rounding to the stored float32 would move the target the IK sees.
        self.targets = np.concatenate([rotations, positions], axis=-1)

        # one mocap id per tracked link, looked up once: the frame loop runs 2000+ times
        self.markers = {link: self.model.body(f"target_{link}_frame").mocapid[0] for link in links}

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
            posture[BASE_COORDS + index] = KNEE_BIAS if "knee" in name else (
                ELBOW_BIAS if "elbow" in name else posture[BASE_COORDS + index])
        posture_task = mink.PostureTask(self.model, cost=0.1)
        posture_task.set_target(posture)

        self.data.qpos = posture
        mujoco.mj_forward(self.model, self.data)
        self.limits = [mink.ConfigurationLimit(self.model)]

        for task in self.tasks:
            task.set_target_from_configuration(self.configuration)
        self.tasks.append(posture_task)

    def rest_offsets(self) -> dict:
        """
        Return one constant rotation per mapped pair, from the source body's frame to the
        link's.

        A mocap bone and the link it drives disagree about which way their axes point. The
        zamuza foot and the lite_pro foot sit 90 degrees apart with both rigs at rest.
        Aiming the IK at the source's absolute orientation therefore twists the robot by
        that much. The offset makes the target the source's rotation relative to its own
        rest pose, applied to the robot's zero pose.
        """
        reference = f"{self.motion_name}_reset"
        assert self.store.reference_file(reference).exists(), (
            f"retargeting needs the source rig's rest pose at {self.store.reference_file(reference)}, "
            f"to measure how far each bone's frame sits from the link it drives. "
            f"scripts/blender/convert_mocap_to_motion.py writes it beside the animation."
        )
        rest = self.store.read_reference_motion(reference)
        zero = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, zero)
        mujoco.mj_kinematics(self.model, zero)

        offsets = {}
        for source, target in self.retargeted_bodies.items():
            source_rest = rest.body_rotations[0, rest.get_body_indices([source])[0]]
            link_zero = zero.xquat[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, target)]
            offsets[source] = quat_mul(source_rest * np.array([1.0, -1.0, -1.0, -1.0]), link_zero)
        return offsets

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

    def show_targets(self, targets) -> None:
        """
        Move the viewer's marker pairs: where each tracked link is, where it is pulled to.

        Only worth doing when a window is open, so ``run`` calls this under its viewer check.
        The markers are named after the robot's links, which is what they follow.
        """
        for (link, mocap_id), target in zip(self.markers.items(), targets):
            mink.move_mocap_to_frame(self.model, self.data, f"current_{link}_frame", link, "body")
            self.data.mocap_pos[mocap_id] = target[4:]
            self.data.mocap_quat[mocap_id] = target[:4]

    def calculate_error(self) -> float:
        """Return the combined error of every task."""
        return np.linalg.norm(
                np.concatenate(
                    [task.compute_error(self.configuration) for task in self.tasks]
                )
            )

    def solve_ik(self) -> float:
        """Run one IK iteration and return the error that remains."""
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
        Solve every frame and write the result as ``<robot>/<motion>``: the joints the solve
        reached, and the goal it was aimed at.

        With ``show_viewer``, a passive MuJoCo window follows the solve at playback speed.
        Without it, the solve runs headless as fast as it can.
        """
        viewer = mujoco.viewer.launch_passive(
            model=self.model, data=self.data, show_left_ui=False, show_right_ui=False,
        ) if show_viewer else None
        if viewer is not None:
            mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
            rate = RateLimiter(frequency=self.fps / 4, warn=False)

        for frame_idx in tqdm(range(self.num_frames)):
            for task, target in zip(self.frame_tasks.values(), self.targets[frame_idx]):
                task.set_target(mink.SE3(wxyz_xyz=target))
            if viewer is not None:
                self.show_targets(self.targets[frame_idx])

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

            mujoco.mj_forward(self.model, self.data)
            self.target_motion.joint_positions[frame_idx, :] = self.data.qpos[BASE_COORDS:]

            # The pose, not the velocity: IK integrates qpos and leaves qvel at zero, so
            # mj_objectVelocity would report the base as motionless.
            self.target_motion.body_positions[frame_idx, 0, :] = self.data.xpos[ROOT_BODY_ID]
            self.target_motion.body_rotations[frame_idx, 0, :] = self.data.xquat[ROOT_BODY_ID]

            if viewer is not None:
                viewer.sync()
                rate.sleep()

        self.target_motion.joint_velocities[1:] = np.diff(self.target_motion.joint_positions, axis=0) * self.fps

        if viewer is not None:
            viewer.close()

        path = self.store.write_robot_motion(self.motion_name, self.target_motion,
                                             self.urdf_path, self.goal_motion)
        print(f"Results saved to {path}")
