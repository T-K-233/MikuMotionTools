
import numpy as np
import mujoco
import mujoco.viewer


class MujocoViewer:
    g1_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5_mocap.xml"

    def __init__(self):
        self.mj_model = mujoco.MjModel.from_xml_path(self.g1_xml)
        self.mj_model.opt.timestep = 0.0001
        self.mj_data = mujoco.MjData(self.mj_model)
        self.viewer = mujoco.viewer.launch_passive(
            model=self.mj_model,
            data=self.mj_data,
            show_left_ui=False,
            show_right_ui=False,
        )
