"""
Generic keypoint mapping for different motion capture systems.

Each body mapping is a dictionary with the following keys:
    - "source": the name of the bone in the source motion
    - "target": the name of the bone in the target model
    - "offset": the offset from the source bone to the target bone in global coordinate frame, containing:
        - "position": the position offset in (x, y, z) in meters
        - "orientation": the orientation offset in (qw, qx, qy, qz) quaternion
    - "weight": the weight / cost of each body in the IK solver, containing:
        - "position": the position weight
        - "orientation": the orientation weight

The position and rotation offsets are the relative offsets between the source bone
and the target bone in global coordinate frame.

We follow the SMPL-X as the body naming convention,
see https://github.com/vchoutas/smplx/blob/main/smplx/joint_names.py#L244C21-L268C18
and https://chingswy.github.io/easymocap-public-doc/database/2_keypoints.html#smpl

"""

import numpy as np


ACCAD_TO_G1_CFG = {
    "pelvis": {  # 0
        "source": "Hips",
        "target": "pelvis",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.612, 0.354, 0.354, -0.612),
        },
        "weight": {
            "position": 10.0,
            "orientation": 1.0,
        },
    },
    "left_hip": {  # 1
        "source": "LeftUpLeg",
        "target": "left_hip_roll_link",
        "offset": {
            "position": (0.0, 0.04, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_hip": {  # 2
        "source": "RightUpLeg",
        "target": "right_hip_roll_link",
        "offset": {
            "position": (0.0, -0.04, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "spine1": {  # 3
        "source": "ToSpine",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_knee": {  # 4
        "source": "LeftLeg",
        "target": "left_knee_link",
        "offset": {
            "position": (0.0, 0.04, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_knee": {  # 5
        "source": "RightLeg",
        "target": "right_knee_link",
        "offset": {
            "position": (0.0, -0.04, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "spine2": {  # 6
        "source": "Spine",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_ankle": {  # 7
        "source": "LeftFoot",
        "target": "",
        "offset": {
            "position": (0.0, 0.04, 0.0),
            "orientation": (0.123, 0.696, 0.696, -0.123),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_ankle": {  # 8
        "source": "RightFoot",
        "target": "",
        "offset": {
            "position": (0.0, -0.04, 0.0),
            "orientation": (0.123, 0.696, 0.696, -0.123),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "spine3": {  # 9
        "source": "Spine1",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_foot": {  # 10
        "source": "LeftToeBase",
        "target": "left_ankle_roll_link",
        "offset": {
            "position": (0.0, 0.04, 0.0),
            "orientation": (0.062, -0.704, -0.704, -0.062),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_foot": {  # 11
        "source": "RightToeBase",
        "target": "right_ankle_roll_link",
        "offset": {
            "position": (0.0, -0.04, 0.0),
            "orientation": (0.062, -0.704, -0.704, -0.062),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "neck": {  # 12
        "source": "Neck",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_collar": {  # 13
        "source": "LeftShoulder",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.0, -0.707, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_collar": {  # 14
        "source": "RightShoulder",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.0, 0.707, 0.0, 0.707),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "head": {  # 15
        "source": "Head",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.455, -0.542, -0.542, -0.455),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_shoulder": {  # 16
        "source": "LeftArm",
        "target": "left_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.0, -0.707, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_shoulder": {  # 17
        "source": "RightArm",
        "target": "right_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.0, 0.707, 0.0, 0.707),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_elbow": {  # 18
        "source": "",
        "target": "left_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_elbow": {  # 19
        "source": "",
        "target": "right_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_wrist": {  # 20
        "source": "LeftHand",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_wrist": {  # 21
        "source": "RightHand",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_hand": {  # 22
        "source": "LeftHand",
        "target": "left_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_hand": {  # 23
        "source": "RightHand",
        "target": "right_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
}


ACTORCORE_TO_G1_CFG = {
    "pelvis": {  # 0
        "source": "CC_Base_Pelvis",
        "target": "pelvis",
        "offset": {
            "position": (0.0, 0.0, 0.05),
            "orientation": (0.455, -0.542, -0.542, -0.455),  # X -100 deg, Z -90 deg
        },
        "weight": {
            "position": 10.0,
            "orientation": 1.0,
        },
    },
    "left_hip": {  # 1
        "source": "CC_Base_L_Thigh",
        "target": "left_hip_roll_link",
        "offset": {
            "position": (0.0, 0.05, -0.08),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_hip": {  # 2
        "source": "CC_Base_R_Thigh",
        "target": "right_hip_roll_link",
        "offset": {
            "position": (0.0, -0.05, -0.08),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "spine1": {  # 3
        "source": "CC_Base_Spine01",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_knee": {  # 4
        "source": "CC_Base_L_Calf",
        "target": "left_knee_link",
        "offset": {
            "position": (0.0, 0.05, -0.03),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_knee": {  # 5
        "source": "CC_Base_R_Calf",
        "target": "right_knee_link",
        "offset": {
            "position": (0.0, -0.05, -0.03),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "spine2": {  # 6
        "source": "CC_Base_Spine02",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_ankle": {  # 7
        "source": "CC_Base_L_Foot",
        "target": "",
        "offset": {
            "position": (0.0, 0.05, 0.0),
            "orientation": (0.704, 0.062, -0.062, 0.704),  # X +10 deg, Z +90 deg
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_ankle": {  # 8
        "source": "CC_Base_R_Foot",
        "target": "",
        "offset": {
            "position": (0.0, -0.05, 0.0),
            "orientation": (0.704, 0.062, -0.062, 0.704),  # X +10 deg, Z +90 deg
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "spine3": {  # 9
        "source": "CC_Base_Spine02",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_foot": {  # 10
        "source": "CC_Base_L_ToeBaseShareBone",
        "target": "left_ankle_roll_link",
        "offset": {
            "position": (0.0, 0.05, 0.0),
            "orientation": (0.706, -0.031, 0.031, 0.706),  # X -5 deg, Z +90 deg
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_foot": {  # 11
        "source": "CC_Base_R_ToeBaseShareBone",
        "target": "right_ankle_roll_link",
        "offset": {
            "position": (0.0, -0.05, 0.0),
            "orientation": (0.706, -0.031, 0.031, 0.706),  # X -5 deg, Z +90 deg
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "neck": {  # 12
        "source": "CC_Base_NeckTwist02",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_collar": {  # 13
        "source": "CC_Base_L_Clavicle",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_collar": {  # 14
        "source": "CC_Base_R_Clavicle",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "head": {  # 15
        "source": "CC_Base_Head",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_shoulder": {  # 16
        "source": "CC_Base_L_Upperarm",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_shoulder": {  # 17
        "source": "CC_Base_R_Upperarm",
        "target": "right_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_elbow": {  # 18
        "source": "CC_Base_L_Forearm",
        "target": "left_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_elbow": {  # 19
        "source": "CC_Base_R_Forearm",
        "target": "right_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.5, 0.5, -0.5, 0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_wrist": {  # 20
        "source": "CC_Base_L_Hand",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.0, 0.0, 0.707),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_wrist": {  # 21
        "source": "CC_Base_R_Hand",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.0, 0.0, 0.707),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_hand": {  # 22
        "source": "CC_Base_L_Hand",
        "target": "left_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.0, 0.0, 0.707),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_hand": {  # 23
        "source": "CC_Base_R_Hand",
        "target": "right_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.0, 0.0, 0.707),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
}


MMD_YYB_TO_G1_CFG = {
    "pelvis": {  # 0
        "source": "下半身",
        "target": "pelvis",
        "offset": {
            "position": (0.0, 0.02, -0.03),
            "orientation": (-0.561, -0.431, -0.431, 0.561),
        },
        "weight": {
            "position": 10.0,
            "orientation": 1.0,
        },
    },
    "left_hip": {  # 1
        "source": "足.L",
        "target": "left_hip_roll_link",
        "offset": {
            "position": (0.04, 0.0, -0.1),
            "orientation": (0.5, 0.5, 0.5, -0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_hip": {  # 2
        "source": "足.R",
        "target": "right_hip_roll_link",
        "offset": {
            "position": (-0.04, 0.0, -0.1),
            "orientation": (0.5, 0.5, 0.5, -0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "spine1": {  # 3
        "source": "上半身",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_knee": {  # 4
        "source": "ひざ.L",
        "target": "left_knee_link",
        "offset": {
            "position": (0.04, 0.0, -0.04),
            "orientation": (0.5, 0.5, 0.5, -0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_knee": {  # 5
        "source": "ひざ.R",
        "target": "right_knee_link",
        "offset": {
            "position": (-0.04, 0.0, -0.04),
            "orientation": (0.5, 0.5, 0.5, -0.5),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "spine2": {  # 6
        "source": "上半身2",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_ankle": {  # 7
        "source": "足首.L",
        "target": "",
        "offset": {
            "position": (0.035, 0.0, 0.0),
            "orientation": (0.153, 0.690, 0.690, -0.153),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_ankle": {  # 8
        "source": "足首.R",
        "target": "",
        "offset": {
            "position": (-0.035, 0.0, 0.0),
            "orientation": (0.153, 0.690, 0.690, -0.153),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "spine3": {  # 9
        "source": "上半身2",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_foot": {  # 10
        "source": "足先EX.L",
        "target": "left_ankle_roll_link",
        "offset": {
            "position": (0.035, 0.0, 0.0),
            "orientation": (0.0, 0.707, 0.707, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_foot": {  # 11
        "source": "足先EX.R",
        "target": "right_ankle_roll_link",
        "offset": {
            "position": (-0.035, 0.0, 0.0),
            "orientation": (0.0, 0.707, 0.707, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "neck": {  # 12
        "source": "首",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_collar": {  # 13
        "source": "肩.L",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_collar": {  # 14
        "source": "肩.R",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "head": {  # 15
        "source": "頭",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_shoulder": {  # 16
        "source": "腕.L",
        "target": "left_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 2.0,
            "orientation": 0.0,
        },
    },
    "right_shoulder": {  # 17
        "source": "腕.R",
        "target": "right_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 2.0,
            "orientation": 0.0,
        },
    },
    "left_elbow": {  # 18
        "source": "ひじ.L",
        "target": "left_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_elbow": {  # 19
        "source": "ひじ.R",
        "target": "right_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_wrist": {  # 20
        "source": "手首.L",
        "target": "",
        "offset": {
            "position": (0.05, 0.0, -0.05),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_wrist": {  # 21
        "source": "手首.R",
        "target": "",
        "offset": {
            "position": (-0.05, 0.0, -0.05),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_hand": {  # 22
        "source": "手首.L",
        "target": "left_rubber_hand",
        "offset": {
            "position": (0.1, 0.0, -0.1),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 5.0,
            "orientation": 0.5,
        },
    },
    "right_hand": {  # 23
        "source": "手首.R",
        "target": "right_rubber_hand",
        "offset": {
            "position": (-0.1, 0.0, -0.1),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 5.0,
            "orientation": 0.5,
        },
    },
}


MIXAMO_TO_G1_CFG = {
    "pelvis": {  # 0
        "source": "mixamorig:Hips",
        "target": "pelvis",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 10.0,
            "orientation": 1.0,
        },
    },
    "left_shoulder": {  # 16
        "source": "mixamorig:LeftArm",
        "target": "left_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_elbow": {  # 18
        "source": "mixamorig:LeftForeArm",
        "target": "left_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_hand": {  # 22
        "source": "mixamorig:LeftHand",
        "target": "left_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_shoulder": {  # 17
        "source": "mixamorig:RightArm",
        "target": "right_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_elbow": {  # 19
        "source": "mixamorig:RightForeArm",
        "target": "right_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_hand": {  # 23
        "source": "mixamorig:RightHand",
        "target": "right_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "left_hip": {  # 1
        "source": "mixamorig:LeftUpLeg",
        "target": "left_hip_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_knee": {  # 4
        "source": "mixamorig:LeftLeg",
        "target": "left_knee_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_foot": {  # 10
        "source": "mixamorig:LeftToeBase",
        "target": "left_ankle_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_hip": {  # 2
        "source": "mixamorig:RightUpLeg",
        "target": "right_hip_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_knee": {  # 5
        "source": "mixamorig:RightLeg",
        "target": "right_knee_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_foot": {  # 11
        "source": "mixamorig:RightToeBase",
        "target": "right_ankle_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "spine3": {  # 9
        "source": "mixamorig:Spine1",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "head": {  # 15
        "source": "mixamorig:Head",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
}


MESHCAPADE_TO_G1_CFG = {
    "pelvis": {  # 0
        "source": "pelvis",
        "target": "pelvis",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 10.0,
            "orientation": 1.0,
        },
    },
    "left_shoulder": {  # 16
        "source": "upperarm_l",
        "target": "left_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_elbow": {  # 18
        "source": "lowerarm_l",
        "target": "left_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_wrist": {  # 20
        "source": "hand_l",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_hand": {  # 22
        "source": "hand_l",
        "target": "left_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_shoulder": {  # 17
        "source": "upperarm_r",
        "target": "right_shoulder_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_elbow": {  # 19
        "source": "lowerarm_r",
        "target": "right_elbow_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_wrist": {  # 21
        "source": "hand_r",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_hand": {  # 23
        "source": "hand_r",
        "target": "right_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "spine2": {  # 6
        "source": "torso",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "head": {  # 15
        "source": "head",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_hip": {  # 1
        "source": "thigh_l",
        "target": "left_hip_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_knee": {  # 4
        "source": "calf_l",
        "target": "left_knee_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "left_ankle": {  # 7
        "source": "foot_l",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "left_foot": {  # 10
        "source": "ball_l",
        "target": "left_ankle_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_hip": {  # 2
        "source": "thigh_r",
        "target": "right_hip_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_knee": {  # 5
        "source": "calf_r",
        "target": "right_knee_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_ankle": {  # 8
        "source": "foot_r",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 0.0,
            "orientation": 0.0,
        },
    },
    "right_foot": {  # 11
        "source": "ball_r",
        "target": "right_ankle_roll_link",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
}

PRESETS = {
    "ACCAD_TO_G1_CFG": ACCAD_TO_G1_CFG,
    "ACTORCORE_TO_G1_CFG": ACTORCORE_TO_G1_CFG,
    "MMD_YYB_TO_G1_CFG": MMD_YYB_TO_G1_CFG,
    "MESHCAPADE_TO_G1_CFG": MESHCAPADE_TO_G1_CFG,
    "MIXAMO_TO_G1_CFG": MIXAMO_TO_G1_CFG,
}


# ============================================================
# motion  ->  robot joints  (direction 1, the IK step)
# ============================================================
# ``{source_body: (target_body, orientation_cost)}`` for MotionRetargetingIK. The source
# names are the mocap bodies of a MOCAP_EXPORTS rig; the targets are links of the robot
# being solved for. A high orientation cost pins a limb's pose; a low one lets IK swing
# the segment freely and only uses it to steer the elbow/knee (a pole target).

ZAMUZA_TO_LITE_PRO = {
    "pelvis": ("pelvis", 0.5),
    "torso": ("chest", 0.5),
    "head": ("head", 0.5),
    "left_hand": ("left_hand", 0.5),
    "right_hand": ("right_hand", 0.5),
    "left_foot": ("left_foot", 0.5),
    "right_foot": ("right_foot", 0.5),
    "left_upper_arm": ("left_shoulder_yaw", 0.1),
    "right_upper_arm": ("right_shoulder_yaw", 0.1),
    "left_lower_arm": ("left_elbow_pitch", 0.1),
    "right_lower_arm": ("right_elbow_pitch", 0.1),
    "left_upper_leg": ("left_hip_yaw", 0.1),
    "right_upper_leg": ("right_hip_yaw", 0.1),
    "left_lower_leg": ("left_knee_pitch", 0.1),
    "right_lower_leg": ("right_knee_pitch", 0.1),
}

RETARGET_MAPS = {
    "ZAMUZA_TO_LITE_PRO": ZAMUZA_TO_LITE_PRO,
}


# ============================================================
# Blender mocap armature  ->  motion  (direction 1: animation -> robot)
# ============================================================
# One entry per source rig for scripts/blender/export_mocap.py. Bones are listed in
# SMPL keypoint order, so the same index means the same body part across every rig.
# ``scale`` converts the rig's units to metres; ``rotate_z`` turns it to +X forward,
# since Blender rigs are authored +Y forward.

MOCAP_EXPORTS = {
    "zamuza": {
        "armature": "Armature",
        "scale": 1.0,
        "rotate_z": 0.0,
        "frames": (0, 2000),
        # this rig has a single spine bone, so it fills the spine1 slot only; listing it
        # twice would export the same pose under one name twice, which the store rejects
        "bones": [
            "pelvis", "left_upper_leg", "right_upper_leg", "torso",
            "left_lower_leg", "right_lower_leg",
            "left_foot", "right_foot", "head",
            "left_upper_arm", "right_upper_arm",
            "left_lower_arm", "right_lower_arm",
            "left_hand", "right_hand",
        ],
    },
    "accad": {
        "armature": "armature",
        "scale": 0.8,
        "rotate_z": -np.pi / 2,
        "frames": (0, 652),
        # LeftForearm/RightForearm are omitted: those two bones are buggy in this rig
        "bones": [
            "Hips", "LeftUpLeg", "RightUpLeg", "ToSpine",
            "LeftLeg", "RightLeg", "Spine", "LeftFoot", "RightFoot", "Spine1",
            "LeftToeBase", "RightToeBase", "Neck",
            "LeftShoulder", "RightShoulder", "Head",
            "LeftArm", "RightArm", "LeftHand", "RightHand",
        ],
    },
    "actorcore": {
        "armature": "Armature",
        "scale": 0.7 * 0.01,
        "rotate_z": np.pi / 2,
        "frames": (0, 293),
        "bones": [
            "CC_Base_Pelvis", "CC_Base_L_Thigh", "CC_Base_R_Thigh", "CC_Base_Spine01",
            "CC_Base_L_Calf", "CC_Base_R_Calf", "CC_Base_Spine02",
            "CC_Base_L_Foot", "CC_Base_R_Foot",
            "CC_Base_L_ToeBaseShareBone", "CC_Base_R_ToeBaseShareBone",
            "CC_Base_NeckTwist02", "CC_Base_L_Clavicle", "CC_Base_R_Clavicle",
            "CC_Base_Head", "CC_Base_L_Upperarm", "CC_Base_R_Upperarm",
            "CC_Base_L_Forearm", "CC_Base_R_Forearm",
            "CC_Base_L_Hand", "CC_Base_R_Hand",
        ],
    },
}


# ============================================================
# lite_pro robot  ->  VRM / VRoid humanoid
# ============================================================
# Bone map for mikumotion.blender.retarget_armature: {target_vrm_bone: source_robot_link}.
#
# The robot models each articulation as a *chain* of 1-DOF links, so each VRM
# ball bone is driven by the robot link that carries that segment's full
# accumulated world orientation (the last link of the chain). E.g. the upper arm
# takes the whole 3-DOF shoulder, so the VRM `Shoulder` bone is intentionally left
# unmapped. Robot fingers have 4 joints (j1..j4); the VRM has 3 phalanges, so
# j1/j2/j3 map to phalanx 1/2/3 and the extra tip (j4) is dropped.

# (target, source) bone whose world *translation* drives the whole rig
LITE_PRO_TO_VROID_TRANSLATION_ROOT = ("J_Bip_C_Hips", "pelvis")

# vrm finger name -> robot finger index (finger1 is the thumb: offset toward +x,
# nearest the wrist; finger3 the longest = middle; finger5 the shortest = little)
VROID_FINGERS = {"Thumb": 1, "Index": 2, "Middle": 3, "Ring": 4, "Little": 5}


def build_lite_pro_to_vroid_bone_map() -> "dict[str, str]":
    m = {
        # spine + head (torso chain: pelvis -> waist_yaw -> waist_roll -> chest)
        "J_Bip_C_Hips": "pelvis",          # translation root
        "J_Bip_C_Spine": "waist_yaw",
        "J_Bip_C_Chest": "waist_roll",
        "J_Bip_C_UpperChest": "chest",
        "J_Bip_C_Neck": "neck_roll",
        "J_Bip_C_Head": "head",
    }
    for v, r in (("L", "left"), ("R", "right")):
        # arms (Shoulder omitted on purpose: UpperArm absorbs the full 3-DOF shoulder)
        m[f"J_Bip_{v}_UpperArm"] = f"{r}_shoulder_yaw"
        m[f"J_Bip_{v}_LowerArm"] = f"{r}_elbow_pitch"
        m[f"J_Bip_{v}_Hand"] = f"{r}_hand"
        # legs (hip chain -> hip_yaw thigh; ankle chain -> foot)
        m[f"J_Bip_{v}_UpperLeg"] = f"{r}_hip_yaw"
        m[f"J_Bip_{v}_LowerLeg"] = f"{r}_knee_pitch"
        m[f"J_Bip_{v}_Foot"] = f"{r}_foot"
        # fingers
        for vf, rn in VROID_FINGERS.items():
            for k in (1, 2, 3):
                m[f"J_Bip_{v}_{vf}{k}"] = f"{r}_finger{rn}_j{k}"
    return m


LITE_PRO_TO_VROID_BONE_MAP = build_lite_pro_to_vroid_bone_map()
