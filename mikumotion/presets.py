"""
Generic keypoint mapping for different motion capture systems.

Each retargeting configuration is a dictionary with the following keys:
    - "scale": the scale factor of the source motion
    - "mapping": a dictionary of body mappings, each mapping is a dictionary with the following keys:
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


ACCAD_TO_G1_CFG = {
    "scale": 1.0,
    "mapping": {
        "pelvis": {  # 0
            "source": "Hips",
            "target": "pelvis",
            "offset": {
                "position": (0.0, 0.0, 0.0),
                "orientation": (0.609, 0.359, 0.359, -0.609),
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
                "position": (0.0, 0.0, 0.0),
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
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
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "orientation": (1.0, 0.0, 0.0, 0.0),
            },
            "weight": {
                "position": 1.0,
                "orientation": 0.0,
            },
        },
        "left_elbow": {  # 18
            "source": "LeftForeArm",
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
            "source": "RightForeArm",
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
}


ACTORCORE_TO_G1_CFG = {
    "scale": 1.0,
    "mapping": {
        "pelvis": {  # 0
            "source": "CC_Base_Pelvis",
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
        "left_hip": {  # 1
            "source": "CC_Base_L_Thigh",
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
        "right_hip": {  # 2
            "source": "CC_Base_R_Thigh",
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
        "spine1": {  # 3
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
        "left_knee": {  # 4
            "source": "CC_Base_L_Calf",
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
        "right_knee": {  # 5
            "source": "CC_Base_R_Calf",
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
                "position": (0.0, 0.0, 0.0),
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
                "position": (0.0, 0.0, 0.0),
                "orientation": (1.0, 0.0, 0.0, 0.0),
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
            "source": "CC_Base_L_Foot",
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
        "right_foot": {  # 11
            "source": "CC_Base_R_Foot",
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
        "neck": {  # 12
            "source": "CC_Base_R_Foot",
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
        "right_shoulder": {  # 17
            "source": "CC_Base_R_Clavicle",
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
        "left_elbow": {  # 18
            "source": "CC_Base_L_Forearm",
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
            "source": "CC_Base_R_Forearm",
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
            "source": "CC_Base_L_Hand",
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
        "right_wrist": {  # 21
            "source": "CC_Base_R_Hand",
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
            "source": "CC_Base_L_Hand",
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
        "right_hand": {  # 23
            "source": "CC_Base_R_Hand",
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
    }
}


MMD_YYB_TO_G1_CFG = {
    "scale": 0.85,
    "mapping": {
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
}


MIXAMO_TO_G1_CFG = {
    "scale": 1.0,
    "mapping": {
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
        "spine1": {  # 3
            "source": "",
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
        "spine2": {  # 6
            "source": "",
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
            "source": "",
            "target": "",
            "offset": {
                "position": (0.0, 0.0, 0.0),
                "orientation": (0.123, 0.696, 0.696, -0.123),
            },
            "weight": {
                "position": 0.0,
                "orientation": 0.0,
            },
        },
        "right_ankle": {  # 8
            "source": "",
            "target": "",
            "offset": {
                "position": (0.0, 0.0, 0.0),
                "orientation": (0.123, 0.696, 0.696, -0.123),
            },
            "weight": {
                "position": 0.0,
                "orientation": 0.0,
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
    }
}


MESHCAPADE_TO_G1_CFG = {
    "scale": 1.0,
    "mapping": {
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
    }
}
