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
accad = {
    "pelvis":           {"bone": "Hips", "quat": [0.609, 0.359, 0.359, -0.609]},  # 0
    "left_hip":         {"bone": "LeftUpLeg",        },  # 1
    "right_hip":        {"bone": "RightUpLeg",        },  # 2
    "spine1":           {"bone": "ToSpine", "func": lambda b: b.tail },  # 3
    "left_knee":        {"bone": "LeftLeg",      },  # 4
    "right_knee":       {"bone": "RightLeg",      },  # 5
    "spine2":           {"bone": "Spine",     },  # 6
    "left_ankle":       {"bone": "LeftFoot", "quat": [0.123, 0.696, 0.696, -0.123]},  # 7
    "right_ankle":      {"bone": "RightFoot", "quat": [0.123, 0.696, 0.696, -0.123]},  # 8
    "spine3":           {"bone": "Spine1",     },  # 9
    "left_foot":        {"bone": "LeftToeBase", "quat": [0.062, -0.704, -0.704, -0.062]},  # 10
    "right_foot":       {"bone": "RightToeBase", "quat": [0.062, -0.704, -0.704, -0.062]},  # 11
    "neck":             {"bone": "Neck",          },  # 12
    "left_collar":      {"bone": "LeftShoulder",        },  # 13
    "right_collar":     {"bone": "RightShoulder",        },  # 14
    "head":             {"bone": "Head", "quat": [0.455, -0.542, -0.542, -0.455]},  # 15
    "left_shoulder":    {"bone": "LeftArm",        },  # 16
    "right_shoulder":   {"bone": "RightArm",        },  # 17
    "left_elbow":       {"bone": "LeftForeArm",      },  # 18
    "right_elbow":      {"bone": "RightForeArm",      },  # 19
    "left_wrist":       {"bone": "LeftHand", "quat": [0.5, 0.5, -0.5, 0.5]},  # 20
    "right_wrist":      {"bone": "RightHand", "quat": [0.5, 0.5, -0.5, 0.5]},  # 21
    "left_hand":        {"bone": "LeftHand", "func": lambda b: b.tail, "quat": [0.5, 0.5, -0.5, 0.5]},  # 22
    "right_hand":       {"bone": "RightHand", "func": lambda b: b.tail, "quat": [0.5, 0.5, -0.5, 0.5]},  # 23
}
# actorcore = {
#     "pelvis"            : "CC_Base_Pelvis",        lambda b: b.head),  # 0
#     "left_hip"          : ("CC_Base_L_Thigh",       lambda b: b.head),  # 1
#     "right_hip"         : ("CC_Base_R_Thigh",       lambda b: b.head),  # 2
#     "spine1"            : ("CC_Base_Spine02",       lambda b: b.head),  # 3
#     "left_knee"         : ("CC_Base_L_Calf",        lambda b: b.head),  # 4
#     "right_knee"        : ("CC_Base_R_Calf",        lambda b: b.head),  # 5
#     "spine2"            : ("CC_Base_Spine02",       lambda b: b.head),  # 6
#     "left_ankle"        : ("CC_Base_L_Foot",        lambda b: b.head),  # 7
#     "right_ankle"       : ("CC_Base_R_Foot",        lambda b: b.head),  # 8
#     "spine3"            : ("CC_Base_Spine02",       lambda b: b.head),  # 9
#     "left_foot"         : ("CC_Base_L_Foot",        lambda b: b.tail),  # 10
#     "right_foot"        : ("CC_Base_R_Foot",        lambda b: b.tail),  # 11
#     "neck"              : ("CC_Base_R_Foot",        lambda b: b.tail),  # 12
#     "left_collar"       : ("CC_Base_L_Clavicle",    lambda b: b.tail),  # 13
#     "right_collar"      : ("CC_Base_L_Clavicle",    lambda b: b.tail),  # 14
#     "head"              : ("CC_Base_Head",          lambda b: b.tail),  # 15
#     "left_shoulder"     : ("CC_Base_L_Clavicle",    lambda b: b.tail),  # 16
#     "right_shoulder"    : ("CC_Base_R_Clavicle",    lambda b: b.tail),  # 17
#     "left_elbow"        : ("CC_Base_L_Forearm",     lambda b: b.head),  # 18
#     "right_elbow"       : ("CC_Base_R_Forearm",     lambda b: b.head),  # 19
#     "left_wrist"        : ("CC_Base_L_Hand",        lambda b: b.head),  # 20
#     "right_wrist"       : ("CC_Base_R_Hand",        lambda b: b.head),  # 21
#     "left_hand"         : ("CC_Base_L_Hand",        lambda b: b.tail),  # 22
#     "right_hand"        : ("CC_Base_R_Hand",        lambda b: b.tail),  # 23
# }

G1_MMD_YYB_MAPPING = {
    "pelvis": {  # 0
        "source": "腰",
        "target": "pelvis",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (-0.327, 0.627, 0.627, 0.327),
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
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
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
            "orientation": (1.0, 0.0, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 0.0,
        },
    },
    "right_shoulder": {  # 17
        "source": "腕.R",
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
        "source": "ひじ.L",
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
        "source": "ひじ.R",
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
        "source": "手首.L",
        "target": "",
        "offset": {
            "position": (0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
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
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
    "right_hand": {  # 23
        "source": "手首.R",
        "target": "right_rubber_hand",
        "offset": {
            "position": (0.0, 0.0, 0.0),
            "orientation": (0.707, 0.707, 0.0, 0.0),
        },
        "weight": {
            "position": 1.0,
            "orientation": 1.0,
        },
    },
}
# mixamo = {
#     "pelvis"            : ("mixamorig:Hips",            lambda b: b.head),
#     "left_shoulder"     : ("mixamorig:LeftArm",         lambda b: b.head),
#     "left_elbow"        : ("mixamorig:LeftForeArm",     lambda b: b.head),
#     "left_hand"         : ("mixamorig:LeftHand",        lambda b: b.head),
#     "right_shoulder"    : ("mixamorig:RightArm",        lambda b: b.head),
#     "right_elbow"       : ("mixamorig:RightForeArm",    lambda b: b.head),
#     "right_hand"        : ("mixamorig:RightHand",       lambda b: b.head),
#     "left_hip"          : ("mixamorig:LeftUpLeg",       lambda b: b.head),
#     "left_knee"         : ("mixamorig:LeftLeg",         lambda b: b.head),
#     "left_foot"         : ("mixamorig:LeftToeBase",     lambda b: b.head),
#     "right_hip"         : ("mixamorig:RightUpLeg",      lambda b: b.head),
#     "right_knee"        : ("mixamorig:RightLeg",        lambda b: b.head),
#     "right_foot"        : ("mixamorig:RightToeBase",    lambda b: b.head),
#     "chest"             : ("mixamorig:Spine1",          lambda b: b.head),
#     "head"              : ("mixamorig:Head",            lambda b: b.head),
# }
# meshcapade = {
#     "pelvis"                    : ("pelvis",        lambda b: b.head),
#     "left_shoulder_roll_link"   : ("upperarm_l",    lambda b: b.head),
#     "left_elbow_link"           : ("lowerarm_l",    lambda b: b.head),
#     "left_wrist_pitch_link"     : ("hand_l",        lambda b: b.head),
#     "left_rubber_hand"          : ("hand_l",        lambda b: b.tail),
#     "right_shoulder_roll_link"  : ("upperarm_r",    lambda b: b.head),
#     "right_elbow_link"          : ("lowerarm_r",    lambda b: b.head),
#     "right_wrist_pitch_link"    : ("hand_r",        lambda b: b.head),
#     "right_rubber_hand"         : ("hand_r",        lambda b: b.tail),
#     "torso_link"                : ("torso",         lambda b: b.head),
#     "head_link"                 : ("head",          lambda b: b.head),
#     "left_hip_roll_link"        : ("thigh_l",       lambda b: b.head),
#     "left_knee_link"            : ("calf_l",        lambda b: b.head),
#     "left_ankle_pitch_link"     : ("foot_l",        lambda b: b.head),
#     "left_ankle_roll_link"      : ("ball_l",        lambda b: b.head),
#     "right_hip_roll_link"       : ("thigh_r",       lambda b: b.head),
#     "right_knee_link"           : ("calf_r",        lambda b: b.head),
#     "right_ankle_pitch_link"    : ("foot_r",        lambda b: b.head),
#     "right_ankle_roll_link"     : ("ball_r",        lambda b: b.head),
# }
