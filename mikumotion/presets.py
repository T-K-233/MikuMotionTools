from configclass import configclass

from mikumotion.mapping import GenericMapping, MappingEntry, Offset, Weight


@configclass
class G1ACCADMapping(GenericMapping):
    pelvis = MappingEntry(
        "Hips", "pelvis",
        offset=Offset(orientation=(0.609, 0.359, 0.359, -0.609)),
        weight=Weight(position=10.0, orientation=1.0),
    )
    left_hand = MappingEntry(
        "LeftHand", "left_rubber_hand",
        offset=Offset(orientation=(0.5, 0.5, -0.5, 0.5)),
        weight=Weight(position=10.0, orientation=1.0),
    )
    right_hand = MappingEntry(
        "RightHand", "right_rubber_hand",
        offset=Offset(orientation=(0.5, 0.5, -0.5, 0.5)),
        weight=Weight(position=10.0, orientation=1.0),
    )
    left_hip = MappingEntry(
        "LeftUpLeg", "left_hip_roll_link",
    )
    right_hip = MappingEntry(
        "RightUpLeg", "right_hip_roll_link",
    )
    left_foot = MappingEntry(
        "LeftToeBase", "left_ankle_roll_link",
        offset=Offset(orientation=(0.062, -0.704, -0.704, -0.062)),
    )
    right_foot = MappingEntry(
        "RightToeBase", "right_ankle_roll_link",
        offset=Offset(orientation=(0.062, -0.704, -0.704, -0.062)),
    )
    left_shoulder = MappingEntry(
        "LeftShoulder", "left_shoulder_roll_link",
    )
    right_shoulder = MappingEntry(
        "RightShoulder", "right_shoulder_roll_link",
    )
    left_elbow = MappingEntry(
        "LeftForeArm", "left_elbow_link",
    )
    right_elbow = MappingEntry(
        "RightForeArm", "right_elbow_link",
    )
    left_knee = MappingEntry(
        "LeftLeg", "left_knee_link",
    )
    right_knee = MappingEntry(
        "RightLeg", "right_knee_link",
    )


# G1_ACCAD_MAPPING = {
#     # "spine1":           {"bone": "ToSpine", "func": lambda b: b.tail },  # 3
#     "left_knee":        {"bone": "LeftLeg",      },  # 4
#     "right_knee":       {"bone": "RightLeg",      },  # 5
#     "spine2":           {"bone": "Spine",     },  # 6
#     "left_ankle":       {"bone": "LeftFoot", "quat": [0.123, 0.696, 0.696, -0.123]},  # 7
#     "right_ankle":      {"bone": "RightFoot", "quat": [0.123, 0.696, 0.696, -0.123]},  # 8
#     "spine3":           {"bone": "Spine1",     },  # 9
#     "left_foot":        {"bone": "LeftToeBase", "quat": [0.062, -0.704, -0.704, -0.062]},  # 10
#     "right_foot":       {"bone": "RightToeBase", "quat": [0.062, -0.704, -0.704, -0.062]},  # 11
#     "neck":             {"bone": "Neck",          },  # 12
#     "left_collar":      {"bone": "LeftShoulder",        },  # 13
#     "right_collar":     {"bone": "RightShoulder",        },  # 14
#     "head":             {"bone": "Head", "quat": [0.455, -0.542, -0.542, -0.455]},  # 15
#     "left_shoulder":    {"bone": "LeftArm",        },  # 16
#     "right_shoulder":   {"bone": "RightArm",        },  # 17
#     "left_elbow":       {"bone": "LeftForeArm",      },  # 18
#     "right_elbow":      {"bone": "RightForeArm",      },  # 19
#     "left_wrist":       {"bone": "LeftHand", "quat": [0.5, 0.5, -0.5, 0.5]},  # 20
#     "right_wrist":      {"bone": "RightHand", "quat": [0.5, 0.5, -0.5, 0.5]},  # 21
#     "left_hand":        {"bone": "LeftHand", "func": lambda b: b.tail, "quat": [0.5, 0.5, -0.5, 0.5]},  # 22
#     "right_hand":       {"bone": "RightHand", "func": lambda b: b.tail, "quat": [0.5, 0.5, -0.5, 0.5]},  # 23
#     }


class G1ActorCoreMapping:
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
    pass


class G1MMDYYBMapping:
    mmd_yyb = {
        "pelvis":           {"bone": "腰", "quat": [-0.327, 0.627, 0.627, 0.327]},  # 0
        "left_hip":         {"bone": "足.L",        },  # 1
        "right_hip":        {"bone": "足.R",        },  # 2
        "spine1":           {"bone": "上半身", "func": lambda b: b.tail },  # 3
        "left_knee":        {"bone": "ひざ.L",      },  # 4
        "right_knee":       {"bone": "ひざ.R",      },  # 5
        "spine2":           {"bone": "上半身2",     },  # 6
        "left_ankle":       {"bone": "足首.L", "quat": [0.153, 0.690, 0.690, -0.153]},  # 7
        "right_ankle":      {"bone": "足首.R", "quat": [0.153, 0.690, 0.690, -0.153]},  # 8
        "spine3":           {"bone": "上半身2",     },  # 9
        "left_foot":        {"bone": "足先EX.L", "quat": [0.0, 0.707, 0.707, 0.0]},  # 10
        "right_foot":       {"bone": "足先EX.R", "quat": [0.0, 0.707, 0.707, 0.0]},  # 11
        "neck":             {"bone": "首",          },  # 12
        "left_collar":      {"bone": "肩.L",        },  # 13
        "right_collar":     {"bone": "肩.R",        },  # 14
        "head":             {"bone": "頭",          },  # 15
        "left_shoulder":    {"bone": "腕.L",        },  # 16
        "right_shoulder":   {"bone": "腕.R",        },  # 17
        "left_elbow":       {"bone": "ひじ.L",      },  # 18
        "right_elbow":      {"bone": "ひじ.R",      },  # 19
        "left_wrist":       {"bone": "手首.L", "quat": [0.707, 0.707, 0.0, 0.0]},  # 20
        "right_wrist":      {"bone": "手首.R", "quat": [0.707, 0.707, 0.0, 0.0]},  # 21
        "left_hand":        {"bone": "手首.L", "func": lambda b: b.tail, "quat": [0.707, 0.707, 0.0, 0.0]},  # 22
        "right_hand":       {"bone": "手首.R", "func": lambda b: b.tail, "quat": [0.707, 0.707, 0.0, 0.0]},  # 23
    }


class G1MixamoMapping:
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
    pass


class G1MeshCapadeMapping:
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
    pass
