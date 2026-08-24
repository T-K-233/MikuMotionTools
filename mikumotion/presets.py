"""
The bone and body maps that configure the pipeline. Data only: no logic lives here.

Body names follow the SMPL-X convention, and every table is keyed by it, so one name means the
same body part from a character's rig through to a robot's link:

    CHARACTER_RIGS               a character's armature -> the mocap rig
    MOCAP_RIGS                   the mocap rig -> a motion
    RETARGET_MAPS                a motion -> one robot's joints
    LITE_PRO_TO_VROID_BONE_MAP   a robot's links -> a VRM character

The first two are read by the ``convert_*`` scripts in ``scripts/blender/``, the third by
``mikumotion retarget``, and the last by ``convert_motion_to_character.py``.

SMPL-X joint names:
https://github.com/vchoutas/smplx/blob/main/smplx/joint_names.py#L244C21-L268C18
https://chingswy.github.io/easymocap-public-doc/database/2_keypoints.html#smpl
"""

import numpy as np


# ============================================================
# character armature  ->  mocap rig  (direction 1, before the export)
# ============================================================
# A character arrives on the armature its authoring tool gave it: MMD, Mixamo, ActorCore,
# Meshcapade. convert_character_to_mocap.py bakes that animation onto the small MOCAP_RIGS
# armature, and convert_mocap_to_motion.py reads that. The mocap rig's bones carry ``.frame``
# children whose orientation, not the head-to-tail axis of the bone itself, is what the export
# records.
#
# The step exists for a rig that cannot carry those helper bones itself. An MMD model has
# hundreds of bones under Japanese names, with local axes that suit MMD, so adding a helper to
# each one is impractical. The accad and actorcore rigs carry their own helpers and go straight
# to the export, which is why they have no entry here.
#
# One bone map per character, named for the format it reads, the same way RETARGET_MAPS holds
# ZAMUZA_TO_LITE_PRO. Keys are mocap bones, values are the character's own, the same direction
# as LITE_PRO_TO_VROID_BONE_MAP.

MMD_TO_ZAMUZA_BONE_MAP = {
    "pelvis": "下半身",
    "spine1": "上半身2",
    "head": "頭",
    "left_shoulder": "腕.L",
    "right_shoulder": "腕.R",
    "left_elbow": "ひじ.L",
    "right_elbow": "ひじ.R",
    "left_hand": "手首.L",
    "right_hand": "手首.R",
    "left_hip": "足.L",
    "right_hip": "足.R",
    "left_knee": "ひざ.L",
    "right_knee": "ひざ.R",
    "left_foot": "足首.L",
    "right_foot": "足首.R",
}

#: Which character armature each preset bakes from, keyed the same as MOCAP_RIGS. A preset
#: appears here only if its rig needs the bake. ``translation_root`` is the character bone whose
#: motion places the whole rig in the world, which differs per format: MMD centres a rig on
#: センター, Mixamo on mixamorig:Hips.
CHARACTER_RIGS = {
    "zamuza": {
        "armature": "YYB式初音ミクv1.02_arm",
        "bone_map": MMD_TO_ZAMUZA_BONE_MAP,
        "translation_root": "センター",
    },
}


# ============================================================
# Blender mocap armature  ->  motion  (direction 1: animation -> robot)
# ============================================================
# One entry per mocap rig, read by convert_mocap_to_motion.py and convert_character_to_mocap.py.
# Bones are listed in SMPL keypoint order, so the same index means the same body part across
# every rig. Only the names are the rig's own: zamuza uses the SMPL-X joint names because it is
# a rig built for this pipeline, while accad and actorcore keep the names their mocap source
# shipped.
# ``scale`` converts the rig's units to metres; ``rotate_z`` turns it to +X forward,
# since Blender rigs are authored +Y forward.

MOCAP_RIGS = {
    "zamuza": {
        "armature": "Armature",
        "scale": 1.0,
        "rotate_z": 0.0,
        "frames": (0, 2000),
        # this rig has a single spine bone, so it fills the spine1 slot only; listing it
        # twice would export the same pose under one name twice, which the store rejects
        "bones": [
            "pelvis", "left_hip", "right_hip", "spine1",
            "left_knee", "right_knee",
            "left_foot", "right_foot", "head",
            "left_shoulder", "right_shoulder",
            "left_elbow", "right_elbow",
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
# motion  ->  robot joints  (direction 1, the IK step)
# ============================================================
# ``{source_body: (target_body, orientation_cost)}`` for MotionRetargetingIK. The source
# names are the mocap bodies of a MOCAP_RIGS rig; the targets are links of the robot
# being solved for. A high orientation cost pins a limb's pose; a low one lets IK swing
# the segment freely and only uses it to steer the elbow/knee (a pole target).

ZAMUZA_TO_LITE_PRO = {
    "pelvis": ("pelvis", 0.5),
    "spine1": ("chest", 0.5),
    "head": ("head", 0.5),
    "left_hand": ("left_hand", 0.5),
    "right_hand": ("right_hand", 0.5),
    "left_foot": ("left_foot", 0.5),
    "right_foot": ("right_foot", 0.5),
    "left_shoulder": ("left_shoulder_yaw", 0.1),
    "right_shoulder": ("right_shoulder_yaw", 0.1),
    "left_elbow": ("left_elbow_pitch", 0.1),
    "right_elbow": ("right_elbow_pitch", 0.1),
    "left_hip": ("left_hip_yaw", 0.1),
    "right_hip": ("right_hip_yaw", 0.1),
    "left_knee": ("left_knee_pitch", 0.1),
    "right_knee": ("right_knee_pitch", 0.1),
}

RETARGET_MAPS = {
    "ZAMUZA_TO_LITE_PRO": ZAMUZA_TO_LITE_PRO,
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
