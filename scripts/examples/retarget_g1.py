import argparse

from mikumotion.motion_retargeting import MotionRetargeting


parser = argparse.ArgumentParser()
parser.add_argument("--motion", type=str, help="Source motion file")
parser.add_argument("--robot", type=str, help="Robot name")
parser.add_argument("--realtime", action="store_true", default=False, help="Visualize in realtime")
args = parser.parse_args()


if __name__ == "__main__":
    motion_file = args.motion
    match args.robot:
        case "unitree_g1":
            robot_xml = "./data/robots/unitree/g1/mjcf/g1_29dof_mode_5_mocap.xml"
            mapping_table = {
                "pelvis": {
                    "body": "pelvis",
                    "weight": {
                        "position": 10.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_rubber_hand": {
                    "body": "left_hand",
                    "weight": {
                        "position": 1.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [0.707, 0.0, 0.707, 0.0],
                    }
                },
                "right_rubber_hand": {
                    "body": "right_hand",
                    "weight": {
                        "position": 1.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [0.707, 0.0, 0.707, 0.0],
                    }
                },
                "left_ankle_roll_link": {
                    "body": "left_foot",
                    "weight": {
                        "position": 1.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_ankle_roll_link": {
                    "body": "right_foot",
                    "weight": {
                        "position": 1.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_shoulder_roll_link": {
                    "body": "left_shoulder",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_elbow_link": {
                    "body": "left_elbow",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_shoulder_roll_link": {
                    "body": "right_shoulder",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_elbow_link": {
                    "body": "right_elbow",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_hip_roll_link": {
                    "body": "left_hip",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_hip_roll_link": {
                    "body": "right_hip",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_knee_link": {
                    "body": "left_knee",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_knee_link": {
                    "body": "right_knee",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
            }
        case "hxg_humanoid":
            robot_xml = "/home/tk/Desktop/AIR/source/air/data/robots/hxg/hxg_humanoid/mjcf/scene_mocap.xml"
            mapping_table = {
                "pelvis_link": {
                    "body": "pelvis",
                    "weight": {
                        "position": 10.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_elbow_yaw_link": {
                    "body": "left_hand",
                    "weight": {
                        "position": 10.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        # "orientation": [0.5, 0.5, -0.5, -0.5],
                        "orientation": [0.271, 0.271, -0.653, -0.653],
                    }
                },
                "right_elbow_yaw_link": {
                    "body": "right_hand",
                    "weight": {
                        "position": 10.0,
                        "orientation": 1.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        # "orientation": [0.5, -0.5, 0.5, -0.5],
                        "orientation": [0.271, -0.271, 0.653, -0.653],
                    }
                },
                "left_foot_link": {
                    "body": "left_ankle",
                    "weight": {
                        "position": 10.0,
                        "orientation": 2.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, -0.1],
                        "orientation": [0.707, 0.0, -0.707, 0.0],
                    }
                },
                "right_foot_link": {
                    "body": "right_ankle",
                    "weight": {
                        "position": 10.0,
                        "orientation": 2.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, -0.1],
                        "orientation": [0.707, 0.0, -0.707, 0.0],
                    }
                },
                "left_shoulder_pitch_link": {
                    "body": "left_shoulder",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.00],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_elbow_pitch_link": {
                    "body": "left_elbow",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_shoulder_pitch_link": {
                    "body": "right_shoulder",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_elbow_pitch_link": {
                    "body": "right_elbow",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_thigh_link": {
                    "body": "left_hip",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_thigh_link": {
                    "body": "right_hip",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "left_shin_link": {
                    "body": "left_knee",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
                "right_shin_link": {
                    "body": "right_knee",
                    "weight": {
                        "position": 1.0,
                        "orientation": 0.0,
                    },
                    "offset": {
                        "position": [0.0, 0.0, 0.0],
                        "orientation": [1.0, 0.0, 0.0, 0.0],
                    }
                },
            }
        case _:
            raise ValueError(f"Unsupported robot: {args.robot}")

    retargeting = MotionRetargeting(motion_file, robot_xml, mapping_table)

    retargeting.run(realtime=args.realtime)
