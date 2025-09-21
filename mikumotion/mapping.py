"""
Bone mapping between different motion capture systems.

Each mapping contains a set of transforms from the source bone to the target bone, with
optional position and rotation offsets. The IK weights (costs) are also included.

The position and rotation offsets are the relative offsets between the source bone
and the target bone in global coordinate frame.

Generic bone names are based on SMPL-X body names,
see https://github.com/vchoutas/smplx/blob/main/smplx/joint_names.py#L244C21-L268C18
and https://chingswy.github.io/easymocap-public-doc/database/2_keypoints.html#smpl
"""

# Note: We decide to use the configclass based system to unclutter the code.
# Compared to using YAML file or Python dictionary, configclass allows initialization
# of default values and type hinting/checking for each field.

from configclass import configclass


@configclass
class Offset:
    """The offset from the source body to the target body in the world frame."""

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """The position offset in (x, y, z) in meters."""

    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    """The orientation offset in (qw, qx, qy, qz) quaternion."""


@configclass
class Weight:
    """IK solver weights for the position and orientation of the target body."""

    position: float = 0.0
    """Position weight for the IK solver."""

    orientation: float = 0.0
    """Orientation weight for the IK solver."""


@configclass
class MappingEntry:
    """The mapping entry for a target body."""

    source: str = ""
    """The source bone name."""

    target: str = ""
    """The target bone name."""

    offset: Offset = Offset(position=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0))
    """The offset from the source body to the target body in the world frame, default to zero."""

    weight: Weight = Weight(position=0.0, orientation=0.0)
    """The IK solver weights for the position and orientation of the target body, default to zero."""


@configclass
class GenericMapping:
    """The generic mapping for all the bodies."""

    pelvis: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the pelvis body, ID = 0."""

    left_hip: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left hip body, ID = 1."""

    right_hip: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right hip body, ID = 2."""

    spine1: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the spine1 body, ID = 3."""

    left_knee: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left knee body, ID = 4."""

    right_knee: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right knee body, ID = 5."""

    spine2: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the spine2 body, ID = 6."""

    left_ankle: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left ankle body, ID = 7."""

    right_ankle: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right ankle body, ID = 8."""

    spine3: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the spine3 body, ID = 9."""

    left_foot: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left foot body, ID = 10."""

    right_foot: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right foot body, ID = 11."""

    neck: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the neck body, ID = 12."""

    left_collar: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left collar body, ID = 13."""

    right_collar: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right collar body, ID = 14."""

    head: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the head body, ID = 15."""

    left_shoulder: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left shoulder body, ID = 16."""

    right_shoulder: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right shoulder body, ID = 17."""

    left_elbow: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left elbow body, ID = 18."""

    right_elbow: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right elbow body, ID = 19."""

    left_wrist: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left wrist body, ID = 20."""

    right_wrist: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right wrist body, ID = 21."""

    left_hand: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the left hand body, ID = 22."""

    right_hand: MappingEntry = MappingEntry(source="", target="")
    """The mapping for the right hand body, ID = 23."""

    def astable(self) -> str:
        data_str = ""
        data_str += "|----------------|------------------------------------|------------------------------------------------|------------|\n"
        data_str += "| Body           | Mapping                            | Offset                                         | Weight     |\n"
        data_str += "|----------------|------------------------------------|------------------------------------------------|------------|\n"
        for key, value in self.__dict__.items():
            if not value.source:
                # skip the body that is not mapped
                continue
            key_str = key.ljust(14)
            map_str = f"{value.source} -> {value.target}".ljust(34)
            offset_str = f"{value.offset.position}, {value.offset.orientation}".ljust(46)
            weight_str = f"{value.weight.position}, {value.weight.orientation}".ljust(10)
            data_str += f"| {key_str} | {map_str} | {offset_str} | {weight_str} |\n"
        return data_str

    def asdict(self) -> dict[str, dict[str, str | dict[str, tuple[float]]]]:
        data_dict = {}
        for key, value in self.__dict__.items():
            if not value.source:
                # skip the body that is not mapped
                continue
            data_dict[key] = {
                "source": value.source,
                "target": value.target,
                "offset": {
                    "position": value.offset.position,
                    "orientation": value.offset.orientation,
                },
                "weight": {
                    "position": value.weight.position,
                    "orientation": value.weight.orientation,
                },
            }
        return data_dict
