"""
Scene XML for the retargeting viewer.

``MotionRetargetingIK --view`` shows a solve as two gizmos per tracked body: one where the link
is now, one where the IK pulls it. MuJoCo cannot add a body to a model it has already compiled,
so the gizmos are spliced into the robot's XML first.

This module imports no mujoco. The work is string assembly, and the caller compiles the result.
"""

#: One gizmo axis each: the name suffix, the offset from the body origin in metres, the rotation
#: that turns the cylinder onto that axis, and the colour.
AXES = (
    ("x", "0.05 0 0", "0.707107 0 0.707107 0", "0.8 0.2 0.2"),
    ("y", "0 0.05 0", "0.707107 0.707107 0 0", "0.2 0.8 0.2"),
    ("z", "0 0 0.05", "1 0 0 0", "0.2 0.2 0.8"),
)
AXIS_SIZE = "0.004 0.05"  # cylinder radius and half length, metres
CENTER_SIZE = "0.01"


def add_body_frames(base_xml, body_names, prefix, center_color):
    """
    Return ``base_xml`` with a mocap gizmo added for each of ``body_names``.

    Each gizmo is a mocap body, so the caller moves it every step. ``prefix`` keeps two sets
    apart in one scene, and ``center_color`` is the ``(r, g, b)`` that tells them apart on
    screen: the solve draws the links in cyan and the targets in magenta.
    """
    insertion_point = base_xml.find("  </worldbody>")
    if insertion_point == -1:
        raise ValueError("the base XML has no </worldbody>, so the frames have nowhere to go")

    rgb = " ".join(str(channel) for channel in center_color)
    frames = ""
    for name in body_names:
        body = f"{prefix}{name}"
        axes = "\n".join(
            f'  <geom name="{body}_{axis}_axis" type="cylinder" size="{AXIS_SIZE}"'
            f' pos="{pos}" quat="{quat}" rgba="{rgba} 0.75" />'
            for axis, pos, quat, rgba in AXES)
        frames += (
            f'\n<body name="{body}_frame" mocap="true">\n{axes}\n'
            f'  <geom name="{body}_center" type="sphere" size="{CENTER_SIZE}" rgba="{rgb} 1" />\n'
            f'  <site name="{body}_site" size="{CENTER_SIZE}" rgba="{rgb} 1" />\n'
            f'</body>\n')
    return base_xml[:insertion_point] + frames + "\n" + base_xml[insertion_point:]
