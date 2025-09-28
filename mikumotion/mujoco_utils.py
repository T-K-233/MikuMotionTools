import mujoco

from mikumotion.motion_sequence import MotionSequence


def create_empty_scene(show_world_frame: bool = False) -> str:
    """
    Create an empty MuJoCo scene.
    """
    world_frame_xml = ""

    if show_world_frame:
        world_frame_xml = """
<body name="world_frame">
  <geom name="world_frame_x_axis" type="cylinder" size="0.001 0.5" pos="0.5 0 0" quat="0.707107 0 0.707107 0" rgba="0.8 0.2 0.2 0.75" />
  <geom name="world_frame_y_axis" type="cylinder" size="0.001 0.5" pos="0 0.5 0" quat="0.707107 0.707107 0 0" rgba="0.2 0.8 0.2 0.75" />
  <geom name="world_frame_z_axis" type="cylinder" size="0.001 0.5" pos="0 0 0.5" quat="1 0 0 0" rgba="0.2 0.2 0.8 0.75" />
  <geom name="world_frame_center" type="sphere" size="0.01" rgba="1 1 1 1" />
  <site name="world_frame_site" size="0.01" rgba="1 1 1 1" />
</body>
"""

    xml = f"""
<mujoco model="scene">
  <compiler angle="radian" />
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072" />
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
             markrgb="0.8 0.8 0.8" width="300" height="300" />
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2" />
  </asset>
  <worldbody>
    <light pos="1 0 3.5" dir="0 0 -1" directional="true" />
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" />
    {world_frame_xml}
  </worldbody>
  <statistic center="0.0 0.0 1.0" extent="2.0" />
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0.9 0.9 0.9" />
    <rgba haze="0.15 0.25 0.35 1" />
    <global azimuth="-140" elevation="-20" />
  </visual>
</mujoco>"""

    return xml


def add_body_frames(
    base_xml: str,
    body_names: list[str],
    prefix: str = "",
    center_color: tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> str:
    """
    Add coordinate frames to the base XML scene.

    Args:
        base_xml: Base XML string with empty worldbody
        body_names: List of body names
        prefix: Prefix for the body names
        center_color: RGB color for the center sphere

    Returns:
        Complete XML string with frames added
    """
    # Find the insertion point (before </worldbody>)
    insertion_point = base_xml.find("  </worldbody>")
    if insertion_point == -1:
        raise ValueError("Could not find </worldbody> tag in base XML")

    axis_size = "0.004 0.05"
    axis_length = "0.05"
    center_size = "0.01"

    # Generate frame XML for each frame
    frames_xml = ""
    for frame_name in body_names:
        print(f"Adding frame for {frame_name}")
        center_color_string = f"{center_color[0]} {center_color[1]} {center_color[2]}"

        frame_xml = f"""
<!-- {frame_name} frame -->
<body name="{prefix}{frame_name}_frame" mocap="true">
  <!-- X-axis (red) -->
  <geom name="{prefix}{frame_name}_x_axis" type="cylinder" size="{axis_size}" pos="{axis_length} 0 0" quat="0.707107 0 0.707107 0" rgba="0.8 0.2 0.2 0.75" />
  <!-- Y-axis (green) -->
  <geom name="{prefix}{frame_name}_y_axis" type="cylinder" size="{axis_size}" pos="0 {axis_length} 0" quat="0.707107 0.707107 0 0" rgba="0.2 0.8 0.2 0.75" />
  <!-- Z-axis (blue) -->
  <geom name="{prefix}{frame_name}_z_axis" type="cylinder" size="{axis_size}" pos="0 0 {axis_length}" quat="1 0 0 0" rgba="0.2 0.2 0.8 0.75" />
  <!-- Center point -->
  <geom name="{prefix}{frame_name}_center" type="sphere" size="{center_size}" rgba="{center_color_string} 1" />
  <site name="{prefix}{frame_name}_site" size="{center_size}" rgba="{center_color_string} 1" />
</body>
"""
        frames_xml += frame_xml

    # Insert frames before </worldbody>
    complete_xml = base_xml[:insertion_point] + frames_xml + "\n" + base_xml[insertion_point:]
    return complete_xml
