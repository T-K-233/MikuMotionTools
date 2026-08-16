"""
Minimal URDF parser used to build Blender armatures and meshes.

This module is intentionally dependency-light (only the standard library and
numpy) so that it can be imported both from a normal Python environment and
from inside Blender's bundled Python.

It parses the kinematic structure (links + joints) and the visual geometry
(mesh files, their local placement, scale, and color) of a robot. The parsed
``RobotModel`` can be converted into an :class:`~mikumotion.armature_tree.ArmatureTree`
(link tree) via :meth:`RobotModel.to_armature_tree`, which the existing
``blender.build_armature`` helper already knows how to consume.

URDF convention notes:
    - A joint's ``<origin>`` describes the child link frame relative to the
      parent link frame at zero joint position.
    - ``rpy`` is a fixed-axis (extrinsic XYZ) rotation, i.e. the rotation matrix
      is ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``. This matches
      ``mikumotion.math.euler_xyz_to_quat``.
    - A revolute joint rotates about ``<axis>``, expressed in the (child) joint
      frame.
"""

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


def _parse_vec(text: Optional[str], default: Tuple[float, ...]) -> np.ndarray:
    """Parse a whitespace-separated float vector, falling back to ``default``."""
    if text is None or text.strip() == "":
        return np.array(default, dtype=np.float32)
    return np.fromstring(text, dtype=np.float32, sep=" ")


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two ``(w, x, y, z)`` quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def rpy_to_quat(rpy: np.ndarray) -> np.ndarray:
    """
    Convert a URDF ``rpy`` (roll, pitch, yaw) into a ``(w, x, y, z)`` quaternion.

    URDF ``rpy`` is a fixed-axis rotation (roll about X, then pitch about Y, then
    yaw about Z, all about the fixed parent axes), giving ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
    The equivalent quaternion is ``qz * qy * qx`` (Hamilton product).
    """
    roll, pitch, yaw = (float(v) for v in rpy)
    qx = np.array([np.cos(roll / 2), np.sin(roll / 2), 0.0, 0.0])
    qy = np.array([np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0])
    qz = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
    return _quat_mul(qz, _quat_mul(qy, qx)).astype(np.float32)


@dataclass
class Visual:
    """A single visual geometry attached to a link."""

    mesh_path: Optional[str]                                # resolved absolute path to the mesh file, or None
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    scale: np.ndarray = field(default_factory=lambda: np.ones(3, dtype=np.float32))
    rgba: Optional[np.ndarray] = None                       # (4,) color or None if unspecified
    material_name: Optional[str] = None


@dataclass
class Link:
    """A rigid body link."""

    name: str
    visuals: List[Visual] = field(default_factory=list)


@dataclass
class Joint:
    """A joint connecting a parent link to a child link."""

    name: str
    type: str                                               # revolute, continuous, prismatic, fixed, ...
    parent: str
    child: str
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0], dtype=np.float32))

    @property
    def origin_quat(self) -> np.ndarray:
        """Origin rotation as a ``(w, x, y, z)`` quaternion."""
        return rpy_to_quat(self.origin_rpy)


class RobotModel:
    """A parsed URDF robot: links, joints, and visual geometry."""

    def __init__(self, name: str, links: Dict[str, Link], joints: List[Joint]):
        self.name = name
        self.links = links
        self.joints = joints
        # map a child link name to the joint that produces it (every non-root link has exactly one)
        self._child_to_joint: Dict[str, Joint] = {j.child: j for j in joints}

    def __repr__(self) -> str:
        return f"RobotModel(name={self.name!r}, num_links={len(self.links)}, num_joints={len(self.joints)})"

    # ------------------------------------------------------------------ topology

    def root_link(self) -> str:
        """Return the name of the root link (the one that is never a joint child)."""
        children = {j.child for j in self.joints}
        roots = [name for name in self.links if name not in children]
        if len(roots) != 1:
            raise ValueError(f"Expected exactly one root link, found {roots}")
        return roots[0]

    def parent_joint(self, link_name: str) -> Optional[Joint]:
        """Return the joint whose child is ``link_name`` (None for the root link)."""
        return self._child_to_joint.get(link_name)

    def child_joints(self, link_name: str) -> List[Joint]:
        """Return the joints whose parent is ``link_name``."""
        return [j for j in self.joints if j.parent == link_name]

    def ordered_link_names(self) -> List[str]:
        """Return link names in topological order (root first, parents before children)."""
        root = self.root_link()
        ordered: List[str] = []
        stack = [root]
        while stack:
            name = stack.pop()
            ordered.append(name)
            # push children; reverse so siblings keep their document order
            children = [j.child for j in self.child_joints(name)]
            stack.extend(reversed(children))
        return ordered

    # ------------------------------------------------------------------ conversion

    def to_armature_tree(self):
        """
        Convert to an :class:`~mikumotion.armature_tree.ArmatureTree`.

        Each link becomes a body; its local translation/rotation is taken from the
        parent joint's ``<origin>`` (the root link gets identity).
        """
        from .armature_tree import ArmatureTree

        ordered = self.ordered_link_names()
        index_of = {name: i for i, name in enumerate(ordered)}

        body_parent_indices = []
        local_translations = []
        local_rotations = []

        for name in ordered:
            joint = self.parent_joint(name)
            if joint is None:
                body_parent_indices.append(-1)
                local_translations.append(np.zeros(3, dtype=np.float32))
                local_rotations.append(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
            else:
                body_parent_indices.append(index_of[joint.parent])
                local_translations.append(joint.origin_xyz.astype(np.float32))
                local_rotations.append(joint.origin_quat)

        return ArmatureTree(
            body_names=ordered,
            body_parent_indices=np.array(body_parent_indices, dtype=np.int32),
            local_translations=np.array(local_translations, dtype=np.float32),
            local_rotations=np.array(local_rotations, dtype=np.float32),
        )

    # ------------------------------------------------------------------ parsing

    @classmethod
    def from_file(cls, path: str, mesh_package_roots: Optional[Dict[str, str]] = None) -> "RobotModel":
        """
        Parse a URDF file.

        Args:
            path: Path to the ``.urdf`` file.
            mesh_package_roots: Optional mapping ``{package_name: absolute_dir}`` used to
                resolve ``package://`` mesh paths. If a package is not listed, it is resolved
                relative to the URDF file's parent directory.

        Returns:
            The parsed :class:`RobotModel`.
        """
        urdf_dir = os.path.dirname(os.path.abspath(path))
        tree = ET.parse(path)
        root = tree.getroot()

        name = root.attrib.get("name", "robot")

        # collect top-level <material> definitions so links can reference them by name
        global_materials: Dict[str, np.ndarray] = {}
        for mat in root.findall("material"):
            color = mat.find("color")
            if color is not None:
                global_materials[mat.attrib["name"]] = _parse_vec(color.attrib.get("rgba"), (0.8, 0.8, 0.8, 1.0))

        links: Dict[str, Link] = {}
        for xml_link in root.findall("link"):
            link_name = xml_link.attrib["name"]
            link = Link(name=link_name)

            for xml_visual in xml_link.findall("visual"):
                visual = cls._parse_visual(xml_visual, urdf_dir, mesh_package_roots, global_materials)
                link.visuals.append(visual)

            links[link_name] = link

        joints: List[Joint] = []
        for xml_joint in root.findall("joint"):
            origin = xml_joint.find("origin")
            axis = xml_joint.find("axis")
            joints.append(Joint(
                name=xml_joint.attrib["name"],
                type=xml_joint.attrib.get("type", "fixed"),
                parent=xml_joint.find("parent").attrib["link"],
                child=xml_joint.find("child").attrib["link"],
                origin_xyz=_parse_vec(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0)),
                origin_rpy=_parse_vec(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0)),
                axis=_parse_vec(axis.attrib.get("xyz") if axis is not None else None, (1, 0, 0)),
            ))

        return cls(name=name, links=links, joints=joints)

    @staticmethod
    def _parse_visual(
        xml_visual: ET.Element,
        urdf_dir: str,
        mesh_package_roots: Optional[Dict[str, str]],
        global_materials: Dict[str, np.ndarray],
    ) -> Visual:
        origin = xml_visual.find("origin")
        origin_xyz = _parse_vec(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0))
        origin_rpy = _parse_vec(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0))

        mesh_path: Optional[str] = None
        scale = np.ones(3, dtype=np.float32)
        mesh = xml_visual.find("geometry/mesh")
        if mesh is not None:
            mesh_path = _resolve_mesh_path(mesh.attrib.get("filename", ""), urdf_dir, mesh_package_roots)
            scale = _parse_vec(mesh.attrib.get("scale"), (1, 1, 1))

        rgba: Optional[np.ndarray] = None
        material_name: Optional[str] = None
        material = xml_visual.find("material")
        if material is not None:
            material_name = material.attrib.get("name")
            color = material.find("color")
            if color is not None:
                rgba = _parse_vec(color.attrib.get("rgba"), (0.8, 0.8, 0.8, 1.0))
            elif material_name in global_materials:
                rgba = global_materials[material_name]

        return Visual(
            mesh_path=mesh_path,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            scale=scale,
            rgba=rgba,
            material_name=material_name,
        )


def _resolve_mesh_path(
    filename: str,
    urdf_dir: str,
    mesh_package_roots: Optional[Dict[str, str]],
) -> Optional[str]:
    """Resolve a URDF mesh ``filename`` to an absolute filesystem path."""
    if not filename:
        return None

    if filename.startswith("package://"):
        rest = filename[len("package://"):]
        package_name, _, relative = rest.partition("/")
        if mesh_package_roots and package_name in mesh_package_roots:
            return os.path.normpath(os.path.join(mesh_package_roots[package_name], relative))
        # fall back: treat the package root as the URDF directory's parent
        return os.path.normpath(os.path.join(os.path.dirname(urdf_dir), package_name, relative))

    if filename.startswith("file://"):
        filename = filename[len("file://"):]

    if os.path.isabs(filename):
        return os.path.normpath(filename)

    return os.path.normpath(os.path.join(urdf_dir, filename))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Inspect a URDF file.")
    parser.add_argument("--urdf", type=str, required=True, help="Path to the .urdf file")
    args = parser.parse_args()

    np.set_printoptions(precision=4, suppress=True)

    robot = RobotModel.from_file(args.urdf)
    print(robot)
    print("root link:", robot.root_link())

    armature_tree = robot.to_armature_tree()
    print(f"armature tree: {armature_tree.num_bodies} bodies")

    num_visuals = sum(len(link.visuals) for link in robot.links.values())
    num_meshes = sum(1 for link in robot.links.values() for v in link.visuals if v.mesh_path)
    print(f"visuals: {num_visuals} ({num_meshes} with meshes)")
