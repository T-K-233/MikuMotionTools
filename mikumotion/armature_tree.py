# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import xml.etree.ElementTree as ET
from typing import List

import numpy as np


class ArmatureTree:
    """
    A complete description of a rigid skeleton, as a tree over named nodes. Each node is one
    body. Each edge carries a local translation, which is the distance between the two nodes
    that the edge connects.

    Derived from NVIDIA CALM poselib:
    https://github.com/NVlabs/CALM/blob/main/calm/poselib/poselib/skeleton/skeleton3d.py
    """

    def __init__(
        self,
        body_names: List[str],
        body_parent_indices: np.ndarray,
        local_translations: np.ndarray,
        local_rotations: np.ndarray,
    ):
        """
        Build the tree from one array per property, all in body order.

        Args:
            body_names: a list of length NUM_BODIES, containing the names for each body.
            body_parent_indices: an int32-typed numpy array of length NUM_BODIES that represents the edge to its parent.
            -1 represents the root body.
            local_translations: a numpy array of shape (NUM_BODIES, 3) that gives local translation information in
            meters.
            local_rotations: a numpy array of shape (NUM_BODIES, 4) that gives local rotation information in
            quaternion (w, x, y, z) format.
        """
        assert len(body_names) == len(body_parent_indices) == len(local_translations) == len(local_rotations), \
            "The number of bodies, body parent indices, local translations, and local rotations must be the same."

        self._body_names = body_names
        self._body_indices = {body_names[i]: i for i in range(len(body_names))}
        self._body_parent_indices = body_parent_indices.astype(np.int32)
        self._local_translations = local_translations.astype(np.float32)
        self._local_rotations = local_rotations.astype(np.float32)

    def __len__(self) -> int:
        """Number of bodies in the armature tree."""
        return len(self.body_names)

    def __getitem__(self, item: int) -> str:
        """ Get the name of the body given the index. """
        return self.body_names[item]

    def __repr__(self) -> str:
        return (
            """ArmatureTree(
    body_names={},
    body_parent_indices={},
    local_translations={},
    local_rotations={},
)
""".format(
                repr(self.body_names),
                repr(self.body_parent_indices),
                repr(self.local_translations),
                repr(self.local_rotations),
            )
        )

    @property
    def num_bodies(self) -> int:
        """Number of bodies in the armature tree."""
        return len(self)

    @property
    def body_names(self) -> List[str]:
        """List of body names."""
        return self._body_names

    @property
    def body_parent_indices(self) -> np.ndarray:
        """Array of body parent indices."""
        return self._body_parent_indices

    @property
    def local_translations(self) -> np.ndarray:
        """Array of local translations in meters."""
        return self._local_translations

    @property
    def local_rotations(self) -> np.ndarray:
        """Array of local rotations, as quaternions."""
        return self._local_rotations

    def get_index(self, body_name: str) -> int:
        """
        Get the index of the given body.

        Args:
            body_name: the name of the body
        Returns:
            The index of the given body
        """
        return self._body_indices[body_name]

    def get_parent_index(self, body_name: str) -> int:
        """
        Get the index of the parent of the given body.

        Args:
            body_name: the name of the body
        Returns:
            The index of the parent of the given body
        """
        return self._body_parent_indices[self.get_index(body_name)]

    def get_parent_name(self, body_name: str) -> str | None:
        """
        Get the name of the parent of the given body.

        Args:
            body_name: the name of the body
        Returns:
            The name of the parent of the given body
        """
        parent_index = self.get_parent_index(body_name)
        if parent_index == -1:
            return None  # root
        return self._body_names[parent_index]

    @classmethod
    def from_mjcf(cls, path: str) -> "ArmatureTree":
        """
        Parses a mujoco xml scene description file and returns an Armature Tree.
        We use the model attribute at the root as the name of the tree.

        Args:
            path: the path to the mjcf file
        Returns:
            The armature tree constructed from the mjcf file
        """
        tree = ET.parse(path)
        xml_doc_root = tree.getroot()
        xml_world_body = xml_doc_root.find("worldbody")
        if xml_world_body is None:
            raise ValueError("Fail to parse MJCF file: cannot find <worldbody>.")

        # assume this is the root
        xml_root_body = xml_world_body.find("body")
        if xml_root_body is None:
            raise ValueError("Fail to parse MJCF file: cannot find any <body> under <worldbody>.")

        body_names = []
        body_parent_indices = []
        local_translations = []
        local_rotations = []

        # recursively walk through the XML tree and add all bodies
        def _add_body_from_xml(xml_node, body_parent_index, body_index):
            body_name = xml_node.attrib.get("name")
            position_offset = np.fromstring(xml_node.attrib.get("pos", "0 0 0"), dtype=np.float32, sep=" ")
            rotation_offset = np.fromstring(xml_node.attrib.get("quat", "1 0 0 0"), dtype=np.float32, sep=" ")

            body_names.append(body_name)
            body_parent_indices.append(body_parent_index)
            local_translations.append(position_offset)
            local_rotations.append(rotation_offset)

            current_body_index = body_index
            body_index += 1
            for next_node in xml_node.findall("body"):
                body_index = _add_body_from_xml(next_node, current_body_index, body_index)
            return body_index

        _add_body_from_xml(xml_root_body, -1, 0)

        return cls(
            body_names,
            np.array(body_parent_indices, dtype=np.int32),
            np.array(local_translations, dtype=np.float32),
            np.array(local_rotations, dtype=np.float32),
        )

    @classmethod
    def from_urdf(cls, path: str) -> "ArmatureTree":
        """
        Parses a URDF file and returns an Armature Tree describing its link structure.

        Each link becomes a body, and takes its local translation and rotation from
        the parent joint's ``<origin>``. The root link (the one that is never a joint
        child) becomes the tree root.

        Args:
            path: the path to the urdf file
        Returns:
            The armature tree constructed from the urdf file
        """
        from .urdf import RobotModel

        return RobotModel.from_file(path).to_armature_tree()


if __name__ == "__main__":
    xml_path = "/home/tk/Desktop/Pianist/source/pianist/data/robots/ude_better_dummy/mjcf/ude_dummy.xml"
    tree = ArmatureTree.from_mjcf(xml_path)
    print(tree)
