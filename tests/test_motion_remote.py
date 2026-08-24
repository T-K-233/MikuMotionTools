"""A remote mirrors the store's layout, so one relative path addresses both sides."""

import shutil
from pathlib import Path

import pytest

from mikumotion.motion_remote import (motion_layers, motion_paths, uri_to_remote)
from mikumotion.motion_sequence import MotionStore

#: A store with two source rigs, one motion solved for two robots, and two files that are not
#: motion data: the sources note that lives in git, and the metadata a pull leaves behind.
LISTING = [
    "reference/zamuza.rrd",
    "reference/zamuza_reset.rrd",
    "reference/lite_pro_tracking.rrd",
    "lite_pro/zamuza.rrd",
    "lite_pro/lite_pro_tracking.rrd",
    "g1/zamuza.rrd",
    "blueprints/lite_pro.rbl",
    "blueprints/g1.rbl",
    "MMD-Motion-Sources.md",
    ".cache/huggingface/download/reference/zamuza.rrd.metadata",
]

LAYERS = [
    "blueprints/g1.rbl",
    "blueprints/lite_pro.rbl",
    "g1/zamuza.rrd",
    "lite_pro/lite_pro_tracking.rrd",
    "lite_pro/zamuza.rrd",
    "reference/lite_pro_tracking.rrd",
    "reference/zamuza.rrd",
    "reference/zamuza_reset.rrd",
]


class DirectoryRemote:
    """
    A remote backed by a directory, inheriting nothing, as a backend does.

    It meets the contract with nothing but ``shutil``, which is the point: if three methods over
    relative paths are enough for a local directory, they are enough for a Hugging Face
    repository or an S3 bucket, and the tests need no network.
    """

    def __init__(self, location):
        self.root = Path(location)

    def paths(self):
        return [path.relative_to(self.root).as_posix()
                for path in self.root.rglob("*") if path.is_file()]

    def pull(self, paths, root):
        for path in paths:
            copy(self.root / path, Path(root) / path)

    def push(self, paths, root):
        for path in paths:
            copy(Path(root) / path, self.root / path)


def copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)


@pytest.fixture
def store(tmp_path):
    """A store holding :data:`LISTING`, each file carrying its own path as its content."""
    root = tmp_path / "motions"
    for path in LISTING:
        target = root / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(path.encode())
    return MotionStore(root)


def test_the_store_lists_every_file_it_holds(store):
    assert sorted(store.paths()) == sorted(LISTING)


def test_no_names_selects_the_whole_store():
    assert motion_paths(LISTING, []) == LAYERS


def test_a_name_selects_its_layers_and_the_blueprint_of_every_robot_it_reached():
    assert motion_paths(LISTING, ["zamuza"]) == [
        "blueprints/g1.rbl",
        "blueprints/lite_pro.rbl",
        "g1/zamuza.rrd",
        "lite_pro/zamuza.rrd",
        "reference/zamuza.rrd",
    ]


def test_a_blueprint_travels_only_with_the_robot_that_solved_the_motion():
    assert motion_paths(LISTING, ["lite_pro_tracking"]) == [
        "blueprints/lite_pro.rbl",
        "lite_pro/lite_pro_tracking.rrd",
        "reference/lite_pro_tracking.rrd",
    ]


def test_a_rest_pose_is_its_own_motion():
    """``retarget`` needs ``<name>_reset``, but naming a motion never implies another one."""
    assert motion_paths(LISTING, ["zamuza_reset"]) == ["reference/zamuza_reset.rrd"]


def test_a_listing_reads_as_motions_and_the_robots_each_was_solved_for():
    """A blueprint carries a robot's name, which must never appear as a motion."""
    assert motion_layers(LISTING) == {
        "lite_pro_tracking": ["lite_pro", "reference"],
        "zamuza": ["g1", "lite_pro", "reference"],
        "zamuza_reset": ["reference"],
    }


def test_an_unknown_name_selects_nothing():
    assert motion_paths(LISTING, ["not_a_motion"]) == []


def push(store, remote, names):
    """What ``mikumotion push`` does: select from the store, hand the paths to the remote."""
    paths = motion_paths(store.paths(), names)
    remote.push(paths, store.root)
    return paths


def pull(remote, store, names):
    """What ``mikumotion pull`` does: select from the remote, hand the paths to it."""
    paths = motion_paths(remote.paths(), names)
    remote.pull(paths, store.root)
    return paths


def test_a_push_and_a_pull_reproduce_the_store_exactly(store, tmp_path):
    remote = DirectoryRemote(tmp_path / "remote")
    assert push(store, remote, []) == LAYERS

    pulled = MotionStore(tmp_path / "pulled")
    assert pull(remote, pulled, []) == LAYERS
    for path in LAYERS:
        assert (pulled.root / path).read_bytes() == (store.root / path).read_bytes()


def test_nothing_but_motion_data_travels(store, tmp_path):
    """The sources note belongs to git, and the download metadata to the machine that pulled."""
    remote = DirectoryRemote(tmp_path / "remote")
    push(store, remote, [])
    assert sorted(remote.paths()) == LAYERS


def test_one_motion_moves_alone(store, tmp_path):
    remote = DirectoryRemote(tmp_path / "remote")
    push(store, remote, ["g1_only_motion", "lite_pro_tracking"])
    assert sorted(remote.paths()) == [
        "blueprints/lite_pro.rbl",
        "lite_pro/lite_pro_tracking.rrd",
        "reference/lite_pro_tracking.rrd",
    ]


def test_a_second_pull_is_a_no_op(store, tmp_path):
    remote = DirectoryRemote(tmp_path / "remote")
    push(store, remote, [])
    pulled = MotionStore(tmp_path / "pulled")
    assert pull(remote, pulled, []) == pull(remote, pulled, [])
    assert sorted(pulled.paths()) == LAYERS


def test_an_unknown_scheme_names_the_ones_that_work():
    with pytest.raises(ValueError, match=r"expected a scheme in \['hf', 's3'\]"):
        uri_to_remote("gs://a-bucket/motions")


def test_a_hugging_face_uri_splits_off_the_revision():
    pytest.importorskip("huggingface_hub")
    remote = uri_to_remote("hf://owner/motions@v2")
    assert (remote.repo_id, remote.revision) == ("owner/motions", "v2")


def test_the_hub_s_own_uri_form_is_accepted():
    """`hf://datasets/<owner>/<name>` is what the `hf` command line takes, so it works here."""
    pytest.importorskip("huggingface_hub")
    remote = uri_to_remote("hf://datasets/owner/motions@v2")
    assert (remote.repo_id, remote.revision) == ("owner/motions", "v2")


def test_a_hugging_face_uri_without_a_revision_reads_the_default_branch():
    pytest.importorskip("huggingface_hub")
    assert uri_to_remote("hf://owner/motions").revision is None


def test_an_s3_prefix_stands_in_for_the_store_root(monkeypatch):
    pytest.importorskip("boto3")
    monkeypatch.setenv("AWS_DEFAULT_REGION", "us-west-2")
    remote = uri_to_remote("s3://a-bucket/datasets/mikumotion/")
    assert remote.bucket == "a-bucket"
    assert remote.key("reference/zamuza.rrd") == "datasets/mikumotion/reference/zamuza.rrd"


def test_an_s3_bucket_needs_no_prefix(monkeypatch):
    pytest.importorskip("boto3")
    monkeypatch.setenv("AWS_DEFAULT_REGION", "us-west-2")
    remote = uri_to_remote("s3://a-bucket")
    assert remote.key("reference/zamuza.rrd") == "reference/zamuza.rrd"
