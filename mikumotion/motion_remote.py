"""
The motion store on a remote host, so a training run and a deployment read one dataset.

A motion is written once, on a workstation that has Blender and the robot assets, and then
read somewhere else: a training cluster, a deployment target, another developer's machine. The
store is a directory of files, so moving it needs nothing more than a file transfer.

The remote mirrors the store's layout exactly. One store-relative posix path, such as
``reference/zamuza.rrd``, addresses the same file on both sides: ``root / path`` on disk, and
``path`` in the repository or the bucket. The layout already names every file, so nothing keeps
a manifest of what is where.

That mirror is the whole contract. A backend is a class with three methods, and it inherits
nothing:

    paths()             every file the remote holds, as store-relative posix paths
    pull(paths, root)   copy those paths down, so each one lands at ``root / path``
    push(paths, root)   copy every ``root / path`` up, to the same path

No method knows what a motion is. :func:`motion_paths` turns motion names into files, once, for
every backend. A URI names the remote, and :func:`uri_to_remote` reads its scheme:

    hf://<owner>/<name>[@<revision>]            a Hugging Face dataset repository
    hf://datasets/<owner>/<name>[@<revision>]   the same, in the Hub's own URI form
    s3://<bucket>/<prefix>                      an S3 bucket

``MIKUMOTION_REMOTE`` holds the default, so an environment is configured once instead of on
every command. A revision pins a read to one version of the dataset, which is what makes a
training run reproducible.

Transfers are idempotent, and a backend may skip a file whose content the other side already
holds. The Hugging Face backend does, in both directions. An S3 transfer re-sends the bytes.

Credentials never reach this module. Each backend uses its own library's credential chain, so
a token stays where that library already keeps it.
"""

import os
from pathlib import Path

from .motion_sequence import BLUEPRINTS

REMOTE = os.environ.get("MIKUMOTION_REMOTE", "hf://T-K-233/mikumotion-motions")


def motion_paths(paths, names):
    """
    Select the store files that make up ``names``, out of a listing from either side.

    Both sides list the same relative paths, so one rule serves a push and a pull, and the two
    directions can never disagree about which files a motion is. An empty ``names`` selects the
    whole store.

    A motion's own files are the ones whose stem is its name, in whichever layer directory. A
    robot layer also needs that robot's blueprint, so the motion arrives with the viewer layout
    it was written for.
    """
    # .rrd and .rbl are the store. Anything else in a listing is not motion data: a dataset
    # card, a .gitattributes, the download metadata that a pull leaves behind.
    layers = [path for path in paths if path.endswith((".rrd", ".rbl"))]
    if not names:
        return sorted(layers)
    motions = [path for path in layers if Path(path).stem in names]
    robots = {Path(path).parent.name for path in motions}
    return sorted(motions + [path for path in layers
                             if path.startswith(BLUEPRINTS + "/") and Path(path).stem in robots])


def motion_layers(paths):
    """
    Group a listing by motion name, so a remote reads as motions rather than as files.

    Blueprints are left out. A blueprint belongs to a robot and not to a motion, so listing
    one would print a robot's name where a motion name belongs.
    """
    layers = {}
    for path in motion_paths(paths, []):
        if path.endswith(".rrd"):
            layers.setdefault(Path(path).stem, []).append(Path(path).parent.name)
    return {name: sorted(found) for name, found in sorted(layers.items())}


class HuggingFaceRemote:
    """
    A Hugging Face dataset repository, ``hf://<owner>/<name>[@<revision>]``.

    The Hub's own URI form, ``hf://datasets/<owner>/<name>[@<revision>]``, is accepted as well,
    so one string works both here and with the ``hf`` command line.

    The repository holds the store's directories as its own, so the dataset is browsable on the
    Hub and a reader can fetch one motion out of it without cloning the rest.

    Each transfer is one call to the library's bulk helper rather than a loop over single files,
    which is the whole reason to reach for them. ``upload_folder`` checks, uploads and commits in
    parallel, resumes an interrupted push when it is run again, and drops a file whose content
    the Hub already holds. ``snapshot_download`` fetches concurrently and skips a file whose
    recorded commit hash still matches, and it keeps that record in ``<root>/.cache``, which both
    helpers then ignore. Both take the selected files as ``allow_patterns``, which they match
    with ``fnmatch``, so a motion whose name contains a glob character would select more than
    itself.

    Large files go to Xet storage, which splits content into chunks and stores each chunk once.
    Every robot layer carries its own copy of the robot's geometry, around 9 MB of identical
    bytes per motion, so the Hub keeps one copy however many motions the dataset holds. Set
    ``HF_XET_HIGH_PERFORMANCE=1`` to let a transfer use the whole machine.

    The repository is created private, because the license of an MMD motion does not permit
    redistribution. Make a dataset public on the Hub, deliberately, once its motions allow it.

    Authentication is ``huggingface_hub``'s own: the ``HF_TOKEN`` environment variable, or the
    token that ``hf auth login`` stored.
    """

    def __init__(self, location):
        from huggingface_hub import HfApi

        repo_id, _, revision = location.partition("@")
        self.repo_id = repo_id.removeprefix("datasets/")
        self.revision = revision or None
        self.api = HfApi(library_name="mikumotion")

    def paths(self):
        return self.api.list_repo_files(self.repo_id, repo_type="dataset",
                                        revision=self.revision)

    def pull(self, paths, root):
        from huggingface_hub import snapshot_download

        snapshot_download(self.repo_id, repo_type="dataset", revision=self.revision,
                          local_dir=root, allow_patterns=paths)

    def push(self, paths, root):
        self.api.create_repo(self.repo_id, repo_type="dataset", private=True, exist_ok=True)
        self.api.upload_folder(repo_id=self.repo_id, folder_path=root, repo_type="dataset",
                               revision=self.revision, allow_patterns=paths,
                               commit_message=f"push {len(paths)} motion file"
                                              + "s" * (len(paths) != 1))


class S3Remote:
    """
    An S3 bucket, ``s3://<bucket>/<prefix>``.

    The prefix stands in for the store root, so a key is that prefix followed by the same
    relative path the other backend uses. A bucket has no revisions, so a URI for one carries
    no ``@revision``. Pin a version by giving each dataset release its own prefix.

    ``download_file`` and ``upload_file`` are boto3's managed transfers, so a file large enough
    is split into concurrent multipart requests without anything here asking for it. They have
    no unchanged-file check, though, so an S3 transfer moves every file it is given: an ETag is
    an MD5 only for a single-part upload, which makes it too unreliable to treat as a content
    identity, and a size alone is too weak.

    Credentials are ``boto3``'s own chain: the environment, the shared credentials file, or the
    instance role that a training node already runs under.
    """

    def __init__(self, location):
        import boto3

        self.bucket, _, prefix = location.partition("/")
        self.prefix = prefix.strip("/")
        self.client = boto3.client("s3")

    def key(self, path):
        """The object key for a store path. The prefix stands in for the store root."""
        return f"{self.prefix}/{path}".lstrip("/")

    def paths(self):
        pages = self.client.get_paginator("list_objects_v2").paginate(Bucket=self.bucket,
                                                                      Prefix=self.prefix)
        return [obj["Key"].removeprefix(self.prefix).lstrip("/")
                for page in pages for obj in page.get("Contents", [])]

    def pull(self, paths, root):
        for path in paths:
            destination = Path(root) / path
            destination.parent.mkdir(parents=True, exist_ok=True)
            self.client.download_file(self.bucket, self.key(path), str(destination))

    def push(self, paths, root):
        for path in paths:
            self.client.upload_file(str(Path(root) / path), self.bucket, self.key(path))


def uri_to_remote(uri=REMOTE):
    """Build the backend that the URI's scheme names."""
    backends = {"hf": HuggingFaceRemote, "s3": S3Remote}
    scheme, _, location = uri.partition("://")
    if scheme not in backends:
        raise ValueError(f"{uri}: expected a scheme in {sorted(backends)}")
    return backends[scheme](location)
