#!/usr/bin/env python3
"""Generate a temporary browser/MuJoCo FK parity contract from live assets."""

import argparse
import ctypes
import errno
import json
import os
import stat
import tempfile
from pathlib import Path
from typing import Dict, List, Mapping

import numpy as np

from tools.hitter_ready_pose_editor.asset_model import HitterAssetModel
from tools.hitter_ready_pose_editor.constants import ARM_JOINT_NAMES
from tools.hitter_ready_pose_editor.pose_validation import PoseValidator
from utils.kinematics import ForwardKinematicsConfig, MujocoKinematics


RANDOM_SEED = 20260726
NEAR_SOFT_BOUNDARY_EPSILON_RAD = 1e-6
RANDOM_HARD_BOUNDARY_EPSILON_RAD = 1e-5
OUTPUT_BASENAME_PREFIX = "hitter-ready-fk."
AT_FDCWD = -100
AT_SYMLINK_FOLLOW = 0x400
AT_EMPTY_PATH = 0x1000


def _link_anonymous_temp(
    anonymous_descriptor: int,
    directory_descriptor: int,
    output_name: str,
) -> None:
    """Publish a completed anonymous inode under the requested output name."""
    libc = ctypes.CDLL(None, use_errno=True)
    try:
        linkat = libc.linkat
    except AttributeError as exc:
        raise RuntimeError(
            "secure output requires Linux linkat(AT_EMPTY_PATH)"
        ) from exc
    linkat.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
    )
    linkat.restype = ctypes.c_int
    result = linkat(
        anonymous_descriptor,
        b"",
        directory_descriptor,
        os.fsencode(output_name),
        AT_EMPTY_PATH,
    )
    error_number = ctypes.get_errno()
    if result != 0 and error_number in {errno.ENOENT, errno.EPERM}:
        proc_descriptor_path_text = "/proc/self/fd/{}".format(
            anonymous_descriptor
        )
        anonymous_stat = os.fstat(anonymous_descriptor)
        proc_stat = os.stat(proc_descriptor_path_text)
        if (
            proc_stat.st_dev != anonymous_stat.st_dev
            or proc_stat.st_ino != anonymous_stat.st_ino
        ):
            raise ValueError(
                "proc fd alias does not match anonymous output inode"
            )
        proc_descriptor_path = os.fsencode(proc_descriptor_path_text)
        result = linkat(
            AT_FDCWD,
            proc_descriptor_path,
            directory_descriptor,
            os.fsencode(output_name),
            AT_SYMLINK_FOLLOW,
        )
        error_number = ctypes.get_errno()
    if result != 0:
        raise OSError(
            error_number,
            os.strerror(error_number),
            output_name,
        )


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate the temporary HITTER live FK parity contract"
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def _joint_map(
    active_joint_names,
    values,
) -> Dict[str, float]:
    return {
        name: float(value)
        for name, value in zip(active_joint_names, values)
    }


def _sample_record(
    validator: PoseValidator,
    comparison_link_names,
    joint_pos_by_name: Mapping[str, float],
    **metadata,
) -> Dict[str, object]:
    joint_pos_29 = np.asarray(
        list(joint_pos_by_name.values()), dtype=np.float64
    )
    return {
        **metadata,
        "jointPosByName29": dict(joint_pos_by_name),
        "mujocoRobotBaseDefault": validator.forward_links_robot_base(
            joint_pos_29,
            comparison_link_names,
        ),
    }


def write_contract_to_precreated_temp(
    output_path: Path,
    contract: Mapping[str, object],
) -> Path:
    """Publish completed JSON after validating a pre-created temp file."""
    payload = (
        json.dumps(
            contract,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=False,
        )
        + "\n"
    ).encode("utf-8")
    temp_root = Path(tempfile.gettempdir()).resolve(strict=True)
    path = Path(os.path.abspath(str(Path(output_path).expanduser())))
    if path.parent != temp_root:
        raise ValueError("--output must be directly under the OS temp root")
    if (
        not path.name.startswith(OUTPUT_BASENAME_PREFIX)
        or len(path.name) == len(OUTPUT_BASENAME_PREFIX)
    ):
        raise ValueError(
            "--output basename must start with {}".format(
                OUTPUT_BASENAME_PREFIX
            )
        )
    if not hasattr(os, "O_NOFOLLOW"):
        raise RuntimeError("secure output requires O_NOFOLLOW")
    if not hasattr(os, "O_DIRECTORY"):
        raise RuntimeError("secure output requires O_DIRECTORY")
    if not hasattr(os, "O_TMPFILE"):
        raise RuntimeError("secure output requires Linux O_TMPFILE")

    directory_flags = (
        os.O_RDONLY
        | os.O_DIRECTORY
        | os.O_NOFOLLOW
        | getattr(os, "O_CLOEXEC", 0)
    )
    directory_descriptor = os.open(str(temp_root), directory_flags)
    caller_descriptor = None
    anonymous_descriptor = None
    try:
        caller_flags = (
            os.O_RDONLY
            | os.O_NOFOLLOW
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NONBLOCK", 0)
        )
        caller_descriptor = os.open(
            path.name,
            caller_flags,
            dir_fd=directory_descriptor,
        )
        opened = os.fstat(caller_descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise ValueError("--output must be a regular file")
        if opened.st_uid != os.geteuid():
            raise ValueError("--output must be owned by the current user")
        if opened.st_nlink != 1:
            raise ValueError("--output must have exactly one link")
        if opened.st_size != 0:
            raise ValueError("--output must be empty")
        named = os.stat(
            path.name,
            dir_fd=directory_descriptor,
            follow_symlinks=False,
        )
        if (
            not stat.S_ISREG(named.st_mode)
            or named.st_dev != opened.st_dev
            or named.st_ino != opened.st_ino
        ):
            raise ValueError("--output changed while it was opened")

        os.unlink(path.name, dir_fd=directory_descriptor)
        detached = os.fstat(caller_descriptor)
        if detached.st_size != 0:
            raise ValueError(
                "--output caller inode must remain empty after unlink"
            )
        if detached.st_nlink != 0:
            raise ValueError("--output caller inode was not detached")
        os.close(caller_descriptor)
        caller_descriptor = None

        anonymous_flags = (
            os.O_RDWR
            | os.O_TMPFILE
            | getattr(os, "O_CLOEXEC", 0)
        )
        try:
            anonymous_descriptor = os.open(
                ".",
                anonymous_flags,
                0o600,
                dir_fd=directory_descriptor,
            )
        except OSError as exc:
            unsupported = {
                errno.EINVAL,
                getattr(errno, "EOPNOTSUPP", errno.EINVAL),
                getattr(errno, "ENOTSUP", errno.EINVAL),
                errno.ENOSYS,
            }
            if exc.errno in unsupported:
                raise RuntimeError(
                    "secure output requires O_TMPFILE support "
                    "in the OS temp root"
                ) from exc
            raise
        os.fchmod(anonymous_descriptor, 0o600)
        anonymous = os.fstat(anonymous_descriptor)
        if (
            not stat.S_ISREG(anonymous.st_mode)
            or anonymous.st_uid != os.geteuid()
            or anonymous.st_nlink != 0
            or anonymous.st_size != 0
            or stat.S_IMODE(anonymous.st_mode) != 0o600
        ):
            raise ValueError("anonymous output metadata is unsafe")

        remaining = memoryview(payload)
        while remaining:
            written = os.write(anonymous_descriptor, remaining)
            if written <= 0:
                raise OSError("short write to anonymous output")
            remaining = remaining[written:]
        os.fsync(anonymous_descriptor)

        completed = os.fstat(anonymous_descriptor)
        if (
            not stat.S_ISREG(completed.st_mode)
            or completed.st_uid != os.geteuid()
            or completed.st_nlink != 0
            or completed.st_size != len(payload)
            or stat.S_IMODE(completed.st_mode) != 0o600
        ):
            raise ValueError("completed anonymous output metadata is unsafe")

        _link_anonymous_temp(
            anonymous_descriptor,
            directory_descriptor,
            path.name,
        )
        published = os.stat(
            path.name,
            dir_fd=directory_descriptor,
            follow_symlinks=False,
        )
        linked = os.fstat(anonymous_descriptor)
        if (
            not stat.S_ISREG(published.st_mode)
            or published.st_uid != os.geteuid()
            or published.st_dev != completed.st_dev
            or published.st_ino != completed.st_ino
            or published.st_size != len(payload)
            or published.st_nlink != 1
            or stat.S_IMODE(published.st_mode) != 0o600
            or linked.st_dev != completed.st_dev
            or linked.st_ino != completed.st_ino
            or linked.st_size != len(payload)
            or linked.st_nlink != 1
            or stat.S_IMODE(linked.st_mode) != 0o600
        ):
            raise ValueError("published output metadata is unsafe")

        os.lseek(anonymous_descriptor, 0, os.SEEK_SET)
        verified = bytearray()
        while len(verified) <= len(payload):
            chunk = os.read(
                anonymous_descriptor,
                len(payload) + 1 - len(verified),
            )
            if not chunk:
                break
            verified.extend(chunk)
        if bytes(verified) != payload:
            raise ValueError("published output bytes are incomplete")

        try:
            os.fsync(directory_descriptor)
        except OSError as exc:
            unsupported = {
                errno.EINVAL,
                errno.EROFS,
                getattr(errno, "ENOTSUP", errno.EINVAL),
            }
            if exc.errno not in unsupported:
                raise
    finally:
        try:
            if anonymous_descriptor is not None:
                os.close(anonymous_descriptor)
        finally:
            try:
                if caller_descriptor is not None:
                    os.close(caller_descriptor)
            finally:
                os.close(directory_descriptor)
    return path


def build_contract(repo_root: Path) -> Dict[str, object]:
    asset_model = HitterAssetModel.from_defaults(repo_root)
    manifest = asset_model.build_manifest()
    public_manifest = asset_model.public_manifest()
    if tuple(manifest.arm_joint_names) != ARM_JOINT_NAMES:
        raise RuntimeError(
            "live arm joint order differs from the editor contract"
        )

    kinematics = MujocoKinematics(
        ForwardKinematicsConfig(
            xml_path=str(manifest.asset_paths["mjcf"]),
            debug_viz=False,
            kinematic_joint_names=list(manifest.active_joint_names),
        )
    )
    validator = PoseValidator(
        asset_model,
        kinematics,
        np.asarray(asset_model._default_root_pos, dtype=np.float64),
    )
    comparison_link_names = (
        ("pelvis",)
        + tuple(
            manifest.joint_by_name[name].child_link
            for name in manifest.active_joint_names
        )
        + (manifest.right_racket_link,)
    )

    default_values = np.asarray(
        manifest.default_joint_pos_rad, dtype=np.float64
    )
    poses: List[Dict[str, object]] = [
        _sample_record(
            validator,
            comparison_link_names,
            _joint_map(manifest.active_joint_names, default_values),
            kind="default",
        )
    ]

    active_index = {
        name: index for index, name in enumerate(manifest.active_joint_names)
    }
    for name in manifest.arm_joint_names:
        spec = manifest.joint_by_name[name]
        if spec.soft_limit_rad is None:
            raise RuntimeError("{} has no soft limit".format(name))
        for boundary, value in (
            (
                "min",
                spec.soft_limit_rad[0]
                + NEAR_SOFT_BOUNDARY_EPSILON_RAD,
            ),
            (
                "max",
                spec.soft_limit_rad[1]
                - NEAR_SOFT_BOUNDARY_EPSILON_RAD,
            ),
        ):
            values = default_values.copy()
            values[active_index[name]] = value
            poses.append(
                _sample_record(
                    validator,
                    comparison_link_names,
                    _joint_map(manifest.active_joint_names, values),
                    kind="near_soft_boundary",
                    variedJointName=name,
                    boundary=boundary,
                )
            )

    random_generator = np.random.default_rng(RANDOM_SEED)
    random_lower = []
    random_upper = []
    for name in manifest.arm_joint_names:
        spec = manifest.joint_by_name[name]
        if spec.hard_limit_rad is None:
            raise RuntimeError("{} has no hard limit".format(name))
        random_lower.append(
            spec.hard_limit_rad[0] + RANDOM_HARD_BOUNDARY_EPSILON_RAD
        )
        random_upper.append(
            spec.hard_limit_rad[1] - RANDOM_HARD_BOUNDARY_EPSILON_RAD
        )
    random_arm_poses = random_generator.uniform(
        low=np.asarray(random_lower, dtype=np.float64),
        high=np.asarray(random_upper, dtype=np.float64),
        size=(100, len(manifest.arm_joint_names)),
    )
    for random_index, random_arm_values in enumerate(random_arm_poses):
        values = default_values.copy()
        for name, value in zip(
            manifest.arm_joint_names, random_arm_values
        ):
            values[active_index[name]] = value
        poses.append(
            _sample_record(
                validator,
                comparison_link_names,
                _joint_map(manifest.active_joint_names, values),
                kind="seeded_random",
                randomIndex=random_index,
            )
        )

    hashes = public_manifest["assetHashes"]
    return {
        "schema": "hitter_live_fk_contract/v1",
        "frame": "robot_base_default",
        "quaternionConvention": "xyzw",
        "angleUnit": "rad",
        "coordinateHandedness": "right",
        "matrixLayout": "column-major",
        "randomSeed": RANDOM_SEED,
        "nearSoftBoundaryEpsilonRad": NEAR_SOFT_BOUNDARY_EPSILON_RAD,
        "randomHardBoundaryEpsilonRad":
            RANDOM_HARD_BOUNDARY_EPSILON_RAD,
        "manifest": public_manifest,
        "asset": {
            "paths": {
                "urdf": manifest.asset_paths["urdf"],
                "mjcf": manifest.asset_paths["mjcf"],
                "assetYaml": manifest.asset_paths["asset_yaml"],
                "allowedAssetRoot":
                    manifest.asset_paths["allowed_asset_root"],
            },
            "hashes": dict(hashes),
            "combinedAssetSignatureSha256":
                manifest.asset_signature_sha256,
        },
        "comparisonLinkNames": list(comparison_link_names),
        "poses": poses,
    }


def main(argv=None) -> int:
    args = _parse_args(argv)
    repo_root = Path(__file__).resolve().parents[3]
    contract = build_contract(repo_root)
    output_path = write_contract_to_precreated_temp(args.output, contract)
    print(
        json.dumps(
            {
                "output": str(output_path),
                "pose_count": len(contract["poses"]),
                "comparison_link_count":
                    len(contract["comparisonLinkNames"]),
                "asset_signature_sha256":
                    contract["asset"]["combinedAssetSignatureSha256"],
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
