import copy
import datetime
import errno
import hashlib
import json
import os
import stat
import tempfile
import threading
import unittest
from pathlib import Path
from types import MappingProxyType
from unittest import mock

import yaml

from tools.hitter_ready_pose_editor.asset_model import AssetManifest
from tools.hitter_ready_pose_editor.constants import (
    ARM_JOINT_NAMES,
    MOTION_NPZ_ARM_INDICES,
    ROBOT29_ARM_INDICES,
)
from tools.hitter_ready_pose_editor.scripts import (
    generate_live_fk_contract as contract_generator,
)
from tools.hitter_ready_pose_editor.pose_serialization import (
    PoseStore,
    PoseStoreError,
    build_pose_record,
)
from tools.hitter_ready_pose_editor.pose_validation import ValidationResult


CHINA_TZ = datetime.timezone(datetime.timedelta(hours=8))
SHA_A = "a" * 64
SHA_B = "b" * 64
SHA_C = "c" * 64
SHA_D = "d" * 64
SHA_E = "e" * 64
SHA_F = "f" * 64
BASE_JOINT_NAMES = tuple("base_joint_%02d" % index for index in range(15))
ACTIVE_JOINT_NAMES = BASE_JOINT_NAMES + ARM_JOINT_NAMES
SAVE_CRITICAL_LINKS = (
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "right_racket_link",
)


class InjectedStoreFailure(RuntimeError):
    """Fault injected at a selected persistence phase."""


class RaiseAt:
    def __init__(self, phase):
        self.phase = phase

    def __call__(self, phase):
        if phase == self.phase:
            raise InjectedStoreFailure(phase)


class GuardedContractOutputTest(unittest.TestCase):
    def setUp(self):
        self.temp_root = Path(tempfile.gettempdir()).resolve(strict=True)
        self.repo_root = Path(__file__).resolve().parents[3]
        self.payload = {"schema": "test/v1", "poses": [1, 2, 3]}
        self.paths = []

    def tearDown(self):
        for path in reversed(self.paths):
            try:
                if path.is_dir() and not path.is_symlink():
                    path.rmdir()
                else:
                    path.unlink()
            except FileNotFoundError:
                continue

    def make_file(self, prefix="hitter-ready-fk."):
        descriptor, raw_path = tempfile.mkstemp(
            prefix=prefix,
            dir=str(self.temp_root),
        )
        os.close(descriptor)
        path = Path(raw_path)
        self.paths.append(path)
        return path

    def canonical_payload(self):
        return (
            json.dumps(
                self.payload,
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=False,
            )
            + "\n"
        ).encode("utf-8")

    def assert_rejected_without_mutation(self, path, expected_bytes):
        with self.assertRaises((OSError, ValueError)):
            contract_generator.write_contract_to_precreated_temp(
                path,
                self.payload,
            )
        self.assertEqual(path.read_bytes(), expected_bytes)

    def test_atomic_publication_never_writes_caller_inode(self):
        path = self.make_file()
        caller_descriptor = os.open(str(path), os.O_RDONLY)
        caller_before = os.fstat(caller_descriptor)
        try:
            result = contract_generator.write_contract_to_precreated_temp(
                path,
                self.payload,
            )
            caller_after = os.fstat(caller_descriptor)
        finally:
            os.close(caller_descriptor)

        published = path.stat()
        self.assertEqual(result, path)
        self.assertNotEqual(
            (published.st_dev, published.st_ino),
            (caller_before.st_dev, caller_before.st_ino),
        )
        self.assertEqual(caller_after.st_nlink, 0)
        self.assertEqual(caller_after.st_size, 0)
        self.assertEqual(stat.S_IMODE(published.st_mode), 0o600)
        self.assertEqual(path.read_bytes(), self.canonical_payload())

    def test_rejects_repo_and_pose_directory_paths_without_creating_them(self):
        paths = (
            self.repo_root / "hitter-ready-fk.repo-output",
            self.repo_root
            / "deploy/data/hitter_ready_poses/hitter-ready-fk.pose-output",
        )
        for path in paths:
            self.assertFalse(path.exists())
            with self.assertRaises((OSError, ValueError)):
                contract_generator.write_contract_to_precreated_temp(
                    path,
                    self.payload,
                )
            self.assertFalse(path.exists())

    def test_rejects_arbitrary_tree_below_temp_root_without_mutation(self):
        external_tree = Path(
            tempfile.mkdtemp(
                prefix="hitter-contract-external-",
                dir=self.temp_root,
            )
        )
        self.paths.append(external_tree)
        path = external_tree / "hitter-ready-fk.nested"
        path.write_bytes(b"")
        self.paths.append(path)
        self.assert_rejected_without_mutation(path, b"")

    def test_rejects_wrong_basename_without_mutation(self):
        path = self.make_file(prefix="not-hitter-ready-fk.")
        self.assert_rejected_without_mutation(path, b"")

    def test_rejects_symlink_without_following_or_mutating_target(self):
        target = self.make_file(prefix="guard-target.")
        target.write_bytes(b"target sentinel")
        link = self.temp_root / ("hitter-ready-fk.link-" + target.name)
        link.symlink_to(target)
        self.paths.append(link)

        with self.assertRaises((OSError, ValueError)):
            contract_generator.write_contract_to_precreated_temp(
                link,
                self.payload,
            )
        self.assertTrue(link.is_symlink())
        self.assertEqual(target.read_bytes(), b"target sentinel")

    def test_rejects_nonempty_file_without_truncation(self):
        path = self.make_file()
        path.write_bytes(b"do not truncate")
        before = path.stat()
        self.assert_rejected_without_mutation(path, b"do not truncate")
        after = path.stat()
        self.assertEqual(
            (after.st_dev, after.st_ino),
            (before.st_dev, before.st_ino),
        )

    def test_rejects_multiply_linked_file_without_mutation(self):
        path = self.make_file()
        link = self.temp_root / ("hitter-ready-fk.hard-" + path.name)
        os.link(str(path), str(link))
        self.paths.append(link)
        self.assertEqual(path.stat().st_nlink, 2)
        self.assert_rejected_without_mutation(path, b"")
        self.assertEqual(path.stat().st_nlink, 2)
        self.assertEqual(link.read_bytes(), b"")

    def test_rejects_non_regular_type_without_mutation(self):
        path = self.make_file()
        path.unlink()
        os.mkfifo(str(path), 0o600)
        reader = os.open(str(path), os.O_RDONLY | os.O_NONBLOCK)
        try:
            with self.assertRaises((OSError, ValueError)):
                contract_generator.write_contract_to_precreated_temp(
                    path,
                    self.payload,
                )
            self.assertEqual(os.read(reader, 1), b"")
        finally:
            os.close(reader)
        self.assertTrue(path.exists())

    @unittest.skipUnless(
        os.geteuid() == 0,
        "changing file ownership safely requires root",
    )
    def test_rejects_file_owned_by_another_uid_without_mutation(self):
        path = self.make_file()
        os.chown(str(path), 65534, -1)
        self.assert_rejected_without_mutation(path, b"")

    def test_content_race_is_rejected_without_generator_write(self):
        path = self.make_file()
        observer = os.open(str(path), os.O_RDONLY)
        real_unlink = contract_generator.os.unlink
        raced = []

        def unlink_after_racer_write(name, *, dir_fd=None):
            if name == path.name and not raced:
                writer = os.open(name, os.O_WRONLY, dir_fd=dir_fd)
                try:
                    os.write(writer, b"racer content")
                finally:
                    os.close(writer)
                raced.append(True)
            return real_unlink(name, dir_fd=dir_fd)

        try:
            with mock.patch.object(
                contract_generator.os,
                "unlink",
                side_effect=unlink_after_racer_write,
            ):
                with self.assertRaisesRegex(ValueError, "empty"):
                    contract_generator.write_contract_to_precreated_temp(
                        path,
                        self.payload,
                    )
            os.lseek(observer, 0, os.SEEK_SET)
            self.assertEqual(os.read(observer, 64), b"racer content")
        finally:
            os.close(observer)
        self.assertEqual(raced, [True])
        self.assertFalse(path.exists())

    def test_hardlink_race_is_rejected_without_writing_shared_inode(self):
        path = self.make_file()
        hardlink = self.temp_root / (
            "guard-race-hardlink-" + path.name
        )
        self.paths.append(hardlink)
        real_unlink = contract_generator.os.unlink
        raced = []

        def unlink_after_racer_link(name, *, dir_fd=None):
            if name == path.name and not raced:
                os.link(
                    name,
                    hardlink.name,
                    src_dir_fd=dir_fd,
                    dst_dir_fd=dir_fd,
                    follow_symlinks=False,
                )
                raced.append(True)
            return real_unlink(name, dir_fd=dir_fd)

        with mock.patch.object(
            contract_generator.os,
            "unlink",
            side_effect=unlink_after_racer_link,
        ):
            with self.assertRaisesRegex(ValueError, "detached"):
                contract_generator.write_contract_to_precreated_temp(
                    path,
                    self.payload,
                )
        self.assertEqual(raced, [True])
        self.assertFalse(path.exists())
        self.assertEqual(hardlink.read_bytes(), b"")

    def test_payload_writes_have_no_visible_output_name(self):
        path = self.make_file()
        real_write = contract_generator.os.write
        output_visibility_during_write = []

        def observe_write(descriptor, payload):
            output_visibility_during_write.append(path.exists())
            return real_write(descriptor, payload)

        with mock.patch.object(
            contract_generator.os,
            "write",
            side_effect=observe_write,
        ):
            contract_generator.write_contract_to_precreated_temp(
                path,
                self.payload,
            )
        self.assertTrue(output_visibility_during_write)
        self.assertEqual(
            output_visibility_during_write,
            [False] * len(output_visibility_during_write),
        )
        self.assertEqual(path.read_bytes(), self.canonical_payload())

    def test_direct_link_publishes_completed_anonymous_inode(self):
        path = self.make_file()
        real_write = contract_generator.os.write
        real_link = contract_generator._link_anonymous_temp
        link_finished = []
        writes_after_link = []
        completed_identities = []

        def observe_write(descriptor, payload):
            if link_finished:
                writes_after_link.append(bytes(payload))
            return real_write(descriptor, payload)

        def observe_direct_link(
            anonymous_descriptor,
            directory_descriptor,
            output_name,
        ):
            self.assertEqual(output_name, path.name)
            self.assertFalse(path.exists())
            completed = os.fstat(anonymous_descriptor)
            self.assertEqual(completed.st_nlink, 0)
            self.assertEqual(completed.st_size, len(self.canonical_payload()))
            self.assertEqual(stat.S_IMODE(completed.st_mode), 0o600)
            completed_identities.append(
                (completed.st_dev, completed.st_ino)
            )
            real_link(
                anonymous_descriptor,
                directory_descriptor,
                output_name,
            )
            published = os.stat(
                output_name,
                dir_fd=directory_descriptor,
                follow_symlinks=False,
            )
            self.assertEqual(
                (published.st_dev, published.st_ino),
                completed_identities[0],
            )
            self.assertEqual(path.read_bytes(), self.canonical_payload())
            link_finished.append(True)

        with mock.patch.object(
            contract_generator.os,
            "write",
            side_effect=observe_write,
        ), mock.patch.object(
            contract_generator,
            "_link_anonymous_temp",
            side_effect=observe_direct_link,
        ):
            contract_generator.write_contract_to_precreated_temp(
                path,
                self.payload,
            )

        self.assertEqual(link_finished, [True])
        self.assertEqual(len(completed_identities), 1)
        self.assertEqual(writes_after_link, [])
        self.assertEqual(path.read_bytes(), self.canonical_payload())
        self.assertEqual(
            (path.stat().st_dev, path.stat().st_ino),
            completed_identities[0],
        )
        self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

    def test_final_name_collision_preserves_race_winner_without_unlink(self):
        path = self.make_file()
        race_winner = self.make_file(prefix="guard-final-race-winner.")
        race_winner.write_bytes(b"unrelated race winner")
        winner_before = race_winner.stat()
        linked_descriptors = []
        unlinked_names = []
        real_link = contract_generator._link_anonymous_temp
        real_unlink = contract_generator.os.unlink

        def collide_before_direct_link(
            anonymous_descriptor,
            directory_descriptor,
            output_name,
        ):
            self.assertEqual(output_name, path.name)
            linked_descriptors.append(anonymous_descriptor)
            os.link(
                race_winner.name,
                path.name,
                src_dir_fd=directory_descriptor,
                dst_dir_fd=directory_descriptor,
                follow_symlinks=False,
            )
            return real_link(
                anonymous_descriptor,
                directory_descriptor,
                output_name,
            )

        def record_unlink(name, *, dir_fd=None):
            unlinked_names.append(name)
            return real_unlink(name, dir_fd=dir_fd)

        descriptor_count_before = len(os.listdir("/proc/self/fd"))
        with mock.patch.object(
            contract_generator,
            "_link_anonymous_temp",
            side_effect=collide_before_direct_link,
        ), mock.patch.object(
            contract_generator.os,
            "unlink",
            side_effect=record_unlink,
        ):
            with self.assertRaises(OSError) as caught:
                contract_generator.write_contract_to_precreated_temp(
                    path,
                    self.payload,
                )
        descriptor_count_after = len(os.listdir("/proc/self/fd"))

        self.assertEqual(caught.exception.errno, errno.EEXIST)
        self.assertEqual(descriptor_count_after, descriptor_count_before)
        self.assertEqual(len(linked_descriptors), 1)
        with self.assertRaises(OSError) as closed:
            os.fstat(linked_descriptors[0])
        self.assertEqual(closed.exception.errno, errno.EBADF)
        self.assertEqual(unlinked_names, [path.name])
        self.assertTrue(path.exists())
        winner_after = race_winner.stat()
        published_after = path.stat()
        self.assertEqual(
            (winner_after.st_dev, winner_after.st_ino),
            (winner_before.st_dev, winner_before.st_ino),
        )
        self.assertEqual(
            (published_after.st_dev, published_after.st_ino),
            (winner_before.st_dev, winner_before.st_ino),
        )
        self.assertEqual(path.read_bytes(), b"unrelated race winner")
        self.assertEqual(race_winner.read_bytes(), b"unrelated race winner")

    def test_old_stat_unlink_replacement_window_has_no_cleanup_unlink(self):
        path = self.make_file()
        replacement = self.make_file(prefix="guard-cleanup-replacement.")
        replacement.write_bytes(b"unrelated cleanup replacement")
        replacement_before = replacement.stat()
        real_stat = contract_generator.os.stat
        real_replace = contract_generator.os.replace
        real_unlink = contract_generator.os.unlink
        stage_stat_count = []
        unlinked_names = []

        def replace_after_old_cleanup_stat(
            name,
            *,
            dir_fd=None,
            follow_symlinks=True,
        ):
            result = real_stat(
                name,
                dir_fd=dir_fd,
                follow_symlinks=follow_symlinks,
            )
            if (
                isinstance(name, str)
                and name.startswith(".{}.stage-".format(path.name))
            ):
                stage_stat_count.append(name)
                if len(stage_stat_count) == 2:
                    real_replace(
                        replacement.name,
                        name,
                        src_dir_fd=dir_fd,
                        dst_dir_fd=dir_fd,
                    )
            return result

        def reject_old_stage_publish(*args, **kwargs):
            raise OSError("injected old stage publish failure")

        def record_unlink(name, *, dir_fd=None):
            unlinked_names.append(name)
            return real_unlink(name, dir_fd=dir_fd)

        descriptor_count_before = len(os.listdir("/proc/self/fd"))
        with mock.patch.object(
            contract_generator.os,
            "stat",
            side_effect=replace_after_old_cleanup_stat,
        ), mock.patch.object(
            contract_generator.os,
            "replace",
            side_effect=reject_old_stage_publish,
        ), mock.patch.object(
            contract_generator.os,
            "unlink",
            side_effect=record_unlink,
        ):
            try:
                result = (
                    contract_generator.write_contract_to_precreated_temp(
                        path,
                        self.payload,
                    )
                )
                error = None
            except OSError as exc:
                result = None
                error = exc
        descriptor_count_after = len(os.listdir("/proc/self/fd"))

        self.assertTrue(
            replacement.exists(),
            "Fix4 cleanup deleted the replacement installed after stat",
        )
        replacement_after = replacement.stat()
        self.assertEqual(
            (replacement_after.st_dev, replacement_after.st_ino),
            (replacement_before.st_dev, replacement_before.st_ino),
        )
        self.assertEqual(
            replacement.read_bytes(),
            b"unrelated cleanup replacement",
        )
        self.assertIsNone(error)
        self.assertEqual(result, path)
        self.assertEqual(stage_stat_count, [])
        self.assertEqual(unlinked_names, [path.name])
        self.assertEqual(descriptor_count_after, descriptor_count_before)
        self.assertEqual(path.read_bytes(), self.canonical_payload())

    def test_post_link_verification_error_leaves_complete_final_and_closes_fds(
        self,
    ):
        path = self.make_file()
        real_link = contract_generator._link_anonymous_temp
        real_stat = contract_generator.os.stat
        output_stat_count = []
        linked_descriptors = []
        completed_identities = []

        def capture_direct_link(
            anonymous_descriptor,
            directory_descriptor,
            output_name,
        ):
            linked_descriptors.append(anonymous_descriptor)
            completed = os.fstat(anonymous_descriptor)
            completed_identities.append(
                (completed.st_dev, completed.st_ino)
            )
            real_link(
                anonymous_descriptor,
                directory_descriptor,
                output_name,
            )

        def fail_post_link_verification(
            name,
            *,
            dir_fd=None,
            follow_symlinks=True,
        ):
            if name == path.name:
                output_stat_count.append(name)
                if len(output_stat_count) == 2:
                    raise OSError(
                        "injected post-link verification failure"
                    )
            return real_stat(
                name,
                dir_fd=dir_fd,
                follow_symlinks=follow_symlinks,
            )

        descriptor_count_before = len(os.listdir("/proc/self/fd"))
        with mock.patch.object(
            contract_generator,
            "_link_anonymous_temp",
            side_effect=capture_direct_link,
        ), mock.patch.object(
            contract_generator.os,
            "stat",
            side_effect=fail_post_link_verification,
        ):
            with self.assertRaisesRegex(
                OSError,
                "injected post-link verification failure",
            ):
                contract_generator.write_contract_to_precreated_temp(
                    path,
                    self.payload,
                )
        descriptor_count_after = len(os.listdir("/proc/self/fd"))

        self.assertEqual(descriptor_count_after, descriptor_count_before)
        self.assertEqual(len(linked_descriptors), 1)
        with self.assertRaises(OSError) as closed:
            os.fstat(linked_descriptors[0])
        self.assertEqual(closed.exception.errno, errno.EBADF)
        self.assertEqual(output_stat_count, [path.name, path.name])
        self.assertTrue(path.exists())
        self.assertEqual(
            (path.stat().st_dev, path.stat().st_ino),
            completed_identities[0],
        )
        self.assertEqual(path.read_bytes(), self.canonical_payload())
        self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

    @unittest.skipUnless(
        hasattr(os, "O_TMPFILE"),
        "anonymous temporary files require Linux O_TMPFILE",
    )
    def test_rejects_when_anonymous_tempfiles_are_unsupported(self):
        path = self.make_file()
        real_open = contract_generator.os.open

        def reject_anonymous_open(name, flags, mode=0o777, *, dir_fd=None):
            if flags & os.O_TMPFILE == os.O_TMPFILE:
                raise OSError(
                    errno.EOPNOTSUPP,
                    "injected unsupported anonymous tempfile",
                )
            return real_open(name, flags, mode, dir_fd=dir_fd)

        with mock.patch.object(
            contract_generator.os,
            "open",
            side_effect=reject_anonymous_open,
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "O_TMPFILE",
            ):
                contract_generator.write_contract_to_precreated_temp(
                    path,
                    self.payload,
                )
        self.assertFalse(path.exists())

    def test_anonymous_write_failure_leaves_no_output(self):
        path = self.make_file()
        with mock.patch.object(
            contract_generator.os,
            "write",
            side_effect=OSError("injected staging write failure"),
        ):
            with self.assertRaisesRegex(
                OSError,
                "injected staging write failure",
            ):
                contract_generator.write_contract_to_precreated_temp(
                    path,
                    self.payload,
                )
        self.assertFalse(path.exists())


def make_manifest():
    hashes = MappingProxyType(
        {
            "display_urdf_sha256": SHA_A,
            "display_mesh_set_sha256": SHA_B,
            "validation_mjcf_sha256": SHA_C,
            "asset_yaml_sha256": SHA_D,
            "urdf_kinematic_sha256": SHA_E,
            "mjcf_kinematic_sha256": SHA_F,
            "asset_signature_sha256": SHA_A,
        }
    )
    return AssetManifest(
        root_link="pelvis",
        active_joint_names=ACTIVE_JOINT_NAMES,
        arm_joint_names=ARM_JOINT_NAMES,
        joints_topological=(),
        joint_by_name=MappingProxyType({}),
        visuals=(),
        table_boxes=(),
        default_joint_pos_rad=tuple(0.0 for _ in range(29)),
        asset_paths=MappingProxyType(
            {
                "urdf": "/approved/display/main.urdf",
                "mjcf": "/approved/validation/model.xml",
                "asset_yaml": "/approved/config/asset.yaml",
                "allowed_asset_root": "/approved/display",
            }
        ),
        asset_hashes=hashes,
        urdf_kinematic_sha256=SHA_E,
        mjcf_kinematic_sha256=SHA_F,
        asset_signature_sha256=SHA_A,
        compatible_for_save=True,
        incompatibilities=(),
        right_racket_link="right_racket_link",
    )


def make_validation_result():
    joint_pos_29 = tuple(float(index) / 100.0 for index in range(29))
    arm_values = {
        name: joint_pos_29[index]
        for name, index in zip(ARM_JOINT_NAMES, ROBOT29_ARM_INDICES)
    }
    links = {
        name: {
            "position_m": [float(index), 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
        for index, name in enumerate(SAVE_CRITICAL_LINKS)
    }
    return ValidationResult(
        joint_pos_by_name=MappingProxyType(arm_values),
        joint_pos_29_rad=joint_pos_29,
        soft_limit_warnings=(),
        needs_soft_limit_confirmation=False,
        fk_robot_base=MappingProxyType(
            {
                "frame": "robot_base_default",
                "quaternion_convention": "xyzw",
                "links": MappingProxyType(links),
            }
        ),
        max_position_error_m=0.0001,
        max_orientation_error_rad=0.001,
        asset_signature_sha256=SHA_A,
    )


class PoseStoreTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.output_dir = Path(self.temp_dir.name) / "poses"
        self.asset_manifest = make_manifest()
        self.validation_result = make_validation_result()
        self.valid_record = build_pose_record(
            self.validation_result,
            self.asset_manifest,
        )
        self.valid_record["validation"]["soft_limits"] = "passed"
        self.record = copy.deepcopy(self.valid_record)
        self.fixed_time = datetime.datetime(
            2026, 7, 27, 10, 11, 12, tzinfo=CHINA_TZ
        )
        self.store = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
        )

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_saved_yaml_and_json_have_identical_semantics(self):
        record = build_pose_record(self.validation_result, self.asset_manifest)
        record["validation"]["soft_limits"] = "passed"
        saved = self.store.save(record, now=self.fixed_time)
        with saved.json_path.open("r", encoding="utf-8") as stream:
            json_data = json.load(stream)
        with saved.yaml_path.open("r", encoding="utf-8") as stream:
            yaml_data = yaml.safe_load(stream)
        self.assertEqual(json_data, yaml_data)
        self.assertEqual(json_data["schema"], "hitter_ready_arm_pose/v1")
        self.assertEqual(json_data["unit"], "rad")
        self.assertEqual(len(json_data["joint_names"]), 14)
        self.assertEqual(len(json_data["robot29"]["joint_pos_rad"]), 29)
        self.assertEqual(
            json_data["motion_npz"]["indices"],
            list(MOTION_NPZ_ARM_INDICES),
        )
        self.assertEqual(json_data["pose_name"], saved.pose_id)
        self.assertEqual(json_data["robot29"]["indices"], list(range(29)))
        self.assertEqual(json_data["fk"]["frame"], "robot_base_default")
        self.assertEqual(json_data["fk"]["root_link"], "pelvis")
        self.assertEqual(
            json_data["fk"]["quaternion_convention"], "xyzw"
        )
        self.assertEqual(json_data["validation"]["hard_limits"], "passed")
        self.assertEqual(
            json_data["validation"]["browser_mujoco_fk"], "passed"
        )
        self.assertEqual(saved.created_at, "2026-07-27T10:11:12+08:00")
        self.assertTrue(saved.json_path.is_absolute())
        self.assertTrue(saved.yaml_path.is_absolute())

    def test_same_second_creates_suffix_without_overwrite(self):
        first = self.store.save(self.record, now=self.fixed_time)
        second = self.store.save(self.record, now=self.fixed_time)
        self.assertNotEqual(first.json_path, second.json_path)
        self.assertEqual(
            second.pose_id, "hitter_ready_arm_pose_20260727_101112_001"
        )
        self.assertTrue(first.json_path.exists())
        self.assertTrue(second.json_path.exists())

    def test_two_stores_atomically_reserve_same_second_without_overwrite(self):
        barrier = threading.Barrier(2)

        def pause_after_json(phase):
            if phase == "after_json_rename":
                barrier.wait(timeout=5.0)

        stores = {
            label: PoseStore(
                self.output_dir,
                asset_manifest=self.asset_manifest,
                fault_hook=pause_after_json,
            )
            for label in ("first", "second")
        }
        records = {
            label: copy.deepcopy(self.valid_record)
            for label in stores
        }
        records["first"]["robot29"]["joint_pos_rad"][0] = 1.0
        records["second"]["robot29"]["joint_pos_rad"][0] = 2.0
        results = {}
        errors = []

        def save(label):
            try:
                results[label] = stores[label].save(
                    records[label],
                    now=self.fixed_time,
                )
            except BaseException as exc:
                errors.append(exc)

        threads = [
            threading.Thread(target=save, args=(label,))
            for label in stores
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=10.0)

        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertEqual(errors, [])
        self.assertEqual(
            sorted(saved.pose_id for saved in results.values()),
            [
                "hitter_ready_arm_pose_20260727_101112",
                "hitter_ready_arm_pose_20260727_101112_001",
            ],
        )
        self.assertEqual(len(self.store.list_records()), 2)
        self.assertEqual(
            {
                label: stores[label].load(saved.pose_id)["robot29"][
                    "joint_pos_rad"
                ][0]
                for label, saved in results.items()
            },
            {"first": 1.0, "second": 2.0},
        )

    def test_recovery_cannot_steal_reservation_before_owner_flock(self):
        reservation_created = threading.Event()
        release_owner = threading.Event()
        recovery_started = threading.Event()
        recovery_finished = threading.Event()
        saver_result = []
        saver_errors = []
        recovery_result = []
        recovery_errors = []

        def pause_before_reservation_flock(phase):
            if phase == "after_reservation_create_before_lock":
                reservation_created.set()
                if not release_owner.wait(timeout=5.0):
                    raise InjectedStoreFailure("reservation release timeout")

        class RecoveryAttemptStore(PoseStore):
            def recover_incomplete(self):
                recovery_started.set()
                return super().recover_incomplete()

        saver_store = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
            fault_hook=pause_before_reservation_flock,
        )

        def save_first():
            try:
                saver_result.append(
                    saver_store.save(
                        self.valid_record,
                        now=self.fixed_time,
                    )
                )
            except BaseException as exc:
                saver_errors.append(exc)

        def construct_recovery_store():
            try:
                recovery_result.append(
                    RecoveryAttemptStore(
                        self.output_dir,
                        asset_manifest=self.asset_manifest,
                    )
                )
            except BaseException as exc:
                recovery_errors.append(exc)
            finally:
                recovery_finished.set()

        saver_thread = threading.Thread(target=save_first)
        saver_thread.start()
        hook_reached = reservation_created.wait(timeout=2.0)
        if not hook_reached:
            release_owner.set()
            saver_thread.join(timeout=5.0)
            self.fail("reservation create-to-flock hook was not reached")

        recovery_thread = threading.Thread(target=construct_recovery_store)
        recovery_thread.start()
        self.assertTrue(recovery_started.wait(timeout=2.0))
        recovery_completed_before_owner = recovery_finished.wait(timeout=0.2)

        release_owner.set()
        saver_thread.join(timeout=10.0)
        recovery_thread.join(timeout=10.0)
        self.assertFalse(saver_thread.is_alive())
        self.assertFalse(recovery_thread.is_alive())
        self.assertEqual(saver_errors, [])
        self.assertEqual(recovery_errors, [])
        self.assertEqual(len(saver_result), 1)
        self.assertEqual(len(recovery_result), 1)

        second = recovery_result[0].save(
            self.valid_record,
            now=self.fixed_time,
        )
        quarantined_reservations = [
            path.name
            for path in (self.output_dir / ".incomplete").iterdir()
            if path.name.endswith(".reserve")
        ]
        self.assertFalse(recovery_completed_before_owner)
        self.assertEqual(quarantined_reservations, [])
        self.assertEqual(
            sorted((saver_result[0].pose_id, second.pose_id)),
            [
                "hitter_ready_arm_pose_20260727_101112",
                "hitter_ready_arm_pose_20260727_101112_001",
            ],
        )
        self.assertEqual(len(saver_store.list_records()), 2)

    def test_load_accepts_only_server_listed_pose_id(self):
        with self.assertRaises(PoseStoreError):
            self.store.load("../../config/mimic/hitter.yaml")

    def test_joint_name_is_authoritative(self):
        record = copy.deepcopy(self.valid_record)
        record["joint_pos_rad"][0], record["joint_pos_rad"][1] = (
            record["joint_pos_rad"][1],
            record["joint_pos_rad"][0],
        )
        with self.assertRaises(PoseStoreError):
            self.store.save(record, now=self.fixed_time)
        self.assertEqual(self.store.list_records(), [])

    def test_robot_and_motion_mappings_are_exact(self):
        saved = self.store.save(self.valid_record, now=self.fixed_time)
        record = self.store.load(saved.pose_id)
        authoritative = record["joint_pos_by_name"]
        self.assertEqual(
            record["joint_pos_rad"],
            [authoritative[name] for name in ARM_JOINT_NAMES],
        )
        self.assertEqual(
            [
                record["robot29"]["joint_pos_rad"][index]
                for index in ROBOT29_ARM_INDICES
            ],
            [authoritative[name] for name in ARM_JOINT_NAMES],
        )
        self.assertEqual(
            record["motion_npz"]["joint_pos_rad"],
            [authoritative[name] for name in ARM_JOINT_NAMES],
        )

    def test_output_dir_is_created_securely(self):
        nested = Path(self.temp_dir.name) / "absent" / "nested" / "poses"
        PoseStore(nested, asset_manifest=self.asset_manifest)
        self.assertTrue(nested.is_dir())
        self.assertEqual(stat.S_IMODE(nested.stat().st_mode), 0o700)
        self.assertEqual(
            stat.S_IMODE((nested / ".incomplete").stat().st_mode),
            0o700,
        )

        real_dir = Path(self.temp_dir.name) / "real_dir"
        real_dir.mkdir()
        dir_link = Path(self.temp_dir.name) / "dir_link"
        dir_link.symlink_to(real_dir, target_is_directory=True)
        with self.assertRaises(PoseStoreError):
            PoseStore(dir_link, asset_manifest=self.asset_manifest)

        real_file = Path(self.temp_dir.name) / "real_file"
        real_file.write_text("not a directory", encoding="utf-8")
        file_link = Path(self.temp_dir.name) / "file_link"
        file_link.symlink_to(real_file)
        with self.assertRaises(PoseStoreError):
            PoseStore(file_link, asset_manifest=self.asset_manifest)

    def test_preexisting_store_directories_require_exact_mode(self):
        wrong_output = Path(self.temp_dir.name) / "wrong_output"
        wrong_output.mkdir(mode=0o700)
        wrong_output.chmod(0o750)
        with self.assertRaises(PoseStoreError):
            PoseStore(
                wrong_output,
                asset_manifest=self.asset_manifest,
            )

        wrong_quarantine = Path(self.temp_dir.name) / "wrong_quarantine"
        wrong_quarantine.mkdir(mode=0o700)
        incomplete = wrong_quarantine / ".incomplete"
        incomplete.mkdir(mode=0o700)
        incomplete.chmod(0o755)
        with self.assertRaises(PoseStoreError):
            PoseStore(
                wrong_quarantine,
                asset_manifest=self.asset_manifest,
            )

    def test_failure_after_json_rename_is_not_visible(self):
        store = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
            fault_hook=RaiseAt("after_json_rename"),
        )
        with self.assertRaises(InjectedStoreFailure):
            store.save(self.valid_record, now=self.fixed_time)
        self.assertEqual(store.list_records(), [])
        restarted = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
        )
        self.assertEqual(restarted.list_records(), [])
        self.assertGreater(
            len(list((self.output_dir / ".incomplete").iterdir())),
            0,
        )

    def test_failure_after_yaml_rename_is_not_visible(self):
        store = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
            fault_hook=RaiseAt("after_yaml_rename"),
        )
        with self.assertRaises(InjectedStoreFailure):
            store.save(self.valid_record, now=self.fixed_time)
        restarted = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
        )
        self.assertEqual(restarted.list_records(), [])
        self.assertGreater(len(list((self.output_dir / ".incomplete").iterdir())), 0)

    def test_commit_marker_is_visibility_point(self):
        observations = {}
        store = None

        def observe(phase):
            if phase == "before_marker_rename":
                observations["before"] = store.list_records()
            elif phase == "after_marker_rename":
                observations["after"] = [
                    item["pose_id"] for item in store.list_records()
                ]

        store = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
            fault_hook=observe,
        )
        saved = store.save(self.valid_record, now=self.fixed_time)
        self.assertEqual(observations["before"], [])
        self.assertEqual(observations["after"], [saved.pose_id])

    def test_post_commit_fault_returns_committed_result(self):
        store = PoseStore(
            self.output_dir,
            asset_manifest=self.asset_manifest,
            fault_hook=RaiseAt("after_marker_rename"),
        )
        saved = store.save(self.valid_record, now=self.fixed_time)
        self.assertIsNotNone(saved.storage_warning)
        self.assertEqual(
            [item["pose_id"] for item in store.list_records()],
            [saved.pose_id],
        )
        self.assertEqual(store.load(saved.pose_id)["pose_name"], saved.pose_id)

    def test_marker_hash_mismatch_is_rejected(self):
        for operation in ("list", "load"):
            with self.subTest(operation=operation):
                output_dir = Path(self.temp_dir.name) / operation
                store = PoseStore(
                    output_dir,
                    asset_manifest=self.asset_manifest,
                )
                saved = store.save(self.valid_record, now=self.fixed_time)
                with saved.json_path.open("ab") as stream:
                    stream.write(b" ")
                with self.assertRaisesRegex(
                    PoseStoreError, "invalid saved pose generation"
                ):
                    if operation == "list":
                        store.list_records()
                    else:
                        store.load(saved.pose_id)
                self.assertFalse(saved.json_path.exists())
                self.assertFalse(saved.yaml_path.exists())
                self.assertGreater(
                    len(list((output_dir / ".incomplete").iterdir())),
                    0,
                )

    def test_committed_files_require_owner_regular_type_and_mode_0600(self):
        for basename in ("json", "yaml", "marker"):
            with self.subTest(basename=basename):
                output_dir = Path(self.temp_dir.name) / ("mode_" + basename)
                store = PoseStore(
                    output_dir,
                    asset_manifest=self.asset_manifest,
                )
                saved = store.save(self.valid_record, now=self.fixed_time)
                paths = {
                    "json": saved.json_path,
                    "yaml": saved.yaml_path,
                    "marker": output_dir
                    / (".%s.commit.json" % saved.pose_id),
                }
                paths[basename].chmod(0o640)
                with self.assertRaisesRegex(
                    PoseStoreError,
                    "invalid saved pose generation",
                ):
                    store.load(saved.pose_id)

    @unittest.skipUnless(os.geteuid() == 0, "file owner change needs root")
    def test_committed_file_with_wrong_owner_is_rejected(self):
        saved = self.store.save(self.valid_record, now=self.fixed_time)
        os.chown(str(saved.json_path), 65534, -1)
        with self.assertRaisesRegex(
            PoseStoreError,
            "invalid saved pose generation",
        ):
            self.store.load(saved.pose_id)

    def test_committed_symlink_is_not_followed(self):
        saved = self.store.save(self.valid_record, now=self.fixed_time)
        outside = Path(self.temp_dir.name) / "outside.json"
        os.replace(str(saved.json_path), str(outside))
        saved.json_path.symlink_to(outside)
        with self.assertRaisesRegex(
            PoseStoreError,
            "invalid saved pose generation",
        ):
            self.store.load(saved.pose_id)
        self.assertTrue(outside.is_file())

    def test_bool_indices_are_rejected(self):
        mutations = (
            ("robot_false", "robot29", 0, False),
            ("robot_true", "robot29", 1, True),
            ("motion", "motion_npz", 0, True),
        )
        for name, section, index, value in mutations:
            with self.subTest(name=name):
                output_dir = Path(self.temp_dir.name) / ("bool_" + name)
                store = PoseStore(
                    output_dir,
                    asset_manifest=self.asset_manifest,
                )
                record = copy.deepcopy(self.valid_record)
                record[section]["indices"][index] = value
                with self.assertRaises(PoseStoreError):
                    store.save(record, now=self.fixed_time)
                self.assertEqual(store.list_records(), [])

    def test_non_arm_robot_joint_name_must_match_startup_manifest(self):
        record = copy.deepcopy(self.valid_record)
        record["robot29"]["joint_names"][0] = "foreign_base_joint"
        with self.assertRaises(PoseStoreError):
            self.store.save(record, now=self.fixed_time)
        self.assertEqual(self.store.list_records(), [])

    def test_store_rejects_mutated_asset_provenance_before_publication(self):
        def change_path(record):
            record["asset"]["display_urdf_path"] = "/foreign/main.urdf"

        def change_hash(record):
            record["asset"]["validation_mjcf_sha256"] = "9" * 64

        def change_signature(record):
            record["asset"]["asset_signature_sha256"] = "8" * 64

        for mutation in (change_path, change_hash, change_signature):
            with self.subTest(mutation=mutation.__name__):
                output_dir = Path(self.temp_dir.name) / mutation.__name__
                store = PoseStore(
                    output_dir,
                    asset_manifest=self.asset_manifest,
                )
                record = copy.deepcopy(self.valid_record)
                mutation(record)
                with self.assertRaises(PoseStoreError):
                    store.save(record, now=self.fixed_time)
                self.assertEqual(store.list_records(), [])

    def test_reload_rejects_self_consistent_foreign_asset_provenance(self):
        saved = self.store.save(self.valid_record, now=self.fixed_time)
        marker_path = self.output_dir / (
            ".%s.commit.json" % saved.pose_id
        )
        with saved.json_path.open("r", encoding="utf-8") as stream:
            json_record = json.load(stream)
        with saved.yaml_path.open("r", encoding="utf-8") as stream:
            yaml_record = yaml.safe_load(stream)
        json_record["asset"]["display_urdf_path"] = "/foreign/main.urdf"
        yaml_record["asset"]["display_urdf_path"] = "/foreign/main.urdf"
        json_payload = (
            json.dumps(
                json_record,
                allow_nan=False,
                ensure_ascii=False,
                indent=2,
            )
            + "\n"
        ).encode("utf-8")
        yaml_payload = yaml.safe_dump(
            yaml_record,
            sort_keys=False,
            allow_unicode=True,
        ).encode("utf-8")
        saved.json_path.write_bytes(json_payload)
        saved.yaml_path.write_bytes(yaml_payload)
        marker = {
            "pose_id": saved.pose_id,
            "json_sha256": hashlib.sha256(json_payload).hexdigest(),
            "yaml_sha256": hashlib.sha256(yaml_payload).hexdigest(),
        }
        marker_path.write_text(
            json.dumps(
                marker,
                allow_nan=False,
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(
            PoseStoreError,
            "invalid saved pose generation",
        ):
            self.store.load(saved.pose_id)

    def test_list_is_newest_first(self):
        older = self.store.save(
            self.valid_record,
            now=self.fixed_time - datetime.timedelta(seconds=1),
        )
        newer = self.store.save(self.valid_record, now=self.fixed_time)
        self.assertEqual(
            [item["pose_id"] for item in self.store.list_records()],
            [newer.pose_id, older.pose_id],
        )

    def test_schema_and_values_are_strict(self):
        def wrong_schema(record):
            record["schema"] = "hitter_ready_arm_pose/v2"

        def duplicate_name(record):
            record["joint_names"][1] = record["joint_names"][0]

        def unknown_name(record):
            record["joint_names"][0] = "unknown_joint"

        def nan_name_value(record):
            record["joint_pos_by_name"][ARM_JOINT_NAMES[0]] = float("nan")

        def infinite_robot_value(record):
            record["robot29"]["joint_pos_rad"][0] = float("inf")

        for mutation in (
            wrong_schema,
            duplicate_name,
            unknown_name,
            nan_name_value,
            infinite_robot_value,
        ):
            with self.subTest(mutation=mutation.__name__):
                record = copy.deepcopy(self.valid_record)
                mutation(record)
                with self.assertRaises(PoseStoreError):
                    self.store.save(record, now=self.fixed_time)
                self.assertEqual(self.store.list_records(), [])


if __name__ == "__main__":
    unittest.main()
