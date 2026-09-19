### Task 4: 增加一次性标定 CLI 模式并保留桌面标定入口

**Files:**
- Modify: `deploy/mocap_bridge/chingmu_table_lcm_bridge.py:178-236,577-690`
- Modify: `deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py`

**Interfaces:**
- Consumes:
  - `_collect_calibration_frames(client, duration_s, argument_name=...)`
  - `calibrate_pelvis_orientation_from_frames(...)`
  - `save_pelvis_orientation_calibration(...)`
  - `_load_runtime_pelvis_orientation(...)`
- Produces:
  - CLI `--save-pelvis-orientation-calib PATH`
  - CLI `--pelvis-calib-sec FLOAT`
  - `_operation_mode(args) -> str`
  - operation modes `table_calibration`, `pelvis_calibration`, `runtime`

- [ ] **Step 1: 写 CLI 模式解析失败测试**

Extend imports:

```python
    _operation_mode,
```

Append:

```python
class PelvisOrientationCliTest(unittest.TestCase):
    def setUp(self):
        self.parser = build_arg_parser()

    def test_selects_pelvis_calibration_mode(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--save-pelvis-orientation-calib",
                "pelvis.json",
            ]
        )
        self.assertEqual(_operation_mode(args), "pelvis_calibration")

    def test_selects_runtime_mode(self):
        args = self.parser.parse_args(
            ["--pelvis-orientation-calib", "pelvis.json"]
        )
        self.assertEqual(_operation_mode(args), "runtime")

    def test_preserves_table_only_calibration_mode(self):
        args = self.parser.parse_args(
            ["--save-table-calib", "table.json"]
        )
        self.assertEqual(_operation_mode(args), "table_calibration")

    def test_pelvis_calibration_requires_existing_table_file_argument(self):
        args = self.parser.parse_args(
            [
                "--save-pelvis-orientation-calib",
                "pelvis.json",
            ]
        )
        with self.assertRaisesRegex(
            ValueError,
            "--table-calib is required",
        ):
            _operation_mode(args)

    def test_pelvis_calibration_forbids_publish(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--save-pelvis-orientation-calib",
                "pelvis.json",
                "--publish",
            ]
        )
        with self.assertRaisesRegex(
            ValueError,
            "cannot be combined with --publish",
        ):
            _operation_mode(args)

    def test_load_and_save_orientation_are_mutually_exclusive(self):
        args = self.parser.parse_args(
            [
                "--table-calib",
                "table.json",
                "--pelvis-orientation-calib",
                "old.json",
                "--save-pelvis-orientation-calib",
                "new.json",
            ]
        )
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            _operation_mode(args)

    def test_missing_orientation_file_is_rejected_outside_table_mode(self):
        args = self.parser.parse_args([])
        with self.assertRaisesRegex(
            ValueError,
            "--pelvis-orientation-calib is required",
        ):
            _operation_mode(args)
```

- [ ] **Step 2: 运行 CLI 测试并确认 RED**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation.PelvisOrientationCliTest \
  -v
```

Expected: import error，指出 `_operation_mode` 或新 CLI 参数不存在。

- [ ] **Step 3: 增加 CLI 参数和模式解析**

Add to `build_arg_parser()`:

```python
    parser.add_argument(
        "--save-pelvis-orientation-calib",
        type=Path,
        help=(
            "Collect one aligned-pose calibration, atomically save "
            "the pelvis orientation JSON, and exit."
        ),
    )
    parser.add_argument(
        "--pelvis-calib-sec",
        type=float,
        default=2.0,
        help="Pelvis orientation aligned-pose collection duration.",
    )
```

Add:

```python
def _operation_mode(args) -> str:
    saving_pelvis = args.save_pelvis_orientation_calib is not None
    loading_pelvis = args.pelvis_orientation_calib is not None
    if saving_pelvis and loading_pelvis:
        raise ValueError(
            "--pelvis-orientation-calib and "
            "--save-pelvis-orientation-calib are mutually exclusive"
        )
    if saving_pelvis:
        if args.table_calib is None:
            raise ValueError(
                "--table-calib is required when saving pelvis "
                "orientation calibration"
            )
        if args.publish:
            raise ValueError(
                "--save-pelvis-orientation-calib cannot be combined "
                "with --publish"
            )
        return "pelvis_calibration"
    if loading_pelvis:
        return "runtime"
    if (
        args.save_table_calib is not None
        and args.table_calib is None
        and not args.publish
    ):
        return "table_calibration"
    raise ValueError(
        "--pelvis-orientation-calib is required outside "
        "table-only calibration mode"
    )
```

- [ ] **Step 4: 让帧采集错误信息区分两个 duration 参数**

Change:

```python
def _collect_calibration_frames(
    client,
    duration_s: float,
    *,
    argument_name: str = "--calib-sec",
) -> list:
    if duration_s <= 0.0:
        raise ValueError(f"{argument_name} must be positive")
    frames = []
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        frame = client.next_frame(timeout_s=0.1)
        if frame is not None:
            frames.append(frame)
    return frames
```

- [ ] **Step 5: 增加标定结果打印函数**

Add:

```python
def _describe_pelvis_orientation(
    calibration: PelvisOrientationCalibration,
) -> None:
    quaternion = quaternion_xyzw_from_rotation(
        calibration.rotation_rigid_from_pelvis
    )
    print(
        "Pelvis orientation calibration: "
        f"samples={calibration.sample_count} "
        f"source_duration_s={calibration.source_duration_s:.3f} "
        f"position_rms_m={calibration.position_rms_m:.6f} "
        f"angular_rms_deg={calibration.angular_rms_deg:.4f} "
        "quaternion_rigid_from_pelvis_xyzw="
        f"{np.array2string(quaternion, precision=7)}",
        flush=True,
    )
```

- [ ] **Step 6: 按模式重排 `main()`**

Immediately after parsing and config validation:

```python
    operation_mode = _operation_mode(args)
```

Remove the unconditional Task 3 call to `_load_runtime_pelvis_orientation` before SDK construction。

After the existing table load/calibration and `_describe_table(...)`:

```python
        if operation_mode == "table_calibration":
            print(
                "Saved table calibration; exiting before pelvis "
                "orientation is required.",
                flush=True,
            )
            return 0

        if operation_mode == "pelvis_calibration":
            print(
                "Pelvis aligned-pose calibration: confirm pelvis "
                "+X/+Y/+Z are parallel to table-world +X/+Y/+Z. "
                f"Collecting for {args.pelvis_calib_sec:.2f} s ...",
                flush=True,
            )
            frames = _collect_calibration_frames(
                client,
                args.pelvis_calib_sec,
                argument_name="--pelvis-calib-sec",
            )
            pelvis_orientation_calibration = (
                calibrate_pelvis_orientation_from_frames(
                    frames,
                    table,
                    config,
                )
            )
            _describe_pelvis_orientation(
                pelvis_orientation_calibration
            )
            save_pelvis_orientation_calibration(
                args.save_pelvis_orientation_calib,
                pelvis_orientation_calibration,
                config,
            )
            print(
                "Saved pelvis orientation calibration to "
                f"{args.save_pelvis_orientation_calib}",
                flush=True,
            )
            return 0

        pelvis_orientation_calibration = (
            _load_runtime_pelvis_orientation(args, config)
        )
        print(
            "Loaded pelvis orientation calibration from "
            f"{args.pelvis_orientation_calib}",
            flush=True,
        )
        _describe_pelvis_orientation(
            pelvis_orientation_calibration
        )
```

Keep LCM publisher construction and `ChingMuTableLcmBridge` construction after this block, so invalid/missing orientation files fail before publishing。

- [ ] **Step 7: 运行全部聚焦测试并确认 GREEN**

Run:

```bash
conda run --no-capture-output -n rb python -m unittest \
  deploy.mocap_bridge.tests.test_chingmu_pelvis_orientation \
  -v
```

Expected: 22 tests pass。

- [ ] **Step 8: 验证 CLI help**

Run:

```bash
conda run --no-capture-output -n rb python \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py --help
```

Expected: output includes:

```text
--pelvis-orientation-calib
--save-pelvis-orientation-calib
--pelvis-calib-sec
```

- [ ] **Step 9: 检查并提交 Task 4**

Run:

```bash
git diff --check -- \
  deploy/mocap_bridge/chingmu_table_lcm_bridge.py \
  deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git add -p -- deploy/mocap_bridge/chingmu_table_lcm_bridge.py
git add -p -- deploy/mocap_bridge/tests/test_chingmu_pelvis_orientation.py
git diff --cached --name-status
git diff --cached --check
git commit -m "feat: add ChingMu pelvis orientation calibration mode"
```

Expected: only the bridge and focused test file are staged。

---

