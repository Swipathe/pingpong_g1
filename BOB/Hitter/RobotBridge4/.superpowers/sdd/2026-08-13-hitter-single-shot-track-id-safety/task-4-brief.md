### Task 4: 建立 canonical runtime 身份类型并完成 `track_epoch -> track_id` 迁移

**Files:**
- Create: `deploy/utils/hitter_runtime_types.py`
- Modify: `deploy/utils/hitter_realtime.py`
- Modify: `deploy/simulator/real_world.py`
- Modify: `deploy/envs/hitter.py`
- Modify: `deploy/diagnostics/hitter_task_models.py`
- Modify: `deploy/diagnostics/hitter_task_attempts.py`
- Modify: `deploy/diagnostics/hitter_task_events.py`
- Modify: `deploy/diagnostics/hitter_task_pipeline.py`
- Modify: `deploy/diagnostics/hitter_task_monitor.py`
- Modify: `deploy/diagnostics/hitter_task_replay.py`
- Modify: `deploy/diagnostics/static/hitter_task_monitor.html`
- Modify: `deploy/tests/test_hitter_runtime_factory.py`
- Modify: `deploy/tests/test_hitter_strike_target_logging.py`
- Modify: `deploy/tests/test_hitter_task_diagnostics_integration.py`
- Modify: `deploy/tests/test_hitter_task_events.py`
- Modify: `deploy/tests/test_hitter_task_frontend.py`
- Modify: `deploy/tests/test_hitter_task_input_adapter.py`
- Modify: `deploy/tests/test_hitter_task_monitor.py`
- Modify: `deploy/tests/test_hitter_task_pipeline.py`
- Create: `deploy/tests/test_hitter_runtime_identity_types.py`
- Create: `deploy/tests/test_mujoco_hitter_track_id_v2.py`

**Interfaces:**
- Consumes: Task 1 wire name `track_id`。
- Produces: `SnapshotKey(track_id, generation)`、`BallEstimateSnapshot.track_id`、`BallEstimateSnapshot.consumed/new_track`、`PlannerResultSnapshot.track_id`；active runtime 不再从 diagnostics 导入身份 key。

`snapshot` fixture 在新测试文件中构造一条 `track_id=9,generation=1,source_frame=1` 的完整 `BallEstimateSnapshot`：position `[1.5,0,1]`、velocity `[-1,0,0]`、有限 base pose、`visible/ready=True`、`consumed=False`、`new_track=True`；它只用于验证字段合同，不依赖 RealWorld。

本任务只前置 `consumed: bool = False` 与 `new_track: bool = False` 的 immutable 字段合同，不实现任何 consume/admission gate。现有生产构造点保持默认 `False`；RealWorld 在 Task 7 才赋 wire-aware 真值，HitterEnv 在 Task 9 才依据这些字段做最终 phase gate。MuJoCo 不得把当前 generation/serve 逻辑提前解释成 `new_track`。

- [ ] **Step 1: 写 canonical type 失败测试**

```python
def test_snapshot_key_requires_positive_track_id():
    assert SnapshotKey(track_id=9, generation=0).to_json_dict() == {
        "schema_version": 2,
        "track_id": 9,
        "generation": 0,
    }
    with pytest.raises(ValueError, match="track_id must be positive"):
        SnapshotKey(track_id=0, generation=0)


def test_runtime_snapshots_expose_no_epoch_attribute(snapshot):
    assert snapshot.track_id == 9
    assert not hasattr(snapshot, "track_epoch")
```

- [ ] **Step 2: 运行测试并做全树旧名称盘点**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_identity_types.py
rg -n '\btrack_epoch\b' deploy --glob '*.py' --glob '*.html'
```

Expected: 测试 import/字段失败；`rg` 显示 runtime、diagnostics 和 fixture 的旧名字。

- [ ] **Step 3: 创建不依赖 diagnostics 的 canonical key**

```python
@dataclass(frozen=True, order=True)
class SnapshotKey:
    track_id: int
    generation: int

    def __post_init__(self) -> None:
        if type(self.track_id) is not int or self.track_id <= 0:
            raise ValueError("track_id must be positive")
        if type(self.generation) is not int or self.generation < 0:
            raise ValueError("generation must be a non-negative integer")

    def to_json_dict(self) -> dict[str, int]:
        return {"schema_version": 2, "track_id": self.track_id,
                "generation": self.generation}
```

`hitter_task_models.py` 从该模块 import 并 re-export `SnapshotKey`，避免一次性破坏外部 import；不得复制第二份 key class。

- [ ] **Step 4: 机械迁移 Python 属性、构造参数、JSON key 和日志标签**

把生产路径全部迁为 `track_id`。MuJoCo 的本地测试 ID 从 1 开始且只服务于离线 simulation；RealWorld 暂时保留原行为到 Task 7，但字段名已与 wire 契约一致。当前 v2 录制只写 `track_id`；旧 `track_epoch` 解析在 Task 13 放入隔离适配器，不能留在 active model。

- [ ] **Step 5: 更新现存 fixture 并证明 active tree 没有旧名称**

Run:

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
PYTHONPATH=deploy /home/loco1/miniconda3/envs/rb/bin/python -m pytest -q \
  deploy/tests/test_hitter_runtime_identity_types.py \
  deploy/tests/test_mujoco_hitter_track_id_v2.py \
  deploy/tests/test_hitter_task_events.py \
  deploy/tests/test_hitter_task_input_adapter.py
test -z "$(rg -l '\btrack_epoch\b' \
  deploy/simulator deploy/envs deploy/utils deploy/diagnostics \
  --glob '*.py' --glob '*.html')"
```

Expected: 指定测试 PASS；`rg` 返回空。

- [ ] **Step 6: 提交 canonical identity rename**

```bash
git add deploy/utils/hitter_runtime_types.py \
  deploy/utils/hitter_realtime.py deploy/diagnostics/hitter_task_models.py \
  deploy/diagnostics/hitter_task_attempts.py \
  deploy/diagnostics/hitter_task_events.py \
  deploy/diagnostics/hitter_task_pipeline.py \
  deploy/diagnostics/hitter_task_replay.py \
  deploy/diagnostics/static/hitter_task_monitor.html \
  deploy/tests/test_hitter_runtime_identity_types.py \
  deploy/tests/test_hitter_strike_target_logging.py \
  deploy/tests/test_hitter_task_events.py \
  deploy/tests/test_hitter_task_frontend.py \
  deploy/tests/test_hitter_task_pipeline.py \
  deploy/tests/test_mujoco_hitter_track_id_v2.py
git add -p -- deploy/simulator/real_world.py deploy/envs/hitter.py
git add -p -- deploy/diagnostics/hitter_task_monitor.py
git add -p -- deploy/tests/test_hitter_runtime_factory.py
git add -p -- deploy/tests/test_hitter_task_diagnostics_integration.py
git add -p -- deploy/tests/test_hitter_task_input_adapter.py
git add -p -- deploy/tests/test_hitter_task_monitor.py
git diff --cached --check
git diff --cached --name-status
git commit -m "refactor: make track id the HITTER runtime identity"
```

在执行该提交前，必须先用 `git diff --cached --name-status` 确认没有把 pre-existing deleted 文件 staged；如有，执行 `git restore --staged <明确路径>` 只取消暂存，不改变工作树。
现有 `deploy/tests/test_mujoco_physical_table_tennis.py` 是实施前用户未跟踪文件；只读参考其中 epoch case 并移植到新建的 v2 测试，禁止编辑或暂存该用户文件。

---
