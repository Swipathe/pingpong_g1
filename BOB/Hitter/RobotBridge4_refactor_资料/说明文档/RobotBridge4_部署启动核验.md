# RobotBridge4 重构后部署启动核验

> 2026-09-19 资料整理：本文已从 Desktop 根目录移入资料目录，相关链接已更新。[资料总索引](../README.md)。历史核验日期与结论保留。

> 2026-09-10 路径更新：项目已移动至 `/home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor`。下方操作命令和源码链接已同步；2026-09-09 的核验结果保留为历史记录。最新结果见 [迁移部署核验](../../RobotBridge4_refactor/docs/relocation_20260910.md)。

核验日期：2026-09-09。重构副本：`/home/yhl/Desktop/RobotBridge4_refactor`，源码提交：`a4a3f20`。对照项目：`/home/yhl/Desktop/BOB/Hitter/RobotBridge4` 的当前磁盘版本。

**结论：MuJoCo 已实际启动并执行策略循环；本次结构调整没有发现启动路径或运行逻辑回归。真机部署的本机离线准备已检查并补齐，但没有连接设备，不能据此确认真机闭环正常，也不能称整个项目零错误。**

## 1. 实际检查结果

| 检查 | 本轮结果 | 能确认的范围 |
| --- | --- | --- |
| 启动入口、构建文件、依赖清单及配置 | 73 个文件与重构前 SHA256 相同 | 入口、配置值和构建定义未被重构改变 |
| MuJoCo 原 CLI、默认 viewer/实时节拍 | 副本连续运行到 28 秒，日志包含 0 秒和 20 秒两次发球；之后由核验进程发送 SIGINT | 实际 `run.py → HitterAgent → HitterEnv → Mujoco` 已启动并进入循环；不代表正常退出 |
| 真实 ONNX + 环境 + 物理循环 | 原项目和副本各完成 1,100 步、22 秒仿真、两次发球 | `obs [1,104] → actions [1,29]`；每步观测、动作、qpos、qvel、ctrl 均为有限数 |
| 固定测试时钟的行为对照 | 1,100 步的上述数组逐字节一致 | 对受控场景的行为等价证据；时钟替换仅在外部测试脚本中发生 |
| MuJoCo 与 real_world Hydra 配置 | 实际 CLI 的 `--cfg job --resolve` 均退出 0 | 两种启动配置可展开；real_world 此检查不实例化控制对象 |
| Unitree `trans` | 本轮新编译通过 | 可写副本先前没有复制旧构建目录，现已补齐本机通信层二进制 |
| Vicon bridge / trans 动态库 | `ldd` 均无 `not found` | 本机动态库解析完整 |
| Vicon 启动器 | `--check-only` 通过；桥接二进制 `--help` 退出 0 | 当前 20260822 标定、planner 参数及命令入口一致，不连接设备或发布数据 |
| 真机启动相关单元测试 | 重构前后均为 118 passed、3 failed | 121 项结果逐项一致，无新增失败、缺失或变化结果 |
| 真机现场链路 | 未执行 | 未验证 Vicon 实时数据、机器人网卡/DDS、R2 操作和真实关节控制 |

本轮没有改动部署源码或配置。只新增本机 `trans` 构建产物、局部 Git 忽略项和外部核验材料；未修改既有 Python 环境的软件包。原项目 1,557 个已记录可读文件，以及上次交付清单中的 1,593 个文件均再次核对一致。

## 2. MuJoCo 仍使用原启动方式

在本机直接使用本次核验的解释器：

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor/deploy
/home/yhl/miniforge3/envs/isaaclab/bin/python run.py \
  --config-name=hitter sim=mujoco device=cpu
```

如果已经激活原来可用的部署环境，仍可使用 `python run.py --config-name=hitter sim=mujoco device=cpu`。需要从 `deploy/` 目录执行，因为模型和资产使用相对路径。不要直接使用当前 base 环境代替已安装部署依赖的环境。

本轮实际 CLI 只额外覆盖了 `hydra.run.dir`，把核验日志写到 `/tmp`；viewer、实时节拍、模型、规划和物理配置均使用原值。外部有限步数检查另外关闭 viewer 和实时等待以加快核验，没有改配置文件。

模型文件仍为 `data/model/hitter/hitter_model20500_20260818_104.onnx`，未更换模型。有限步数检查不是只做零输入推理，而是每步运行真实观测、策略推理、动作变换和 MuJoCo 物理步进。未评价击球成功率或长期稳定性。

## 3. 真机部署顺序保持不变

```mermaid
flowchart LR
    A["固定标定与配置预检"] --> B["终端 1：Vicon bridge"]
    B --> C["终端 2：Unitree trans"]
    C --> D["终端 3：HITTER policy<br/>sim=real_world"]
    D --> E["沿用现场 Enter / R2 操作<br/>姿态过渡后进入控制"]
```

本机副本的离线检查命令已实际通过：

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor
bash deploy/mocap_bridge/run_robotbridge4_vicon_real.sh --check-only
```

部署现场仍按三个终端操作。以下命令是部署说明，本轮没有执行会连接设备或发送控制指令的分支。

终端 1：

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor
bash deploy/mocap_bridge/run_robotbridge4_vicon_real.sh
```

终端 2：把 `YOUR_ROBOT_INTERFACE` 替换为原来现场验证过的机器人通信网卡名；不要照抄另一台电脑的网卡名。

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor
LD_LIBRARY_PATH="$PWD/unitree_sdk2/thirdparty/lib/x86_64:${LD_LIBRARY_PATH:-}" \
  ./unitree_sdk2/.build-robotbridge4-v2/bin/trans YOUR_ROBOT_INTERFACE
```

终端 3：

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor/deploy
LOGURU_LEVEL=INFO PYTHONUNBUFFERED=1 \
  /home/yhl/miniforge3/envs/isaaclab/bin/python -u run.py \
  --config-name=hitter sim=real_world device=cpu
```

HITTER 后端选择参数是 `sim=real_world`。原来的关节映射、LCM 通道、标定文件、球桌参数、首帧 5 秒过渡和 R2 操作代码均保留。`trans` 按提示 Enter；Policy 按原流程按下并释放 R2，在有效骨盆状态满足条件后进入策略。

这些路径针对本机可写副本；迁移到机器人或另一台部署机时，改为那台机器的实际目录和已验证环境。新编译的 `trans` 是本机 x86_64 产物，不能把“本机编译通过”视为目标机器验证通过。

## 4. 本轮补齐的构建

现在副本内已有：

```text
unitree_sdk2/.build-robotbridge4-v2/bin/trans
deploy/mocap_bridge/.build-v2/vicon_table_lcm_bridge_v2
```

第二项在上次重构核验时已构建，本轮补齐第一项。由于本机没有系统级 LCM 开发包配置，编译显式使用既有 isaaclab 包中的头文件和库，没有修改项目 CMakeLists 或增加兼容逻辑：

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor
cmake -S unitree_sdk2 -B unitree_sdk2/.build-robotbridge4-v2 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS=-I/home/yhl/miniforge3/envs/isaaclab/lib/python3.10/site-packages/include \
  '-DCMAKE_EXE_LINKER_FLAGS=-L/home/yhl/miniforge3/envs/isaaclab/lib/python3.10/site-packages/lib -Wl,-rpath,/home/yhl/miniforge3/envs/isaaclab/lib/python3.10/site-packages/lib'
cmake --build unitree_sdk2/.build-robotbridge4-v2 --target trans -j2
```

构建目录不纳入重构源码补丁。换机器后需要根据目标环境重新构建，不能依赖当前副本的绝对 rpath。

## 5. 尚未消除的问题

### MuJoCo 退出错误在重构前已经存在

原项目和副本在完成有限步数后调用 `HitterEnv.close()`，都会出现：

```text
AttributeError: 'Mujoco' object has no attribute 'joint_state_subscriber'
```

原因定位到 [BaseSim.close](../../RobotBridge4_refactor/deploy/simulator/base_sim.py:166) 使用 `joint_state_subscriber`，而 [Mujoco._init_communication](../../RobotBridge4_refactor/deploy/simulator/mujoco.py:276) 创建的是 `teleop_state_subscriber`。`BaseSim` 属于此次逐字节保留的原文件。两个实际 GUI CLI 在核验发送 SIGINT 后均输出该异常，并以 `-11` 退出；段错误的底层原因没有进一步确定。

因此可以确认启动和循环跑通，不能把整个进程退出码算作通过。此前确认的重构边界不包含顺手修复既有缺陷，本轮没有静默修改该退出逻辑。

### 三项真机相关测试失败保持原状

本轮对照的失败都位于 `test_real_world_v2_consumer.py`：

- `test_early_new_id_is_consumed_and_does_not_revive_after_gate_time`
- `test_quarantined_new_track_cuts_old_window_without_adding_own_sample`
- `test_every_valid_ball_clears_no_ball_timer_even_when_quarantined`

它们涉及轨迹准入、隔离窗口和无球计时，不应仅凭“重构前也失败”就认定没有实际影响。本轮确认它们没有因重构产生新的结果差异，没有裁决这些旧断言与现有策略的正确性。

### 本机环境存在偶发启动异常

重复核验时，原项目一次 GUI 启动在输出日志前以 `-11` 退出，两个有限步数进程还在 Torch/标准库导入期间异常。原始失败日志保留为 `*_initial.*`。随后从 `/tmp` 执行三次独立依赖导入均成功，同一源码顺序重跑后也能完成检查。

这些证据不足以确定偶发异常的根因，不能声称环境完全稳定；没有通过更改项目代码或训练环境来掩盖问题。

### 设备和数据范围

没有运行真机 policy、启动 DDS 控制或向动捕通道发布消息。当前两份 20260822 标定已复制且预检通过；上次记录的 12 个其他不可读标定仍未复制。现场仍需验证实时 Vicon、机器人状态通信、按键交互和真实动作。

## 6. 对照方法与证据

未替换时钟的仿真包含 `time.monotonic()`，原项目重复运行自身的完整轨迹哈希也不同。为了区分执行时序与代码变化，额外在测试进程里将 HitterEnv 的时钟绑定到仿真时间；没有修改磁盘源码。受控对照的 1,100 步观测、动作、位置、速度和控制数组总 SHA256 相同：

```text
c647c05cfbeed76730618f059559ce08af7322b6628bc5c9b34f47f2c11a830c
```

这只支持该受控场景的等价性，不能代替原时钟运行或真机性能验收。最初测试脚本误写模型输入名为 `actor_obs`，已按实际模型的 `obs` 修正；该失败保存在 `harness_initial_attempt/`，没有改动项目输入接口。

- [73 个入口与配置哈希](../核验材料/RobotBridge4_部署启动核验/entrypoint_config_hashes.json)
- [实际 MuJoCo CLI 日志](../核验材料/RobotBridge4_部署启动核验/mujoco_actual_cli.log)
- [原项目有限步数结果](../核验材料/RobotBridge4_部署启动核验/mujoco_loop_before.json) / [副本结果](../核验材料/RobotBridge4_部署启动核验/mujoco_loop_after.json)
- [受控仿真对照](../核验材料/RobotBridge4_部署启动核验/simulation_comparison.json)
- [真机启动相关测试对比](../核验材料/RobotBridge4_部署启动核验/real_startup_test_comparison.json)
- [trans 构建命令与结果](../核验材料/RobotBridge4_部署启动核验/trans_build.json)
- [标定、动态库和 bridge 入口检查](../核验材料/RobotBridge4_部署启动核验/real_deploy_preflight.json)
- [源码和上次交付清单完整性检查](../核验材料/RobotBridge4_部署启动核验/source_integrity.json)
- [有限步数核验脚本](../核验材料/RobotBridge4_部署启动核验/sim_loop.py) / [CLI 核验脚本](../核验材料/RobotBridge4_部署启动核验/cli_start.py)
