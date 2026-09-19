# RobotBridge4_refactor 目录迁移后的部署核验

核验日期：2026-09-10。当前项目目录：`/home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor`。

**已修复目录移动造成的构建缓存和动态库路径问题。MuJoCo 已在新目录实际启动并运行策略；真机部署的配置、二进制加载、标定和相关离线测试已验证。没有连接机器人或 Vicon，真机闭环效果仍需现场确认。**

## 1. 移动影响及修复

| 项目 | 移动后发现的问题 | 本次处理 |
| --- | --- | --- |
| Python 主入口与配置 | `run.py` 根据自身位置定位配置；模型和资产仍使用相对路径 | 无需改代码，policy 从新项目的 `deploy/` 启动 |
| Unitree `trans` | ELF RUNPATH 仍指向旧目录，清空 `LD_LIBRARY_PATH` 后 `libddsc.so.0`、`libddscxx.so.0` 无法找到 | 备份旧构建目录，重新 CMake 配置并编译 `trans` |
| Unitree CMake 缓存 | source、build、SDK 静态库路径仍指向旧目录 | 在新位置生成全新缓存，不复用旧缓存 |
| Vicon C++ v2 桥 | RUNPATH 仍指向旧 SDK 目录，`libViconDataStreamSDK_CPP.so` 无法找到 | 重新编译 `.build-v2/` 下六个程序，写入当前路径 |
| Vicon Python helper 路线 | 默认 `bin/vicon_frame_stream` 是既有产物，原始 RUNPATH 含其他机器的历史目录 | 验证既有 `ViconSdkClient._build_env()` 会传入当前 SDK 路径，helper 的动态加载和 `--help` 成功；未改变该路线 |
| 当前使用文档 | Desktop 说明和项目 README 中的操作路径过时 | 同步新目录、Python 环境、源码链接及真机网卡占位符 |

重新构建的主部署产物为：

```text
unitree_sdk2/.build-robotbridge4-v2/bin/trans
deploy/mocap_bridge/.build-v2/vicon_table_lcm_bridge_v2
```

另重新构建 `.build-v2/` 中的 `nexus_probe_cpp`、`vicon_datastream_dump`、`vicon_frame_stream`、`test_transformation_t_v2`、`test_vicon_ball_track_v2`。本机没有系统级 `pkg-config lcm` 配置，构建继续显式使用 isaaclab 中已有的 LCM 头文件和库。

本次没有修改 Python/C++/Shell 源码、YAML、模型或标定，没有创建旧路径软链接，也没有调整全局环境。旧构建已移到外部核验目录的 `previous_builds/` 备份。新二进制适用于当前机器和当前路径；再次移动目录或换机器后，需要重新检查动态库路径及构建缓存。

## 2. 实际验证结果

| 检查 | 结果及范围 |
| --- | --- |
| 文件完整性 | 核对上次交付清单的 1,593 个文件；仅两份现有 Markdown 因本次更新而变化，其余文件一致 |
| MuJoCo 实际 GUI CLI | 连续运行至核验设定的 28 秒；仿真 0 秒、20 秒各发球一次；由核验进程发送 SIGINT 停止 |
| ONNX + MuJoCo 有限步数循环 | 完成 1,100 步、22 秒仿真、两次发球；104 维观测、29 维动作及物理状态逐步检查均为有限数 |
| 受控行为对照 | 测试进程中将生命周期时钟绑定仿真时间；1,100 步轨迹 SHA256 与迁移前相同；磁盘源码未修改 |
| Hydra 两种配置 | `sim=mujoco`、`sim=real_world` 的 `--cfg job --resolve` 均退出 0 |
| 新编译 `trans` 与 Vicon 桥 | 不依赖继承的 `LD_LIBRARY_PATH` 时，`ldd` 无 `not found`；RUNPATH 和新 CMake 缓存不再引用旧目录 |
| `trans` 加载入口 | 不传网卡参数时输出 Usage，按源码预期退出 255；未构造机器人控制对象 |
| Vicon 桥与 Python helper | `--help` 退出 0；Python helper 按既有 client 环境加载 SDK；未连接设备 |
| 标定预检 | `--check-only` 输出 `Calibration preflight: PASS`；该模式只检查文件/参数，动态加载另行核验 |
| C++ 离线测试 | v2 消息协议、球跟踪两个测试程序均退出 0 |
| Python 离线测试 | 28 passed：机器人连接等待、首帧过渡、v2 消息及 C++/Python 编解码、Vicon client；硬件接口使用测试替身 |

受控轨迹 SHA256：

```text
c647c05cfbeed76730618f059559ce08af7322b6628bc5c9b34f47f2c11a830c
```

这次检查覆盖迁移影响，没有重跑全项目测试，也不代表先前已记录的测试失败得到修复。

### 仍需明确的限制

MuJoCo 的已有退出异常仍能复现：`BaseSim.close()` 访问不存在的 `joint_state_subscriber`。GUI 在核验 SIGINT 后报告该异常并退出 `-11`；有限步数脚本完成循环后也因该关闭异常退出 1。因此上述结果确认的是启动和运行循环，不能算整个进程正常退出。迁移前已经记录同类结果，相关代码保持不变；`-11` 的底层原因未另行确定。

首次受控循环检查还在 Torch JIT 导入阶段遇到 `AttributeError: type object 'ExprBuilder' has no attribute 'value'`，尚未进入仿真。保留失败日志后，独立依赖导入成功，同一源码顺序重跑完成 1,100 步。迁移前已有偶发导入异常记录，但现有证据不足以确定本次异常的根因，不能据此声称环境完全稳定。

本轮没有执行 Vicon 实时连接、机器人 DDS 控制或真机 policy。现场的实时动捕、网卡、R2 操作及真实关节动作仍需设备验证。

## 3. 当前 MuJoCo 命令

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor/deploy
/home/yhl/miniforge3/envs/isaaclab/bin/python run.py \
  --config-name=hitter sim=mujoco device=cpu
```

默认显示窗口。无窗口运行追加 `robot.control.viewer=false`。必须从 `deploy/` 执行，以正确解析模型和场景资源的相对路径。

## 4. 当前真机部署命令

先运行离线预检：

```bash
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor
bash deploy/mocap_bridge/run_robotbridge4_vicon_real.sh --check-only
```

现场按以下顺序在三个终端启动。`YOUR_ROBOT_INTERFACE` 替换为实际机器人通信网卡名。

```bash
# 终端 1：Vicon bridge
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor
bash deploy/mocap_bridge/run_robotbridge4_vicon_real.sh
```

```bash
# 终端 2：Unitree trans
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor
LD_LIBRARY_PATH="$PWD/unitree_sdk2/thirdparty/lib/x86_64:${LD_LIBRARY_PATH:-}" \
  ./unitree_sdk2/.build-robotbridge4-v2/bin/trans YOUR_ROBOT_INTERFACE
```

```bash
# 终端 3：HITTER policy
cd /home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor/deploy
LOGURU_LEVEL=INFO PYTHONUNBUFFERED=1 \
  /home/yhl/miniforge3/envs/isaaclab/bin/python -u run.py \
  --config-name=hitter sim=real_world device=cpu
```

`trans` 按提示 Enter，policy 沿用原来的 R2 按下/释放和姿态准备流程。专用启动器仍固定使用 20260822 的球桌、骨盆标定，Vicon 地址为 `192.168.10.1:801`、LCM 通道为 `vicon_state_data_v2`，无需因目录迁移更换这些配置。

## 5. 本机核验材料

材料保存在项目外部 `/home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/`；以下链接用于当前机器。2026-09-09 的历史审计日志保留当时路径，不参与程序启动。

- [迁移前动态库检查](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/before.json)
- [本次一次性重建脚本](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/rebuild.py) / [构建命令与结果](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/build_results.json)；脚本会备份旧构建，已有同名备份时拒绝覆盖，不用于日常启动。
- [离线核验脚本](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/offline_check.py) / [结果](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/offline_results.json)
- [GUI 启动结果](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/mujoco_actual_cli-relocated.json) / [日志](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/mujoco_actual_cli-relocated.log)
- [1,100 步循环结果](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/mujoco_loop.json) / [轨迹对照](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/simulation_comparison.json) / [首次导入失败](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/mujoco_loop_initial.log)
- [28 项离线测试日志](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/offline_tests.log)
- [交付文件完整性核对](../../RobotBridge4_refactor_资料/核验材料/RobotBridge4_refactor_迁移核验/source_integrity.json)
