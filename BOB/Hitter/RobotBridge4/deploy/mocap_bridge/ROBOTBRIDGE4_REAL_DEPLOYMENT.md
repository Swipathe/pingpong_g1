# RobotBridge4 HITTER 真机部署

本部署入口固定使用会展场地于 2026-08-18 完成并验证的 Vicon 标定组合：

- 球桌：`calibrations/vicon_table_frame_20260818_validated.json`
- 机器人骨盆：`calibrations/vicon_g1_pelvis_orientation_20260818_validated.json`

启动器会在连接 Vicon 前验证两份文件的 SHA256，并检查 Policy planner 的球桌中心、
长宽高、LCM 通道和 Pelvis subject 与标定组合一致。文件缺失、内容被改动或 Policy
配置不一致时会直接退出，不会发布 LCM 数据。不要用通用的无标定 bridge 命令代替
终端 1。

## 启动前检查

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
./deploy/mocap_bridge/run_robotbridge4_vicon_real.sh --check-only
```

预期最后一行：

```text
Calibration preflight: PASS
```

如需同时检查 Vicon 实时连接、刚体可见性和两份标定的加载结果，可执行以下命令。
它只运行 3 秒且不发布 LCM：

```bash
./deploy/mocap_bridge/run_robotbridge4_vicon_real.sh --check-live
```

## 终端 1：Vicon bridge

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
./deploy/mocap_bridge/run_robotbridge4_vicon_real.sh
```

## 终端 2：机器人通信

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4
LD_LIBRARY_PATH="$PWD/unitree_sdk2/thirdparty/lib/x86_64:${LD_LIBRARY_PATH:-}" \
./unitree_sdk2/.build-robotbridge4-v2/bin/trans enx9c69d30201e2
```

启动后按一次 Enter。

## 终端 3：Policy

```bash
cd /home/loco1/BOB/Hitter/RobotBridge4/deploy
LOGURU_LEVEL=INFO \
PYTHONUNBUFFERED=1 \
conda run --no-capture-output -n rb \
python -u run.py \
  --config-name=hitter \
  sim=real_world \
  device=cpu
```

Policy 端的 planner 使用世界坐标参数，不直接读取 Vicon 标定 JSON。当前配置已与
20260818 球桌标定一致：中心 `[1.365369, 0.0] m`、高度 `0.760000 m`、长度
`2.730738 m`、宽度 `1.512451 m`。
