# HITTER model56600 ONNX 转换与部署设计

## 目标

将 `/home/loco1/Downloads/model_56600.pt` 转换成 RobotBridge4 可直接加载的 ONNX，并将生产 HITTER 配置切换到新模型。保留当前 model17500，以便快速回退；不自动重启正在运行的真机 policy。

## 已确认的兼容合同

- checkpoint SHA256 为 `4925539287da2e914f47b5be62f2c5dd29def8377e88205e04c6fdec8409574e`，内部迭代号为 `56600`。
- actor 为 `104 -> 512 -> 256 -> 128 -> 29` 的 ELU MLP；critic 输入为 301 维。
- checkpoint 的 policy normalizer 为 104 维，与当前 model17500 的 normalizer 逐元素相同。
- RobotBridge4 需要固定的 `float32 obs[1,104] -> actions[1,29]` 合同，以及关节顺序、PD 参数、默认姿态、action scale、观测名称等 11 项 ONNX metadata。
- 用户将该 checkpoint 描述为最新训练模型；结合完全相同的结构和 normalizer，本次按 model17500 同一训练链的后续 checkpoint 处理。

## 导出方案

复用 model17500 已保存的 MOSAIC source、参数和 motion 快照，通过 `scripts/rsl_rl/play.py` 及项目 `_OnnxMotionPolicyExporter` 导出 opset 18 ONNX。该路径会把 checkpoint normalizer 包进计算图，并由环境附加真机所需 metadata，避免裸 `torch.onnx.export` 遗漏部署合同。

新产物使用唯一文件名：

`hitter_model56600_8x4090_poswin001_velstd18_velwin003_104_20260814.onnx`

它会保存在 MOSAIC 独立导出目录，并复制到 RobotBridge4 的 `deploy/data/model/hitter/`。同时生成同名 `.provenance.txt`，记录源 checkpoint 哈希、导出环境、接口和验证边界。不会覆盖 `hitter.onnx` 或 model17500。

## 验证与切换

导出后执行以下验证：

1. `onnx.checker` 通过，opset 为 18。
2. ONNX Runtime 显示输入 `obs[1,104]`、输出 `actions[1,29]`，且 11 项部署 metadata 完整。
3. 对零值、固定递增值和固定随机种子输入，PyTorch checkpoint actor 与 ONNX Runtime 输出数值一致且均为有限值。
4. 配置只将 `deploy/config/mimic/hitter.yaml` 的 `policy.checkpoint` 改为新文件；其他当前未提交改动保持不变。
5. 检查目标文件 SHA256、配置解析结果和精确差异。

## 失败与回退

- exporter、metadata 或数值一致性任一验证失败时，不修改生产模型指针。
- 切换完成后若真机表现不符合预期，可把 `policy.checkpoint` 一行改回现有 model17500 文件。
- 本任务只证明 checkpoint 可加载、接口一致和推理数值一致，不证明 MuJoCo 效果或真机击球质量。
- 当前正在运行的 policy 不会热加载新 ONNX；只有人工停止旧 policy 并重新启动后，新模型才生效。

## 完成标准

- 新 ONNX 与 provenance 文件存在于 RobotBridge4 模型目录。
- ONNX 结构、metadata、有限输出和 PyTorch/ONNX Runtime 数值一致性验证全部通过。
- `hitter.yaml` 只在模型指针上切换到 model56600，旧 model17500 仍可回退。
- 未启动、停止或重启真机控制进程。
