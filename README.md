# pingpong_g1

Desktop 项目迁移快照，保留各子项目原有相对目录。已上传本次快照中所有可读取文件（用户指定压缩包及 Git 历史除外）；上传统计见 [upload-status.json](.github-transfer/upload-status.json)。部分源目录和文件因本机权限不足没有纳入，详见下方范围说明。

## 在另一台机器上使用

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/Swipathe/pingpong_g1.git
cd pingpong_g1
git lfs install
git lfs pull
python .github-transfer/restore_large_files.py
```

本次 28 个超大文件已通过 25 份去重分卷上传，占用约 9.69 GiB LFS 存储；[待上传清单](.github-transfer/pending-files.json)为空。超过 GitHub 单文件限制的文件使用 LFS 分卷保存，恢复后仍位于原路径，且经过 SHA-256 校验。已有且内容不同的文件不会被覆盖。

当前主要部署项目：[BOB/Hitter/RobotBridge4_refactor](BOB/Hitter/RobotBridge4_refactor/README.md)。击球策略配置指向 `deploy/data/model/hitter/hitter_model20500_20260818_104.onnx`。模型文件本身随快照保留；本次迁移不执行训练或连接机器人。

普通 Git 仓库包含大量历史训练权重，首次克隆较大；LFS 首次完整拉取约需 9.69 GiB 下载流量，GitHub 免费流量按账号共享。仅需要代码时可以暂不执行 `git lfs pull`。

## 环境恢复

`.github-transfer/environments/` 包含本机 base、gmr、gvhmr、isaaclab 的 Conda 导出文件：`*-history.yml` 为直接安装依赖，`*-environment.yml` 为完整包版本参考。

```bash
conda env create -f .github-transfer/environments/isaaclab-environment.yml
conda activate isaaclab
cd BOB/Hitter/RobotBridge4_refactor
pip install -r requirements.txt
```

环境清单包含本机已有软件组合，并未在另一台机器实际安装验证。原代码中的绝对路径、CUDA/驱动、Isaac Sim 和机器人 SDK 仍需按目标机器调整；编译产物应按各子项目 README 重新编译。

## 快照范围

- 排除用户指定的 `BOB/Hitter.tar.gz`。
- 各嵌套仓库的工作文件按原路径纳入，原 `.git` 历史和 worktree 元数据不纳入。
- 无法读取的目录及文件列在 `.github-transfer/omissions.json`，其中含部分旧版本标定及机器人资源；这些路径没有备份成功。
- Desktop 内部绝对符号链接改为等价相对链接，记录在 `.github-transfer/symlinks.json`；一个原本指向 `/home/loco1/.../cache` 的外部链接仍需人工配置。
- 原 Desktop 未修改。`.gitattributes` 的迁移规则只存在于上传副本，用于普通 Git 文件与 LFS 分卷的正确取回。
- 本次不启用付费超额使用。遇到免费额度不足的文件留在本机并列入待上传清单。

原有的 375 个空目录记录在 `.github-transfer/empty-directories.json`，恢复脚本会一并创建。完整克隆、LFS 缓存和文件还原建议为项目预留约 120 GB 空间，Conda/CUDA 等环境另计。
