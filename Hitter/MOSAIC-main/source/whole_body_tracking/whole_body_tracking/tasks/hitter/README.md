# HITTER 任务结构

当前正式训练任务是 `Hitter-Striking-PlannerDomain-Flat-G1-v0`。

## 任务注册

- `config/g1/__init__.py`
  - 注册 G1 平地击球训练任务。

## 环境配置

- `hitter_env_cfg.py`
  - 定义通用 HITTER scene、observation、reward、termination、event 配置。
- `config/g1/flat_env_cfg.py`
  - 绑定 G1 球拍机器人资产。
  - 设置 planner-domain 训练指令范围、reward 权重和随机化开关。
- `config/g1/agents/rsl_rl_ppo_cfg.py`
  - RSL-RL PPO 训练配置。

## MDP 项

- `mdp/commands.py`
  - HITTER 随机 WBC command sampler。
  - 维护 base target、racket target、击球时间、可视化 marker 和指标。
- `mdp/observations.py`
  - policy/critic 观测中 HITTER 专用项。
- `mdp/rewards.py`
  - base、racket、joint reference imitation 和正则化 reward。
- `mdp/terminations.py`
  - HITTER 专用提前终止项。
- `mdp/actions.py`
  - 备用 action 扩展入口；当前正式训练没有启用。
- `mdp/__init__.py`
  - 汇总 IsaacLab、MOSAIC tracking 和 HITTER MDP 项。

## 资产依赖

- `robots/g1.py`
  - G1 机器人和 G1+球拍 articulation 配置。
- `assets/unitree_description/usd/g1_hitter_racket/main.usda`
  - 默认使用的旧手握球拍 G1 USD 资产。
- `assets/unitree_description/usd/g1_hitter_racket_cad_v1/main.usd`
  - 可通过 `HITTER_G1_HITTER_RACKET_USD_PATH` 切换的新刚性连接件、球拍套和球拍资产。
- `assets/unitree_description/urdf/g1_hitter_racket_cad_v1/main.urdf`
  - 新 CAD 组件对应的独立 URDF 源文件。
- `assets/unitree_description/usd/g1_hitter_racket/target_marker.usda`
  - racket target 可视化拍面。
- `assets/unitree_description/urdf/g1/pelvis_target_marker.urdf`
  - base target 可视化 pelvis marker。
- `data/hitter_motions/iphone_manual_hitter_g1_npz_strike43_clean`
  - 当前训练使用的 forehand/backhand motion 数据。

## 当前不参与训练

`mdp/actions.py` 默认不在当前 PPO 训练入口中使用。
