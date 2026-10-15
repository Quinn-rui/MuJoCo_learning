# MuJoCo 学习记录

用 MuJoCo 从零学习机器人仿真与控制基础，每个脚本对应一个小实验。

## 环境

```bash
conda activate embodied
python --version    # Python 3.8
```

主要依赖：`mujoco` / `numpy` / `matplotlib`

环境完整配方见 [`environment.yml`](./environment.yml)，重建方式：

```bash
conda env create -f environment.yml
conda activate embodied
```

## 脚本一览

### 1. 入门：加载模型 + 跑仿真

| 文件 | 说明 |
|------|------|
| `hello.xml` | 最小 MJCF 模型：一个 box |
| `hello_box_headless.py` | 无界面跑 2 秒仿真，打印 box 的世界坐标 |
| `hello_box_viewer.py` | 同上，但打开可视化窗口实时观察 |

**学到的概念**：`MjModel`（结构/参数）与 `MjData`（运行时状态）的区分；`mj_step` 按 `timestep` 推进一步。

```bash
python hello_box_headless.py
python hello_box_viewer.py      # 需要图形界面
```

### 2. 控制：单摆 PD

| 文件 | 说明 |
|------|------|
| `pendulum.xml` | 单摆模型，同时定义了 `pos_act`（position 执行器）和 `tor_act`（motor） |
| `pd_single_joint.py` | 两种控制方式对比：手写 PD 写入 `ctrl[1]` vs 用内置 position 执行器写 `ctrl[0]` |
| `pd_experiments.py` | 无界面批量实验，输出响应曲线图 |

**学到的概念**：

- 手写 PD：`tau = Kp * (q_des - q) + Kd * (0 - dq)`
- 用 motor 手搓 PD 时，要**关掉** XML 里同时存在的 position 执行器（清掉 `actuator_gainprm` / `actuator_biasprm`），否则两套 PD 会叠加
- 增益对响应的影响：欠阻尼 / 临界 / 过阻尼
- 纯 P 控制在恒定外扰下**存在稳态误差**，加 D 项或积分项才消得掉

```bash
python pd_single_joint.py       # 带可视化
python pd_experiments.py        # 输出 pd_kd_sweep.png 和 pd_disturbance_error.png
```

`pd_experiments.py` 包含两组实验：

- **实验 1**：固定 `Kp=40`，改变 `Kd`（0.5 / 12 / 40），观察欠阻尼 → 临界 → 过阻尼
- **实验 2**：3 秒后施加恒定外扰，对比纯 P（`Kd=0`）和 PD（`Kd=1`）的稳态误差

### 3. 运动学：3 自由度平面臂

| 文件 | 说明 |
|------|------|
| `planar_3dof.xml` | 3 关节平面臂，末端有 `end_site` |
| `planar_3dof.py` | 🚧 **未完成**：用自己写的 FK 和 MuJoCo 的 `site_xpos` 对比，验证正运动学推导 |

```bash
python planar_3dof.py           # 目前会报错，见下
```

> ⚠️ 这个脚本还没写完，缺两处：
> 1. 开头少了 `import numpy as np`
> 2. `fk_planar(q)` 函数尚未定义——需要自己实现 3 自由度平面臂的正运动学，返回 4×4 齐次变换矩阵
>
> 期望结果：`max |p_ours - p_mj|` 在 `1e-14 ~ 1e-12` 量级；若超过 `1e-6`，说明参数或轴向写错了。

## 学习笔记

- [环境管理与 Git 工作流指南](https://github.com/Quinn-rui/qrx-/blob/main/环境管理与Git工作流指南.md) —— 环境隔离原理、conda 管理、Git 日常工作流

## 目录结构

```
mujoco_learning/
├── README.md              # 本文件
├── .gitignore
├── environment.yml        # 环境配方
├── hello.xml              # ┐
├── hello_box_headless.py  # │ 入门实验
├── hello_box_viewer.py    # ┘
├── pendulum.xml           # ┐
├── pd_single_joint.py     # │ PD 控制实验
├── pd_experiments.py      # ┘
├── planar_3dof.xml        # ┐
└── planar_3dof.py         # ┘ 运动学实验（未完成）
```
