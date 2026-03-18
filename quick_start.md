# al-ilqr-starter

一个面向教学的 **AL-iLQR（Augmented Lagrangian iLQR）** 轨迹优化项目。

从零实现一个可运行、可验证、可视化的自动驾驶轨迹优化系统，涵盖从无约束 iLQR 到约束 AL-iLQR 的完整算法链路。

> **注意**：本仓库实现的是 AL-iLQR 主线算法，不是完整的 ALTRO 求解器。与论文的详细差异参见 [guide/chapter8_extensions.md](guide/chapter8_extensions.md#与论文的差异汇总)。

---

## 快速开始

### 环境要求

| 依赖 | 最低版本 | 说明 |
|------|---------|------|
| CMake | 3.16 | 构建系统 |
| g++ 或 clang++ | 支持 C++17 | 编译器 |
| Eigen3 | 3.3 | 线性代数库（系统安装） |
| OpenGL + GLFW + Dear ImGui | — | 交互式前端（GLFW 和 ImGui 由 CMake 自动下载） |
| Python3 + matplotlib + numpy | — | 可视化脚本（可选） |

### 一键安装依赖（Ubuntu / Debian）

```bash
git clone https://github.com/my-al-ilqr/al-ilqr-starter.git
cd al-ilqr-starter
bash scripts/install_deps.sh
```

脚本会自动安装编译工具、Eigen3、OpenGL 图形库和 Python 可视化依赖。

<details>
<summary>手动安装（如果不使用脚本）</summary>

```bash
# 编译工具和核心依赖
sudo apt-get install -y build-essential cmake libeigen3-dev

# ImGui 前端所需的图形库
sudo apt-get install -y libgl-dev libx11-dev libxrandr-dev \
  libxinerama-dev libxcursor-dev libxi-dev libwayland-dev libxkbcommon-dev

# Python 可视化（可选）
sudo apt-get install -y python3 python3-matplotlib python3-numpy
```

</details>

### 编译

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

首次编译时 CMake 会自动下载 GLFW 和 Dear ImGui（约 10 MB），后续编译不再重复下载。

如果不需要 ImGui 前端（例如无图形环境的服务器），可以关闭：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DMY_AL_ILQR_ENABLE_IMGUI_FRONTEND=OFF
```

### 启动交互式前端

```bash
./build/imgui_frontend
```

在 ImGui 窗口中可以：
- 调整车辆模型参数、约束范围、障碍物位置
- 点击 **Plan** 运行 AL-iLQR 求解
- 查看规划轨迹、速度曲线、曲率曲线、加速度曲线
- 结果自动导出到 `build/imgui_frontend_trajectory.csv`

---

## 项目结构

```
MY-AL-iLQR/
├── include/                  ← 头文件
│   ├── core/                 ← 数学类型、轨迹容器
│   ├── dynamics/             ← 动力学模型接口
│   ├── cost/                 ← 代价函数
│   ├── constraints/          ← 约束函数
│   ├── lqr/                  ← 有限时域 LQR
│   ├── ilqr/                 ← iLQR 求解器
│   ├── al/                   ← 增广拉格朗日 + AL-iLQR
│   ├── autodrive/            ← 自动驾驶场景建模
│   ├── problems/             ← OCP 问题定义
│   └── visualization/        ← CSV 导出
├── src/                      ← 源文件（与 include/ 对应）
├── apps/
│   └── rerun_imgui_frontend.cpp  ← ImGui 交互式前端
├── examples/                 ← 各阶段可执行示例
├── tests/                    ← 各阶段单元测试
├── config/
│   └── vehicle_bicycle_config.ini  ← 车辆参数配置
├── visualization/            ← Python 绘图脚本
├── guide/                    ← 开发指南文档（理论+代码详解）
├── tutorial/                 ← 独立教程工程（每章一个 CMake 工程）
├── paper/                    ← 参考论文 PDF
├── ref/                      ← 参考代码（不参与编译）
│   ├── altro-cpp/
│   └── toy-example-of-iLQR/
├── scripts/
│   └── install_deps.sh       ← 一键安装依赖脚本
└── CMakeLists.txt
```

---

## 可执行程序

### 阶段示例（examples/）

从简单到复杂，每个 phase 可独立运行：

| 程序 | 说明 |
|------|------|
| `phase0_linear_rollout` | 线性系统前向仿真 |
| `phase1_unicycle_rollout` | 独轮车模型 rollout |
| `phase1_bicycle_rollout` | 自行车模型 rollout |
| `phase2_linear_lqr_demo` | 有限时域 LQR |
| `phase3_unicycle_ilqr_demo` | 无约束 iLQR |
| `phase4_unicycle_ilqr_stable_demo` | iLQR + 正则化 + 线搜索 |
| `phase5_constraint_evaluation_demo` | 约束评估 |
| `phase6_augmented_lagrangian_demo` | 增广拉格朗日代价 |
| `phase7_al_ilqr_demo` | 完整 AL-iLQR |
| `phase8_autodrive_demo` | 自动驾驶避障 |
| `phase9_autodrive_visualization_demo` | 自动驾驶 + CSV 导出 |
| `phase10_numerical_enhancement_demo` | 数值增强 |
| `phase11_dynamic_obstacle_demo` | 动态障碍物 |
| `phase12_dynamic_obstacle_speed_sweep_demo` | 障碍物速度扫描 |

运行示例：

```bash
./build/phase8_autodrive_demo
```

### 交互式前端

| 程序 | 说明 |
|------|------|
| `imgui_frontend` | Dear ImGui 交互式规划界面 |

---


---

## 理论参考

本项目主要参考以下两篇论文：

1. **AL_iLQR_Tutorial.pdf** — AL-iLQR 主理论参考
   - 离散时间最优控制问题建模
   - iLQR 的 backward pass 和 forward pass
   - 线搜索与正则化
   - 增广拉格朗日外循环

2. **ALTRO A Fast Solver for Constrained Trajectory Optimization.pdf** — 工程增强参考
   - AL-iLQR 的简洁算法表达
   - 数值稳定性与大罚参数病态性
   - Square-root backward pass、projection polishing 等高级特性（本项目未实现）

---

## 算法核心特性

- **动力学模型**：线性点质量、独轮车、运动学自行车模型
- **代价函数**：二次代价（参考跟踪）、车道跟踪代价
- **约束类型**：控制箱约束、终端目标、速度限制、单圆障碍物避障
- **求解器**：数值差分 iLQR + 增广拉格朗日外循环
- **正则化**：自适应 Levenberg-Marquardt 风格正则化
- **线搜索**：Backtracking line search + ratio test
- **障碍物**：静态/动态障碍物，单圆车体近似
- **配置化**：车辆参数通过 INI 文件配置

---
