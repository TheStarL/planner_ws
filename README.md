# planner_ws — 机器人规划框架实践（任务15）

复现 **Fast-Planner**（Kinodynamic A\* 前端 + B样条/ESDF 梯度优化后端）与 **EGO-Planner**
（无 ESDF 梯度优化核心）的核心思想，并在 ROS 2 中搭建“感知-规划-控制”全链路闭环、动态滚动重规划、
有限视场感知建图，以及可复现的性能评估。

> 本 README 是**团队协作主文档**：包含每个文件的职责、运行方式、参数、可视化说明、代码结构与后续改进点。
> 详细实验数据见 [`report/性能评估报告.md`](report/性能评估报告.md)。

- **环境**：Ubuntu 22.04 · ROS 2 Humble · colcon · C++17（仅依赖 `rclcpp` 与标准消息，无第三方库）

---

## 1. 快速开始

```bash
cd ~/planner_ws
colcon build --symlink-install
source install/setup.bash

# 任选一个 demo 并打开 RViz（demo 见下表节点名）
ros2 launch planner_core_demo planner_demo.launch.py demo:=ego_planner_demo
```

---

## 2. 目录结构

```
planner_ws/
├── README.md                         # 本文档（项目主文档）
├── report/
│   ├── 性能评估报告.md                # 任务⑤评估报告（含实验环境对照表，先读这里）
│   └── benchmark_results.csv          # planner_benchmark 导出的原始数据
└── src/
    ├── planner_core_demo/             # 核心包：前端/后端/EGO/闭环/评估/感知建图
    │   ├── CMakeLists.txt             # 用 foreach 统一编译所有可执行；安装 launch/ 与 rviz/
    │   ├── package.xml                # 依赖声明（rclcpp + 标准消息 + launch/rviz 运行依赖）
    │   ├── launch/
    │   │   ├── planner_demo.launch.py # 启动指定 demo + RViz（参数 demo / rviz）
    │   │   └── benchmark.launch.py    # 启动批量评估（参数 num_trials / output_csv）
    │   ├── rviz/
    │   │   └── planner_demo.rviz       # RViz 配置：订阅三个 Marker 话题
    │   └── src/
    │       ├── random_map_demo.cpp     # 随机盒式障碍地图生成与发布（基础）
    │       ├── astar_grid_demo.cpp     # 3D 栅格 A\* 基线（对照 Kinodynamic）
    │       ├── kino_astar_demo.cpp     # 任务① Fast-Planner 前端 Kinodynamic A\*
    │       ├── bspline_backend_demo.cpp# 任务② 真实 ESDF 距离场 + B样条梯度优化后端
    │       ├── ego_planner_demo.cpp    # 任务③ EGO(无ESDF) vs ESDF 后端效率对比
    │       ├── closed_loop_demo.cpp    # 任务④ 感知-规划-控制闭环（含动态障碍+滚动重规划+噪声）
    │       ├── planner_benchmark.cpp   # 任务⑤ 批量随机评估，导出 CSV
    │       └── mapping_demo.cpp        # 环境升级：有限视场传感器 + 增量占据/ESDF 建图
    └── planner_visualization_demo/    # 辅助包：最简 RViz Marker 教学示例
        ├── CMakeLists.txt
        ├── package.xml
        └── src/marker_demo.cpp         # 发布一条折线 + 起终点球（RViz 入门）
```

---

## 3. 各文件职责与运行方式

所有规划节点都在统一的 2D/3D 障碍场景中工作，发布 `visualization_msgs/MarkerArray` 供 RViz 显示。
节点参数大多写死在各自源文件的 `initParams()` 中（便于查看与修改）；下表“可调参数”指通过 ROS 参数暴露、
可在命令行 `-p` 覆盖的项。

### 3.1 `planner_core_demo` 包

| 节点 | 任务 | 功能 | 发布话题 | 可调 ROS 参数 |
|---|---|---|---|---|
| `kino_astar_demo` | ① | **前端**：在 (位置,速度) 状态空间用离散加速度运动基元做 A\* 搜索，生成满足 v/a 约束的动力学可行路径；打印规划时间/扩展节点数/长度/最小安全距离等指标 | `/planner_core_demo/markers` | 无（改 `initParams()`） |
| `bspline_backend_demo` | ② | **后端**：构建真实 **ESDF**（栅格+三线性插值）；对 B样条控制点做梯度下降优化（平滑+碰撞(ESDF梯度)+可行性）；打印优化前后平滑性/安全性/最大加速度对比；可视化 ESDF 切片 | `/planner_core_demo/markers` | 无 |
| `ego_planner_demo` | ③ | **EGO 核心**：同一前端路径上并行跑 ESDF 后端与 EGO(无ESDF, `{p,v}`解析梯度) 后端，打印两者耗时/加速比与轨迹质量 | `/planner_core_demo/markers` | 无 |
| `closed_loop_demo` | ④ | **全链路闭环**：地图→Kino A\* 前端→EGO 后端→时间分配→PD+前馈跟踪。默认开启**动态障碍 + 0.25 s 滚动重规划 + 控制延迟/噪声/扰动**；可关回静态理想基线 | `/planner_core_demo/closed_loop_markers`、`/closed_loop_demo/odom` | `dynamic`(bool)、`disturbance`(bool)、`replan_period`(s) |
| `planner_benchmark` | ⑤ | **批量评估**：在 N 组随机地图上跑完整管线，统计前端成功率/避障成功率/规划时间/轨迹长度/平滑度/最小距离/EGO-ESDF 加速比，导出 CSV 后退出 | 无（批处理） | `num_trials`(int)、`output_csv`(str) |
| `mapping_demo` | 升级 | **有限视场感知建图**：真值世界对规划器不可见；激光式传感器(限距+限视场+遮挡)增量构建三态占据栅格与 2D ESDF；只在已知地图上规划，发现障碍即重规划；真值碰撞单独统计 | `/planner_core_demo/mapping_markers`、`/mapping_demo/odom` | `sensor_range`(m)、`sensor_fov_deg`(°) |
| `random_map_demo` | 基础 | 随机生成盒式障碍地图并发布（教学/地图生成基础） | `/planner_core_demo/markers` | 无 |
| `astar_grid_demo` | 基础 | **3D 栅格 A\***（占据栅格上的标准 A\*），作为 Kinodynamic 前端的对照基线 | `/planner_core_demo/markers` | 无 |

### 3.2 `planner_visualization_demo` 包

| 节点 | 功能 | 发布话题 |
|---|---|---|
| `marker_demo` | 最简 RViz Marker 示例（一条 LINE_STRIP 折线 + 起终点 SPHERE），用于熟悉 Marker 可视化 | `/planner_demo/markers` |

### 3.3 运行命令

```bash
# 通用：启动某个 demo + RViz（demo 取上表任一 planner_core_demo 节点名）
ros2 launch planner_core_demo planner_demo.launch.py demo:=kino_astar_demo
ros2 launch planner_core_demo planner_demo.launch.py demo:=bspline_backend_demo
ros2 launch planner_core_demo planner_demo.launch.py demo:=ego_planner_demo
ros2 launch planner_core_demo planner_demo.launch.py demo:=mapping_demo
ros2 launch planner_core_demo planner_demo.launch.py demo:=ego_planner_demo rviz:=false  # 不开 RViz

# 任务④ 闭环：默认动态+重规划+噪声
ros2 launch planner_core_demo planner_demo.launch.py demo:=closed_loop_demo
# 切回静态理想基线：
ros2 run planner_core_demo closed_loop_demo --ros-args -p dynamic:=false -p disturbance:=false

# 感知建图：调节传感器
ros2 run planner_core_demo mapping_demo --ros-args -p sensor_range:=2.5 -p sensor_fov_deg:=90

# 任务⑤ 批量评估并导出 CSV
ros2 launch planner_core_demo benchmark.launch.py num_trials:=30 \
     output_csv:=$HOME/planner_ws/report/benchmark_results.csv

# 单独运行某节点（不经 launch）
ros2 run planner_core_demo ego_planner_demo
```

---

## 4. RViz 可视化图例

`planner_demo.rviz` 同时订阅三个话题（固定坐标系 `map`），按当前运行的 demo 看对应图层：

| 话题 | 来自 | 主要 Marker 颜色 |
|---|---|---|
| `/planner_core_demo/markers` | 前端/后端/EGO/地图/栅格 demo | 障碍(红)、起点(绿)/终点(蓝)、前端路径(绿)、ESDF 轨迹(橙)、EGO 轨迹(蓝)、ESDF 切片(蓝→红渐变) |
| `/planner_core_demo/closed_loop_markers` | `closed_loop_demo` | 静态障碍(红)/动态障碍(橙)、参考轨迹(黄)、实际轨迹(青)、机器人(橙球)、期望状态(品红) |
| `/planner_core_demo/mapping_markers` | `mapping_demo` | 真值世界(半透明灰幽灵)、已知占据(红立柱)/已知自由(绿地块)、传感器视场(黄扇形)、参考/实际轨迹、机器人 |

> 注：当前为无显示环境时只能验证节点端逻辑；在带桌面的机器上 `mapping_demo` 可直观看到地图“边走边点亮”。

---

## 5. 算法模块说明（简要）

- **前端 Kinodynamic A\***：状态 `(p, v)`，控制量为离散加速度 `a∈{-amax,0,amax}³`，固定时长 `dt` 推进；
  代价 `g=Σ(dt+ρ‖a‖²dt)`，启发为到目标时间下界 + 速度对齐项；`isStateValid` 强制 `‖v‖≤vmax` 与无碰撞。
- **后端 B样条优化**：前端路径→均匀三次 B样条控制点；最小化 `J=λ_s·J_smooth+λ_c·J_collision(+λ_f·J_feasible)`。
  - **ESDF 方式（`bspline_backend_demo`）**：构建欧氏符号距离场栅格，碰撞梯度由场插值+中心差分得到。
  - **EGO 方式（`ego_planner_demo` / `closed_loop_demo` / `mapping_demo`）**：不建 ESDF，对碰撞控制点即时取
    锚点 `p`、排斥方向 `v`，`d=(q-p)·v`，梯度 `-2λ_c(d_safe-d)v`，解析无场查询。
- **时间分配**：按 B样条采样点累计弧长 / 巡航速度赋时间戳。
- **控制**：PD + 加速度前馈跟踪时间参数化轨迹，二阶积分器模型，带 v/a 限幅。
- **滚动重规划（`closed_loop_demo`）**：每 `replan_period` 用 EGO 后端热启动局部重优化，并把跟踪时间重锚定。
- **感知建图（`mapping_demo`）**：扇形射线扫描(遮挡)→三态占据栅格→chamfer 增量 ESDF→只在已知地图规划→发现障碍触发前端全重规划。

---

## 6. 实验环境与关键结论

⚠️ **报告中的指标来自 4 套不同环境，并非全部同源**，横向对比前请阅读
[`report/性能评估报告.md`](report/性能评估报告.md) 开头的“实验环境对照表”。概要结论：

- **任务② 后端 ESDF 优化**：平滑度代价 ↓ ~135×，最小障碍距离 0.286→0.543 m，最大加速度 6.4→2.36 m/s²，三目标同步改善且无碰撞。
- **任务③ EGO vs ESDF**：平均加速比 ≈ **42×**（省去 ESDF 距离场构建），轨迹质量相当。
- **任务④ 闭环**：静态与动态(37 次实时重规划+噪声)两模式均 RMS ≈ 0.13 m、0 碰撞到达。
- **感知不作弊（mapping）**：3.5 m/120° 传感器，初始一无所知，沿途发现 14/23 障碍、3 次前端全重规划，**0 次真值碰撞**到达。
- **任务⑤ 批量(30 组随机)**：前端成功率 100%，避障成功率 96.7%。

---

## 7. 代码结构与协作约定

- **单文件自包含**：每个 demo 是一个独立 `.cpp`，含一个 `rclcpp::Node` 子类，在构造函数里
  `initParams()`→生成场景→规划，再用 `wall_timer` 周期发布/控制。便于单独阅读与修改，代价是
  前端/后端代码在多个文件间有**重复**（见下方改进点）。
- **参数集中在 `initParams()`**：调参（地图范围、vmax/amax、`d_safe`、各 `lambda`、迭代数、传感器等）改这里。
- **新增可执行文件**：把 `.cpp` 放进 `planner_core_demo/src/`，并在 `CMakeLists.txt` 的
  `PLANNER_DEMO_TARGETS` 列表里加一行节点名即可（`foreach` 会自动编译+安装）。
- **新增话题图层**：在 `rviz/planner_demo.rviz` 增加一个 `MarkerArray` 显示项。
- **坐标系**：统一 `map`；高度方向 3D demo 用 z∈[0.25,3.5]，`mapping_demo` 固定 z=1.0 平面。
- **编码风格**：跟随现有文件（2 空格缩进、`Vec3`/`BoxObstacle` 结构体、`computeXxx` 命名）。

---

## 8. 后续改进方向（TODO）

**算法**
- [ ] 前端提速：当前 Kino A\* 单次 ~480 ms。可引入 JPS/分层启发式、OBVP 最优控制启发式，或缩减运动基元/碰撞子步。
- [ ] 后端优化器：把手写梯度下降换成 **L-BFGS**（更快更稳，免去步长/位移限幅调参）。
- [ ] 重规划支持**同伦类切换**：当通道被完全封死时，局部形变不够，需要拓扑级重规划（目前由前端全重规划兜底）。
- [ ] `mapping_demo` 升级为 **3D 体素占据 + 3D ESDF**（现为 2.5D 立柱近似）。

**仿真环境**
- [ ] 把动态障碍 / 感知建图场景纳入 `planner_benchmark`，统计动态避障成功率与探索覆盖率。
- [ ] 增加**窄通道/迷宫/森林**等难度分级地图，压力测试避障成功率。
- [ ] 更真实机器人模型：差速 / Ackermann / 四旋翼动力学 + 姿态环（替换二阶积分点质量）。
- [ ] 场景与参数**从 YAML/ROS 参数加载**，替代写死在 `initParams()`（便于批量实验）。

**工程化**
- [ ] 抽出公共库：把前端 Kino A\*、B样条、ESDF、度量函数提到 `include/planner_core_demo/` 的共享头/库，消除多文件重复。
- [ ] 定义自定义消息（轨迹/控制点）并发布，便于与真实控制器或 Gazebo 对接。
- [ ] 增加单元测试（`ament_cmake_gtest`）与 CI；接入 `rosbag` 录制评估。

