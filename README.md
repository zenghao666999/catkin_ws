# catkin_ws

WPB Home 机器人 ROS 工作空间，实习期间搭建和学习的测试环境。

---

## 环境

- Ubuntu 20.04 (VM)
- ROS Melodic
- Gazebo 11

---

## 包含的包

| 包名 | 说明 |
|---|---|
| imu_filter | IMU 姿态滤波，包含 Madgwick 和 Mahony 两种算法 |
| wpb_home | WPB Home 机器人核心功能包 |
| wpr_simulation | Gazebo 仿真场景 |
| abot_base | 机器人底盘驱动 |

---

## 测试相关

实习期间在 imu_filter 包上练习了测试框架搭建：

### 测试层次

1. **pytest** — 对 imu_math.py 里的数学函数写单元测试，直接运行不需要 roscore
2. **rostest** — 对 bias_calculator、Madgwick_filter、Mahony_filter 三个 C++ 节点做集成测试，需要编译和启动 roscore
3. **rosbridge** — 尝试通过 WebSocket 接口对机器人话题做远程调用（还在调试中，部分操作会超时）

### 运行方式

```bash
cd ~/catkin_ws
source devel/setup.bash
cd src/imu_filter

./run_tests.sh --direct      # 只跑 pytest
./run_tests.sh --rebuild     # 编译后跑 rostest
./run_tests.sh --api         # 尝试 rosbridge 接口（需要先启动 rosbridge_server）
```

### 已通过的测试

- imu_math.py 中 Madgwick/Mahony 四元数计算函数（pytest）
- bias_calculator 零偏估计节点编译和链接（GTest via rostest）
- Madgwick_filter、Mahony_filter 节点编译和链接（GTest via rostest）

### 遇到的问题

- rosbridge 的 twisted/bson/PIL 依赖在 ROS Melodic 下有版本冲突，已记录在 POSTMAN_SETUP.md
- Gazebo GUI 在 VM 中需要额外配置 OpenGL 渲染

---

## 启动仿真

```bash
roslaunch wpr_simulation wpb_simple.launch
```

---


