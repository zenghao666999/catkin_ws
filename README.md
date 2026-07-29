# catkin_ws — WPB Home 机器人测试工作空间

**ROS Melodic · Gazebo 11 · 三层测试架构 · Postman/Newman API 测试**

面向 **机器人测试工程师** 岗位的全栈测试工作空间，以 WPB Home 服务机器人为平台，覆盖仿真、感知、导航、抓取全链路。

---

## 工作空间结构

`
catkin_ws/
├── src/
│   ├── wpb_home/           # WPB Home 机器人核心 (bringup, behaviors, tutorials, remote)
│   ├── wpr_simulation/     # Gazebo 仿真场景 (robocup_home, navigation, manipulation)
│   ├── abot_base/          # ABOT 底盘驱动 (bringup, IMU, lidar, model)
│   ├── cam_track/          # 摄像头目标跟踪
│   └── imu_filter/         # IMU 姿态滤波 (Madgwick/Mahony) + 测试框架 🔥
├── test/
│   └── postman/            # rosbridge API 测试 (Postman collection + Python runner)
└── docs/
    ├── testing_roadmap.md  # 5+1 模块 pytest 学习路线
    └── POSTMAN_SETUP.md    # rosbridge + Newman 环境搭建指南
`

---

## 三层测试架构

`
┌─────────────────────────────────────────────┐
│ L1  pytest (纯 Python)                      │
│    函数/算法逻辑，可直接运行，无需 roscore   │
│    ./run_tests.sh --direct                  │
├─────────────────────────────────────────────┤
│ L2  rostest (ROS 集成)                      │
│    ROS 节点 + 话题通信 + GTest               │
│    ./run_tests.sh --rebuild                 │
├─────────────────────────────────────────────┤
│ L3  rosbridge + Postman (API / 远控)        │
│    WebSocket API 测试，模拟远程控制场景      │
│    ./run_tests.sh --api                     │
└─────────────────────────────────────────────┘
`

**对应岗位 JD 能力映射：**

| JD 要求 | 本仓库覆盖 |
|---|---|
| 功能、性能、稳定性测试 | L1 pytest 回归 + L2 rostest 集成 |
| 自动化测试脚本开发 | run_tests.sh 一键调度三层 |
| 测试环境搭建和维护 | Gazebo 仿真 + rosbridge 桥接 |
| 远控产品测试 | L3 Postman/Newman API 测试 |
| Linux 开发环境 | WSL2 / Ubuntu 20.04 + ROS Melodic |
| SLAM/导航算法了解 | wpr_simulation 提供建图+导航仿真场景 |

---

## 快速开始

### 环境

- Ubuntu 20.04 (WSL2 / VM)
- ROS Melodic
- Gazebo 11
- Python 3.8+ (pytest)

### 编译

`ash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
`

### 运行测试

`ash
# L1 — pytest 直接模式（无需 roscore）
cd src/imu_filter
./run_tests.sh --direct

# L2 — rostest 集成模式
./run_tests.sh --rebuild

# L3 — rosbridge API 模式（需先 roslaunch rosbridge_server）
./run_tests.sh --api
`

### 启动仿真

`ash
roslaunch wpr_simulation wpb_simple.launch    # 基础场景
roslaunch wpr_simulation wpb_navigation.launch # 导航场景
`

---

## IMU Filter 测试详情

| 被测目标 | 语言 | 框架 | 验证内容 |
|---|---|---|---|
| ias_calculator | C++ | GTest | 零偏估计收敛性 |
| Madgwick_filter | C++ | GTest | 四元数融合精度 |
| Mahony_filter | C++ | GTest | 互补滤波稳定性 |
| imu_math | Python | pytest | 数学工具函数 |
| invsqrt | C++ | GTest | 快速反平方根精度 |

---

## API 测试 (rosbridge + Postman)

`ash
# 1. 启动 rosbridge
roslaunch imu_filter rosbridge_test.launch

# 2. 运行测试
cd test/postman
python3 rosbridge_test_runner.py

# 3. 或导入 Postman
# 打开 Postman → Import → test/postman/robot_api_tests.postman_collection.json
`

---

## Git 提交规范

`
feat: add rostest+pytest framework for imu_filter
feat: add rosbridge + Postman/Newman API testing layer
docs: add testing roadmap and Postman setup guide
`

---

## 待扩展

- [ ] 激光雷达传感器测试
- [ ] 摄像头 (cam_track) 测试
- [ ] CI 集成 (GitHub Actions + Jenkins)
- [ ] rosbag 回放回归测试
- [ ] Docker 容器化测试环境
