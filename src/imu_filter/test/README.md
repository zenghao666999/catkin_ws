# Project 1: C++ 单元测试基础

## 你在学什么

这个目录下有 2 个测试文件，每个文件里有带 `TODO` 标记的练习。
你的任务是**填充这些 TODO**，使测试运行并通过。

完成后你将掌握：
- gtest 的 4 种核心断言：`EXPECT_EQ`, `EXPECT_NEAR`, `EXPECT_TRUE`, `EXPECT_FLOAT_EQ`
- 测试命名规范：`TEST(TestSuiteName, TestName)` —— Suite 是"被测对象"，Name 是"测什么场景"
- 测试夹具：`TEST_F()` + `SetUp()` —— 在多个测试间共享初始化代码
- 浮点比较的正确方式
- 纯函数提取的思路

## 文件结构

```
imu_filter/
├── include/imu_filter/
│   └── math_utils.h        ← 这是我从 Madgwick_filter.cpp 和 bias_calculator.cpp 里提取的纯函数
├── test/
│   ├── test_invsqrt.cpp     ← 练习 1-3: 最简单的测试，从这开始
│   ├── test_bias_calculator.cpp ← 练习 4a-4d: 带测试夹具的略复杂案例
│   └── README.md            ← 你正在看的文件
├── CMakeLists.txt           ← 已添加 catkin_add_gtest 构建规则
└── package.xml              ← 已添加 <test_depend>rostest</test_depend>
```

## 如何使用

### 1. 打开第一个测试文件

打开 `test/test_invsqrt.cpp`，阅读注释，找到所有 `// TODO` 标记。

### 2. 填写断言

每个 TODO 是一个空白断言语句。根据注释中的提示填写。

**断言速查表：**

| 断言 | 用途 | 示例 |
|------|------|------|
| `EXPECT_EQ(a, b)` | 整数/字符串精确相等 | `EXPECT_EQ(4, 2+2)` |
| `EXPECT_NEAR(a, b, tol)` | 浮点近似相等（最常用！） | `EXPECT_NEAR(0.5, invSqrt(4.0), 0.001)` |
| `EXPECT_TRUE(cond)` | 布尔条件为真 | `EXPECT_TRUE(result > 0)` |
| `EXPECT_FALSE(cond)` | 布尔条件为假 | `EXPECT_FALSE(ok)` |
| `EXPECT_DOUBLE_EQ(a, b)` | double 精确比较（4 ULPs） | `EXPECT_DOUBLE_EQ(0.1, bx)` |
| `ASSERT_NEAR(a, b, tol)` | 同 EXPECT_NEAR 但失败后立即终止当前测试 | 慎用，一般 EXPECT 就够 |

**`EXPECT_*` vs `ASSERT_*`：**
- `EXPECT_*`：失败后**继续执行**该测试的后续断言（推荐）
- `ASSERT_*`：失败后**立即终止**该测试（只在"不满足前提条件就无法继续"时用）

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make --pkg filter
```

如果编译报错，检查：
- 你是否在 `#include` 之前写了断言语句
- 分号是否遗漏
- 引用的函数名是否和 `math_utils.h` 中的一致（注意 `imu_filter::invSqrt` 命名空间）

### 4. 运行测试

```bash
# 方式一：单独运行一个测试
./devel/lib/filter/test_invsqrt

# 方式二：运行所有 filter 包的测试
catkin_make run_tests --pkg filter

# 方式三：查看测试结果摘要
catkin_test_results build/filter
```

成功输出示例：
```
[==========] Running 4 tests from 1 test suite.
[----------] 4 tests from InvSqrtTest
[ RUN      ] InvSqrtTest.PositiveNumber_ReturnsCorrectApproximation
[       OK ] InvSqrtTest.PositiveNumber_ReturnsCorrectApproximation (0 ms)
...
[  PASSED  ] 4 tests.
```

### 5. 做 bias_calculator 测试

`test_bias_calculator.cpp` 稍微复杂一些，因为它用了 **Test Fixture**（`TEST_F`）。
Fixture 的好处是 `SetUp()` 在每个测试前自动运行，保证测试相互独立。

## 进阶挑战（做完基础练习后）

### 挑战 A：自己提取并测试一个函数

1. 打开 `Madgwick_filter.cpp`，找到 `qua2Euler` 函数
2. 我已经在 `math_utils.h` 中放了一个干净的版本：`imu_filter::quatToEuler()`
3. 在 `test/` 下新建 `test_quat_euler.cpp`
4. 写测试覆盖以下场景：
   - 单位四元数 (0,0,0,1) → 所有 Euler 角为 0
   - 绕 Z 轴旋转 90° 的四元数 (0,0,0.7071,0.7071) → yaw=π/2, roll=pitch=0
   - 绕 X 轴旋转 90° 的四元数 (0.7071,0,0,0.7071) → roll=π/2
5. 在 CMakeLists.txt 中添加 `catkin_add_gtest(test_quat_euler test/test_quat_euler.cpp)`

### 挑战 B：刻意制造 bug 验证测试

1. 在 `math_utils.h` 中把 `calculateBias` 的除法改成乘法：`bias_x = sum_x * count;`
2. 运行测试 → 观察哪些测试失败、失败信息长什么样
3. 把 bug 改回来 → 确认测试重新通过
4. 思考：这个 bug 在真实 ROS 节点中会导致什么问题？测试帮你发现了什么？

### 挑战 C：为 MadgwickAHRSupdateIMU 写测试

这是最难的练习。因为这个函数依赖全局变量（`q0, q1, q2, q3, sampleFreq, beta`），
你需要设计测试来覆盖它。

提示：
1. 先设置 `sampleFreq = 100.0`, `beta = 0.1`
2. 初始 q0=1, q1=0, q2=0, q3=0（单位四元数，表示无旋转）
3. 输入静止 IMU 数据：`gx=gy=gz=0, ax=0, ay=0, az=9.81`（只有重力加速度）
4. 验证 quaternion 保持单位四元数（因为无角速度，姿态不变）

## 常见问题

**Q: 为什么不用 `ASSERT_EQ(0.5, invSqrt(4.0))`？**
A: `ASSERT_EQ` 做精确比较。浮点数 `1/sqrt(4)` 可能算出来是 `0.49999...` 而不是精确的 `0.5`。
浮点比较永远用 `EXPECT_NEAR` 或 `EXPECT_DOUBLE_EQ`。

**Q: 我的测试编译通过了但运行时报 "Could not find gtest"？**
A: 检查是否 source 了工作区：`source devel/setup.bash`

**Q: `SetUp()` 和构造函数有什么区别？**
A: `SetUp()` 在**每个** `TEST_F` 之前运行，构造函数只在对象创建时运行一次。
gtest 为每个测试创建一个新的 fixture 对象，所以实际上两者效果相同。
习惯上用 `SetUp()` 来表达"这是测试准备步骤"的意图。

**Q: 我需要修改 `math_utils.h` 来修复 bug 吗？**
A: 当前 `calculateBias` 不处理 negative count。在练习 4d 中你会发现这一点。
决定是否修复、如何修复，是这个练习的一部分。

## 下一步

完成所有练习后 → 回到 [testing_roadmap.md](../../docs/testing_roadmap.md) 查看"学到什么程度才算过关"清单，
确认自己达标后进入项目 2：Python 单元测试。
