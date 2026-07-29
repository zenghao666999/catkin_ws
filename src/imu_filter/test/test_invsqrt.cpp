#include <gtest/gtest.h>
#include "imu_filter/math_utils.h"

// =============================================================================
// 练习 1: invSqrt —— 快速平方根倒数
//
// 这是最简单的测试目标。invSqrt 是一个纯数学函数：
//   输入: float x
//   输出: float (1/sqrt(x) 的近似值，误差通常在 0.2% 以内)
//
// 你的任务: 在下面的 TODO 位置填写断言语句。
// 提示: 使用 EXPECT_NEAR(expected, actual, tolerance) 做浮点比较。
// =============================================================================

TEST(InvSqrtTest, PositiveNumber_ReturnsCorrectApproximation) {
  // invSqrt(4.0) ≈ 0.5（因为 1/sqrt(4) = 0.5）
  // TODO: 填写断言
  EXPECT_NEAR(invSqrt(4.0), 0.5, 0.0001);

}

TEST(InvSqrtTest, One_ReturnsOne) {
  // invSqrt(1.0) 应该非常接近 1.0
  // TODO: 填写断言
  EXPECT_NEAR(invSqrt(1.0), 1.0, 0.0001);
}

TEST(InvSqrtTest, LargeNumber_ReturnsSmallValue) {
  // invSqrt(10000.0) ≈ 0.01（因为 1/sqrt(10000) = 0.01）
  // TODO: 填写断言
  EXPECT_NEAR(invSqrt(10000.0), 0.01, 0.0001);
}

// =============================================================================
// 练习 2: 边界值测试
//
// 思考：invSqrt(0.0) 应该返回什么？
//   数学上 1/sqrt(0) = +inf
//   但 IEEE 754 中，浮点除零可能产生 inf 或触发异常
//   这个测试帮助你发现函数的实际行为
// =============================================================================

TEST(InvSqrtTest, Zero_BehaviorDocumented) {
  // TODO: 用 EXPECT_TRUE 或 EXPECT_FLOAT_EQ 记录 invSqrt(0.0) 的实际行为
  // 思考：这里应该用什么断言才合理？
  EXPECT_TRUE(std::isnan(invSqrt(0.0)));
}

// =============================================================================
// 练习 3: 负数的行为
//
// invSqrt 从 Madgwick 滤波器中来，调用前归一化的平方和总是正数。
// 但如果我们给它一个负数呢？这测试的是"防御性"——函数在意外输入下的表现。
// =============================================================================

TEST(InvSqrtTest, NegativeInput_ReturnsNaN) {
  // invSqrt(-1.0) 应该返回 NaN（因为负数的平方根不是实数）
  // TODO: 用 std::isnan() 验证结果
  EXPECT_TRUE(std::isnan(invSqrt(-1.0)));
}

// =============================================================================
// 提示: 编译和运行测试的命令
//
//   cd ~/catkin_ws
//   catkin_make run_tests --pkg filter
//
// 或者单独编译测试:
//   catkin_make --pkg filter
//   ./devel/lib/filter/test_invsqrt
// =============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}