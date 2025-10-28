#include "gtest/gtest.h"
#include "robot.h"

TEST(KinematicsTest, BodyToWheelSplitsVelocities) {
    const double wheel_base = 0.32;
    const WheelVelocities wheels = DifferentialDriveRobot::bodyToWheel(1.0, 0.0, wheel_base);
    EXPECT_NEAR(wheels.left, 1.0, 1e-9);
    EXPECT_NEAR(wheels.right, 1.0, 1e-9);

    const WheelVelocities turn = DifferentialDriveRobot::bodyToWheel(0.0, 1.0, wheel_base);
    EXPECT_NEAR(turn.left, -0.5 * wheel_base, 1e-9);
    EXPECT_NEAR(turn.right, 0.5 * wheel_base, 1e-9);
}

TEST(KinematicsTest, DynamicsRespondToCommands) {
    DifferentialDriveRobot robot(0.32, 1.0, 0.05);
    for (int i = 0; i < 200; ++i) {
        robot.update(1.0, 1.0, 0.0, 0.01);
    }
    const WheelVelocities wheels = robot.wheel_velocities();
    EXPECT_NEAR(wheels.left, 1.0, 0.05);
    EXPECT_NEAR(wheels.right, 1.0, 0.05);
    EXPECT_GT(robot.state().x, 0.0);
    EXPECT_NEAR(robot.state().y, 0.0, 0.05);
}