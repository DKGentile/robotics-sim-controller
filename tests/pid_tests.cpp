#include "gtest/gtest.h"
#include "pid.h"

TEST(PidControllerTest, ProportionalResponse) {
    PidController::Gains gains{1.0, 0.0, 0.0, -1.0, 1.0};
    PidController pid(gains);
    const double output = pid.update(1.0, 0.0, 0.01);
    EXPECT_DOUBLE_EQ(output, 1.0);
}

TEST(PidControllerTest, IntegratorClampsWithinLimits) {
    PidController::Gains gains{0.5, 1.0, 0.0, -0.5, 0.5};
    PidController pid(gains);

    double output = 0.0;
    for (int i = 0; i < 200; ++i) {
        output = pid.update(10.0, 0.0, 0.01);
        EXPECT_LE(output, gains.output_max + 1e-9);
        EXPECT_GE(output, gains.output_min - 1e-9);
    }
    EXPECT_NEAR(output, gains.output_max, 1e-3);
}

TEST(PidControllerTest, DerivativeZeroOnFirstUpdate) {
    PidController::Gains gains{0.0, 0.0, 1.0, -10.0, 10.0};
    PidController pid(gains);
    const double first = pid.update(1.0, 0.0, 0.01);
    const double second = pid.update(1.0, 0.5, 0.01);
    EXPECT_DOUBLE_EQ(first, 0.0);
    EXPECT_GT(second, 0.0);
}