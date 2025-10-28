#pragma once

#include "robot.h"

#include <random>

struct SensorConfig {
    double encoder_noise_std{0.0};
    double imu_noise_std{0.0};
    double imu_drift_rate{0.0};
};

struct SensorMeasurements {
    double left_wheel_velocity{0.0};
    double right_wheel_velocity{0.0};
    double yaw{0.0};
};

class SensorSuite {
public:
    explicit SensorSuite(const SensorConfig& config);

    SensorMeasurements sample(const DifferentialDriveRobot& robot, double dt);

private:
    double gaussian_noise(double stddev);

    double encoder_noise_std_;
    double imu_noise_std_;
    double imu_drift_rate_;
    double yaw_bias_;
    std::mt19937 rng_;
};