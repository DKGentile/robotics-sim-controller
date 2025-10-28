#include "sensors.h"

#include <chrono>

SensorSuite::SensorSuite(const SensorConfig& config)
    : encoder_noise_std_(config.encoder_noise_std),
      imu_noise_std_(config.imu_noise_std),
      imu_drift_rate_(config.imu_drift_rate),
      yaw_bias_(0.0),
      rng_(static_cast<std::mt19937::result_type>(
          std::chrono::high_resolution_clock::now().time_since_epoch().count())) {}

double SensorSuite::gaussian_noise(double stddev) {
    if (stddev <= 0.0) {
        return 0.0;
    }
    std::normal_distribution<double> distribution(0.0, stddev);
    return distribution(rng_);
}

SensorMeasurements SensorSuite::sample(const DifferentialDriveRobot& robot, double dt) {
    SensorMeasurements measurements;
    yaw_bias_ += imu_drift_rate_ * dt;

    const auto wheels = robot.wheel_velocities();
    measurements.left_wheel_velocity = wheels.left + gaussian_noise(encoder_noise_std_);
    measurements.right_wheel_velocity = wheels.right + gaussian_noise(encoder_noise_std_);
    measurements.yaw = robot.state().yaw + yaw_bias_ + gaussian_noise(imu_noise_std_);
    return measurements;
}