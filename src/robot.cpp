#include "robot.h"

#include <algorithm>
#include <cmath>

DifferentialDriveRobot::DifferentialDriveRobot(double wheel_base,
                                               double max_wheel_speed,
                                               double motor_time_constant)
    : wheel_base_(wheel_base),
      max_wheel_speed_(max_wheel_speed),
      motor_time_constant_(motor_time_constant),
      state_{},
      wheel_velocities_{} {}

WheelVelocities DifferentialDriveRobot::bodyToWheel(double linear_velocity,
                                                    double angular_velocity,
                                                    double wheel_base) {
    WheelVelocities velocities;
    velocities.left = linear_velocity - 0.5 * wheel_base * angular_velocity;
    velocities.right = linear_velocity + 0.5 * wheel_base * angular_velocity;
    return velocities;
}

WheelVelocities DifferentialDriveRobot::bodyToWheel(double linear_velocity,
                                                    double angular_velocity) const {
    return bodyToWheel(linear_velocity, angular_velocity, wheel_base_);
}

void DifferentialDriveRobot::update(double left_pwm, double right_pwm, double slip_ratio, double dt) {
    const double slip = std::clamp(slip_ratio, 0.0, 0.95);

    auto motor_response = [&](double pwm, double& velocity) {
        const double limited_pwm = std::clamp(pwm, -1.0, 1.0);
        const double desired = limited_pwm * max_wheel_speed_ * (1.0 - slip);
        const double tau = std::max(motor_time_constant_, 1e-6);
        double alpha = dt / tau;
        alpha = std::clamp(alpha, 0.0, 1.0);
        velocity += (desired - velocity) * alpha;
    };

    motor_response(left_pwm, wheel_velocities_.left);
    motor_response(right_pwm, wheel_velocities_.right);

    state_.linear_velocity = 0.5 * (wheel_velocities_.left + wheel_velocities_.right);
    state_.angular_velocity =
        (wheel_velocities_.right - wheel_velocities_.left) / wheel_base_;

    state_.yaw += state_.angular_velocity * dt;
    const double cos_yaw = std::cos(state_.yaw);
    const double sin_yaw = std::sin(state_.yaw);
    state_.x += state_.linear_velocity * cos_yaw * dt;
    state_.y += state_.linear_velocity * sin_yaw * dt;
}