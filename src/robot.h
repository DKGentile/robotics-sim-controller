#pragma once

struct WheelVelocities {
    double left{0.0};
    double right{0.0};
};

struct RobotState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double linear_velocity{0.0};
    double angular_velocity{0.0};
};

class DifferentialDriveRobot {
public:
    DifferentialDriveRobot(double wheel_base, double max_wheel_speed, double motor_time_constant);

    static WheelVelocities bodyToWheel(double linear_velocity, double angular_velocity, double wheel_base);
    WheelVelocities bodyToWheel(double linear_velocity, double angular_velocity) const;

    void update(double left_pwm, double right_pwm, double slip_ratio, double dt);

    const RobotState& state() const { return state_; }
    WheelVelocities wheel_velocities() const { return wheel_velocities_; }
    double wheel_base() const { return wheel_base_; }

private:
    double wheel_base_;
    double max_wheel_speed_;
    double motor_time_constant_;
    RobotState state_;
    WheelVelocities wheel_velocities_;
};