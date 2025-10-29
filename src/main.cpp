#include "cli.h"
#include "pid.h"
#include "robot.h"
#include "sensors.h"
#include "telemetry.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

PidController::Gains default_gains() {
    PidController::Gains gains;
    gains.kp = 2.0;
    gains.ki = 0.5;
    gains.kd = 0.05;
    gains.output_min = -1.0;
    gains.output_max = 1.0;
    return gains;
}

std::optional<double> find_value(const std::string& block, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?)");
    std::smatch match;
    if (std::regex_search(block, match, pattern)) {
        return std::stod(match[1].str());
    }
    return std::nullopt;
}

PidController::Gains parse_gains_block(const std::string& block, const PidController::Gains& defaults) {
    PidController::Gains gains = defaults;
    if (const auto value = find_value(block, "kp")) {
        gains.kp = *value;
    }
    if (const auto value = find_value(block, "ki")) {
        gains.ki = *value;
    }
    if (const auto value = find_value(block, "kd")) {
        gains.kd = *value;
    }
    if (const auto value = find_value(block, "output_min")) {
        gains.output_min = *value;
    }
    if (const auto value = find_value(block, "output_max")) {
        gains.output_max = *value;
    }
    if (gains.output_min > gains.output_max) {
        std::swap(gains.output_min, gains.output_max);
    }
    return gains;
}

struct ControllerConfig {
    PidController::Gains left;
    PidController::Gains right;
};

ControllerConfig load_pid_config(const std::string& path) {
    ControllerConfig config{default_gains(), default_gains()};
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Warning: unable to open PID config '" << path << "', using defaults.\n";
        return config;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string contents = buffer.str();

    const std::regex block_regex("\"([A-Za-z0-9_]+)\"\\s*:\\s*\\{([^}]*)\\}");
    for (std::sregex_iterator it(contents.begin(), contents.end(), block_regex);
         it != std::sregex_iterator(); ++it) {
        const std::string name = (*it)[1].str();
        const std::string block = (*it)[2].str();
        if (name == "left") {
            config.left = parse_gains_block(block, config.left);
        } else if (name == "right") {
            config.right = parse_gains_block(block, config.right);
        }
    }
    return config;
}

double compute_profile(ProfileType type, double amplitude, double time, double period) {
    if (std::abs(amplitude) < 1e-9) {
        return 0.0;
    }
    const double magnitude = std::abs(amplitude);
    const double sign = amplitude >= 0.0 ? 1.0 : -1.0;

    switch (type) {
        case ProfileType::Step:
            return amplitude;
        case ProfileType::Triangle: {
            double phase = std::fmod(time, period);
            if (phase < 0.0) {
                phase += period;
            }
            const double half = period / 2.0;
            double value = 0.0;
            if (phase < half) {
                value = -magnitude + (2.0 * magnitude / half) * phase;
            } else {
                value = magnitude - (2.0 * magnitude / half) * (phase - half);
            }
            return value * sign;
        }
        case ProfileType::Square: {
            double phase = std::fmod(time, period);
            if (phase < 0.0) {
                phase += period;
            }
            const double half = period / 2.0;
            const double value = (phase < half) ? magnitude : -magnitude;
            return value * sign;
        }
    }
    return amplitude;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        SimulationConfig config = SimulationConfig::from_args(argc, argv);
        if (config.duration <= 0.0) {
            throw std::invalid_argument("Duration must be positive.");
        }

        config.slip = std::clamp(config.slip, 0.0, 0.95);
        config.encoder_noise = std::max(0.0, config.encoder_noise);
        config.yaw_drift = std::max(0.0, config.yaw_drift);

        const ControllerConfig gains = load_pid_config(config.gains_path);
        PidController left_pid(gains.left);
        PidController right_pid(gains.right);

        DifferentialDriveRobot robot(0.32, 1.2, 0.12);
        SensorConfig sensor_config{config.encoder_noise, config.encoder_noise, config.yaw_drift};
        SensorSuite sensors(sensor_config);

        TelemetryLogger logger(config.csv_path, !config.quiet);

        constexpr double dt = 0.01;  // 100 Hz
        const double period = std::max(2.0, config.duration / 4.0);
        const std::size_t total_steps = static_cast<std::size_t>(std::ceil(config.duration / dt));

        SensorMeasurements measurement = sensors.sample(robot, 0.0);
        double time = 0.0;

        for (std::size_t step = 0; step < total_steps; ++step) {
            const double linear_ref = compute_profile(config.profile, config.linear_amplitude, time, period);
            const double angular_ref = compute_profile(config.profile, config.angular_amplitude, time, period);

            const WheelVelocities wheel_ref =
                DifferentialDriveRobot::bodyToWheel(linear_ref, angular_ref, robot.wheel_base());

            const double left_pwm = left_pid.update(wheel_ref.left, measurement.left_wheel_velocity, dt);
            const double right_pwm = right_pid.update(wheel_ref.right, measurement.right_wheel_velocity, dt);

            robot.update(left_pwm, right_pwm, config.slip, dt);
            time += dt;

            SensorMeasurements next_measurement = sensors.sample(robot, dt);
            const WheelVelocities actual_wheels = robot.wheel_velocities();

            TelemetryRow row;
            row.time = time;
            row.linear_setpoint = linear_ref;
            row.linear_measured =
                0.5 * (next_measurement.left_wheel_velocity + next_measurement.right_wheel_velocity);
            row.angular_setpoint = angular_ref;
            row.angular_measured =
                (next_measurement.right_wheel_velocity - next_measurement.left_wheel_velocity) / robot.wheel_base();
            row.left_setpoint = wheel_ref.left;
            row.right_setpoint = wheel_ref.right;
            row.left_measured = next_measurement.left_wheel_velocity;
            row.right_measured = next_measurement.right_wheel_velocity;
            row.left_actual = actual_wheels.left;
            row.right_actual = actual_wheels.right;
            row.left_pwm = left_pwm;
            row.right_pwm = right_pwm;
            row.imu_yaw = next_measurement.yaw;
            row.yaw_true = robot.state().yaw;
            row.x = robot.state().x;
            row.y = robot.state().y;

            logger.write(row);
            measurement = next_measurement;
        }

        std::clog << "Telemetry written to " << logger.path() << '\n';
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Simulation error: " << ex.what() << '\n';
        return 1;
    }
}
