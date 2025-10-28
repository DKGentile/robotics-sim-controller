#pragma once

#include <fstream>
#include <string>

struct TelemetryRow {
    double time{0.0};
    double linear_setpoint{0.0};
    double linear_measured{0.0};
    double angular_setpoint{0.0};
    double angular_measured{0.0};
    double left_setpoint{0.0};
    double right_setpoint{0.0};
    double left_measured{0.0};
    double right_measured{0.0};
    double left_actual{0.0};
    double right_actual{0.0};
    double left_pwm{0.0};
    double right_pwm{0.0};
    double imu_yaw{0.0};
    double yaw_true{0.0};
    double x{0.0};
    double y{0.0};
};

class TelemetryLogger {
public:
    TelemetryLogger(const std::string& file_path, bool echo_stdout);

    void write(const TelemetryRow& row);

    const std::string& path() const { return file_path_; }

private:
    void write_header();

    std::ofstream file_;
    std::string file_path_;
    bool echo_stdout_;
    bool header_written_;
};