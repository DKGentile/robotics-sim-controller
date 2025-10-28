#include "telemetry.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

TelemetryLogger::TelemetryLogger(const std::string& file_path, bool echo_stdout)
    : file_path_(file_path),
      echo_stdout_(echo_stdout),
      header_written_(false) {
    const std::filesystem::path path(file_path_);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    file_.open(file_path_, std::ios::out | std::ios::trunc);
    if (!file_) {
        throw std::runtime_error("Failed to open telemetry file: " + file_path_);
    }
    write_header();
}

void TelemetryLogger::write_header() {
    static const std::string header =
        "time,linear_setpoint,linear_measured,angular_setpoint,angular_measured,"
        "left_setpoint,right_setpoint,left_measured,right_measured,"
        "left_actual,right_actual,left_pwm,right_pwm,imu_yaw,yaw_true,x,y";
    file_ << header << '\n';
    if (echo_stdout_) {
        std::cout << header << '\n';
    }
    header_written_ = true;
}

void TelemetryLogger::write(const TelemetryRow& row) {
    if (!header_written_) {
        write_header();
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << row.time << ','
        << row.linear_setpoint << ','
        << row.linear_measured << ','
        << row.angular_setpoint << ','
        << row.angular_measured << ','
        << row.left_setpoint << ','
        << row.right_setpoint << ','
        << row.left_measured << ','
        << row.right_measured << ','
        << row.left_actual << ','
        << row.right_actual << ','
        << row.left_pwm << ','
        << row.right_pwm << ','
        << row.imu_yaw << ','
        << row.yaw_true << ','
        << row.x << ','
        << row.y;

    file_ << oss.str() << '\n';
    if (echo_stdout_) {
        std::cout << oss.str() << '\n';
    }
}