#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

enum class ProfileType {
    Step,
    Triangle,
    Square
};

namespace detail {
inline std::string to_lower(std::string value) {
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

inline ProfileType parse_profile(const std::string& value) {
    const std::string lower = to_lower(value);
    if (lower == "step") {
        return ProfileType::Step;
    }
    if (lower == "triangle") {
        return ProfileType::Triangle;
    }
    if (lower == "square") {
        return ProfileType::Square;
    }
    throw std::invalid_argument("Unknown profile: " + value);
}
}  // namespace detail

struct SimulationConfig {
    double duration{20.0};
    double linear_amplitude{0.3};
    double angular_amplitude{0.2};
    double slip{0.05};
    double encoder_noise{0.01};
    double yaw_drift{0.002};
    ProfileType profile{ProfileType::Step};
    std::string gains_path{"config/gains.json"};
    std::string csv_path{"out/telemetry.csv"};
    bool quiet{false};

    static SimulationConfig from_args(int argc, char** argv) {
        SimulationConfig config;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto require_value = [&](const std::string& flag) -> std::string {
                if (i + 1 >= argc) {
                    throw std::invalid_argument(flag + " requires a value");
                }
                return argv[++i];
            };

            if (arg == "--duration") {
                config.duration = std::stod(require_value(arg));
            } else if (arg == "--v") {
                config.linear_amplitude = std::stod(require_value(arg));
            } else if (arg == "--w") {
                config.angular_amplitude = std::stod(require_value(arg));
            } else if (arg == "--slip") {
                config.slip = std::stod(require_value(arg));
            } else if (arg == "--noise") {
                config.encoder_noise = std::stod(require_value(arg));
            } else if (arg == "--drift") {
                config.yaw_drift = std::stod(require_value(arg));
            } else if (arg == "--profile") {
                config.profile = detail::parse_profile(require_value(arg));
            } else if (arg == "--config" || arg == "--gains") {
                config.gains_path = require_value(arg);
            } else if (arg == "--csv") {
                config.csv_path = require_value(arg);
            } else if (arg == "--quiet") {
                config.quiet = true;
            } else if (arg == "--help" || arg == "-h") {
                std::cout
                    << "Usage: robotics-sim [options]\n"
                    << "  --duration <seconds>           Simulation duration (default 20)\n"
                    << "  --v <m/s>                      Linear velocity amplitude (default 0.3)\n"
                    << "  --w <rad/s>                    Angular velocity amplitude (default 0.2)\n"
                    << "  --profile <step|triangle|square>\n"
                    << "  --slip <ratio>                 Slip coefficient (0..1)\n"
                    << "  --noise <stddev>               Encoder noise standard deviation (m/s)\n"
                    << "  --drift <rad/s>                IMU yaw drift rate\n"
                    << "  --config <file>                PID gains JSON path\n"
                    << "  --csv <file>                   Output CSV path (default out/telemetry.csv)\n"
                    << "  --quiet                        Disable telemetry echo to stdout\n";
                std::exit(EXIT_SUCCESS);
            } else {
                std::cerr << "Warning: unrecognized argument '" << arg << "' ignored.\n";
            }
        }
        return config;
    }
};