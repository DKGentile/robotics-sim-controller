#pragma once

#include <algorithm>
#include <cmath>

class PidController {
public:
    struct Gains {
        double kp{0.0};
        double ki{0.0};
        double kd{0.0};
        double output_min{-1.0};
        double output_max{1.0};
    };

    explicit PidController(const Gains& gains = Gains())
        : gains_(gains), integrator_(0.0), prev_error_(0.0), first_update_(true) {}

    void set_gains(const Gains& gains) {
        gains_ = gains;
        integrator_ = std::clamp(integrator_, gains_.output_min, gains_.output_max);
    }

    void reset() {
        integrator_ = 0.0;
        prev_error_ = 0.0;
        first_update_ = true;
    }

    double update(double setpoint, double measurement, double dt) {
        const double clamped_dt = dt > 1e-6 ? dt : 1e-6;
        const double error = setpoint - measurement;
        const double derivative = first_update_ ? 0.0 : (error - prev_error_) / clamped_dt;

        double proposed_integrator = integrator_ + gains_.ki * error * clamped_dt;
        proposed_integrator = std::clamp(proposed_integrator, gains_.output_min, gains_.output_max);

        const double provisional_output =
            gains_.kp * error + proposed_integrator + gains_.kd * derivative;
        const bool saturating_high = provisional_output > gains_.output_max;
        const bool saturating_low = provisional_output < gains_.output_min;

        if ((saturating_high && error > 0.0) || (saturating_low && error < 0.0)) {
            // Skip integrator update in the direction that would worsen saturation.
        } else {
            integrator_ = proposed_integrator;
        }

        integrator_ = std::clamp(integrator_, gains_.output_min, gains_.output_max);

        double output = gains_.kp * error + integrator_ + gains_.kd * derivative;
        output = std::clamp(output, gains_.output_min, gains_.output_max);

        prev_error_ = error;
        first_update_ = false;
        return output;
    }

    const Gains& gains() const { return gains_; }

private:
    Gains gains_;
    double integrator_;
    double prev_error_;
    bool first_update_;
};