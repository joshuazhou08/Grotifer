#pragma once
#include "core/utils/TimeUtils.hpp"

// PID Controller with anti-windup for position/velocity control
// Implements proportional, integral, and derivative control with output limiting
class PIControl {
public:
    // Constructor - initialize controller with gains and limits
    PIControl(double kp, double ki, double hLim, double lLim, double kd = 0.0);
    ~PIControl() = default;

    // Get controller parameters
    double getKp() const { return kp_; }
    double getKi() const { return ki_; }
    double getKd() const { return kd_; }
    double getError() const { return error_; }
    double getSignal() const {return signal_; }

    // Main controller calculation
    // Returns control signal based on setpoint and actual value
    double calculate(double setpoint, double actualValue);

private:
    // Controller gains
    double kp_;  // Proportional gain
    double ki_;  // Integral gain
    double kd_;  // Derivative gain

    // Output limits
    double hLim_;  // Upper limit
    double lLim_;  // Lower limit

    // Controller state
    double error_ = 0.0;
    double prevError_ = 0.0;
    double integralPart_ = 0.0;
    double time_ = 0.0;
    double prevTime_ = 0.0;

    // Last signal
    double signal_ = 0.0;
};

