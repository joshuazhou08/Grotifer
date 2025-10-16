#include "core/control/PIControl.hpp"
#include <algorithm>

using namespace TimeUtils;
using namespace std;

// Constructor - initialize controller with gains and limits
PIControl::PIControl(double kp, double ki, double hLim, double lLim, double kd)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      hLim_(hLim),
      lLim_(lLim),
      error_(0.0),
      prevError_(0.0),
      integralPart_(0.0),
      time_(0.0),
      prevTime_(GetTimeNow())
{
}

// Main controller calculation
double PIControl::calculate(double setpoint, double actualValue) {
    // Update time and calculate delta
    time_ = GetTimeNow();
    double deltaT = time_ - prevTime_;
    prevTime_ = time_;

    // Calculate error
    error_ = setpoint - actualValue;

    // Calculate proportional part
    double proportionalPart = kp_ * error_;

    // Calculate integral part with anti-windup
    // Only integrate if output is not saturated or error would reduce saturation
    double tempOutput = proportionalPart + integralPart_;
    if ((tempOutput > hLim_ && error_ > 0.0) || (tempOutput < lLim_ && error_ < 0.0)) {
        // Output is saturated, don't integrate to prevent windup
        // Keep integralPart_ unchanged
    } else {
        // Update integral
        integralPart_ += ki_ * error_ * deltaT;
    }

    // Calculate derivative part (if kd is non-zero)
    double derivativePart = 0.0;
    if (kd_ > 0.0 && deltaT > 0.0) {
        derivativePart = kd_ * (error_ - prevError_) / deltaT;
    }
    prevError_ = error_;

    // Calculate control signal (PID)
    double controlSignal = proportionalPart + integralPart_ + derivativePart;

    // Clamp output to limits
    controlSignal = clamp(controlSignal, lLim_, hLim_);

    return controlSignal;
}
