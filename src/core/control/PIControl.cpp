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

double PIControl::calculate(double setpoint, double actualValue)
{
    time_ = GetTimeNow();
    double deltaT = time_ - prevTime_;
    prevTime_ = time_;

    if (deltaT <= 0.0)
        deltaT = 0.0; // or a small epsilon / nominal dt

    error_ = setpoint - actualValue;

    double p = kp_ * error_;

    double d = 0.0;
    if (kd_ > 0.0)
    {
        d = kd_ * (error_ - prevError_) / deltaT;
    }

    // 1) form a trial output using the *current* integrator
    double u_trial = p + integralPart_ + d;

    // 2) saturate it
    double u_sat = std::clamp(u_trial, lLim_, hLim_);

    // 3) anti-windup: if saturated, force integrator to be consistent
    //    otherwise integrate normally
    if (u_trial != u_sat)
    {
        // If you're saturated, "solve for" I so that u_sat = p + I + d
        integralPart_ = u_sat - (p + d);
    }
    else
    {
        integralPart_ += ki_ * error_ * deltaT;
    }

    prevError_ = error_;

    // final output
    double u = p + integralPart_ + d;
    return std::clamp(u, lLim_, hLim_);
}