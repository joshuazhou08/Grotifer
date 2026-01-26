#pragma once

#include <Eigen/Dense>
#include "core/utils/RotationHelpers.hpp"
#include "Config.hpp"

using namespace Eigen;

class MotionProfileSolver
{
public:
    MotionProfileSolver() = default;


    void initialize(const RotationCommand &command,
                    double currentTime,
                    double deltaTaskTime,
                    const Matrix3d& startingOrientation);


    std::pair<Vector3d, Matrix3d> solve(double time,
                                        double deltaT);
    
    inline bool initialized() const { return initialized_; }
    inline double accelEnd() const { return accelerationEnd_; }
    inline double constEnd() const { return constantEnd_; }
    inline double decelEnd() const { return decelerationEnd_; }
    inline const RotationCommand& command() const { return command_; }
    inline double angleSoFar() const { return angle_; }

    // Retrieves the current profile angular velocity vector
    inline Vector3d velocityNow() const { return velocity_ * axis_; }

    // Done when we’ve passed decel end and are effectively at target
    inline bool isDone(double time, double angleTolerance = 1e-6, double velTolerance = 1e-6) const { 
        return time - 1 >= decelerationEnd_;       // 1 second grace
    }

    inline void reset() { initialized_ = false; }  // let AttitudeControl decide when to reset

    /**
     * @brief Compute the timing segments of a trapezoidal profile.
     *
     * Computes the ending times of the acceleration, constant, and deceleration phases
     * required to reach a desired setpoint while respecting first- and second-derivative
     * constraints.
     *
     * @param setpoint     Desired target value (e.g., angle; velocity).
     * @param firstDeriv   Desired first derivative constant value (e.g., angular velocity, acceleration). 
     * @param secondDeriv  Desired second derivative constant value (e.g., angular acceleration, jerk).
     * @param time         Current time (starting point for profile).
     *
     * @return A tuple (rampUpTime, constantTime, rampDownTime) describing the time spent
     *         accelerating, moving at constant first derivative, and decelerating,
     *         respectively. The first derivative is assumed to ramp up from 0 to firstDeriv,
     *         hold, then ramp back down to 0, following a trapezoidal profile.
     */
    static inline std::tuple<double, double, double> computeProfileTimes(double setpoint, double firstDeriv, double secondDeriv, double time);

private:
    double velocity_ = 0.0;
    double angle_ = 0.0;
    Vector3d axis_ = Vector3d::Zero();
    RotationCommand command_;
    Matrix3d startingOrientation_ = Matrix3d::Zero();
    Matrix3d endingOrientation_ = Matrix3d::Zero();
    double endingAngle_ = 0.0;

    double accelerationEnd_ = 0.0;
    double constantEnd_ = 0.0;
    double decelerationEnd_ = 0.0;
    bool initialized_ = false;
};
