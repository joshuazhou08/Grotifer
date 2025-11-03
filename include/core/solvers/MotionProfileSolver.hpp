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
