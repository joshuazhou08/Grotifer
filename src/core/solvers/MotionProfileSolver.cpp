#include "core/utils/RotationHelpers.hpp"
#include "core/solvers/MotionProfileSolver.hpp"
#include <iostream>
#include <cmath>
using namespace Eigen;

void MotionProfileSolver::initialize(const RotationCommand &command,
                                     double currentTime,
                                     double deltaTaskTime,
                                     const Matrix3d& startingOrientation)
{
    command_ = command;
    velocity_ = 0.0;
    angle_ = 0.0;
    axis_ = command.axis.normalized();

    double accelDur = std::abs(command.velocity / command.acceleration) + deltaTaskTime;
    double constDur = std::abs(command.angle / command.velocity) + deltaTaskTime;
    double deccelDur = accelDur;

    accelerationEnd_ = currentTime + accelDur;
    constantEnd_ = currentTime + accelDur + constDur;
    decelerationEnd_ = currentTime + accelDur + constDur + deccelDur;
    startingOrientation_ = startingOrientation;
    initialized_ = true;

    std::cout << "[MotionProfileSolver] Initialized motion profile" << std::endl;
}

std::pair<Vector3d, Matrix3d>
MotionProfileSolver::solve(double time, double deltaT)
{
    if (!initialized_)
    {
        std::cerr << "[MotionProfileSolver] Warning: solve() called before initialize()" << std::endl;
        return {Vector3d::Zero(), startingOrientation_};
    }

    // Update velocity & angle based on motion phase
    if (time < accelerationEnd_)
        velocity_ += command_.acceleration * deltaT;
    else if (time < constantEnd_)
        velocity_ = command_.velocity;
    else if (time < decelerationEnd_)
        velocity_ -= command_.acceleration * deltaT;
    else
        velocity_ = 0.0;

    angle_ += velocity_ * deltaT;

    // Compute inertial velocity and orientation
    Vector3d inertialVel = axis_ * velocity_;
    Matrix3d rotMat = RotationHelpers::calculateRotationMatrix(axis_ * angle_);
    Matrix3d targetOrientation = startingOrientation_ * rotMat;

    return {inertialVel, targetOrientation};
}
