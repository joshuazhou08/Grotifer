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

    // compute profile times
    auto[rampUpDur, constDur, rampDownDur] =
        computeProfileDurations(command.angle,
                            command.velocity,
                            command.acceleration);

    accelerationEnd_ = currentTime + rampUpDur;
    constantEnd_     = currentTime + rampUpDur + constDur;
    decelerationEnd_ = currentTime + rampUpDur + constDur + rampDownDur;

    startingOrientation_ = startingOrientation;
    endingOrientation_ = startingOrientation * RotationHelpers::calculateRotationMatrix(axis_ * command.angle);
    endingAngle_ = command.angle;

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
    else {
        angle_ = endingAngle_;
        return { Vector3d::Zero(), endingOrientation_};
    }
    angle_ += velocity_ * deltaT;

    // Compute inertial velocity and orientation
    Vector3d inertialVel = axis_ * velocity_;
    Matrix3d rotMat = RotationHelpers::calculateRotationMatrix(axis_ * angle_);
    Matrix3d targetOrientation = startingOrientation_ * rotMat;

    return {inertialVel, targetOrientation};
}


// public helper for getting times 
std::tuple<double, double, double> MotionProfileSolver::computeProfileDurations(double setpoint, double firstDeriv, double secondDeriv)
{
    double rampUpDur = std::abs(firstDeriv / secondDeriv);
    double changeDuringRamp = std::abs(0.5 * rampUpDur * firstDeriv); // integrate triangle area of trapeizoidal profile during ramp-up

    double remainingChange = std::abs(setpoint) - 2 * changeDuringRamp; // multiply by 2 as change occurs during both ramp-up and ramp-down
    if (remainingChange < 0) {
        std::cout << "[MotionProfileSolver] Warning: setpoint too small for trapezoidal profile, returning triangle profile times." << std::endl;
        // first derivative has a triangle profile case
        rampUpDur = std::sqrt(std::abs(setpoint) / secondDeriv);
        remainingChange = 0.0;
    }

    double constDur = std::abs(remainingChange / firstDeriv);
    double decelDur = rampUpDur;

    return std::make_tuple(rampUpDur, constDur, decelDur);
}