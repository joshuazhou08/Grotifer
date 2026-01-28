#include "tasks/AttitudeControlTask.hpp"
#include "core/solvers/TriadSolver.hpp"
#include "core/solvers/AngularVelocitySolver.hpp"
#include "core/utils/LogHelpers.hpp"
#include "core/Filters.hpp"
#include "core/utils/RotationHelpers.hpp"
#include "core/utils/TimeUtils.hpp"
#include "core/control/PIControl.hpp"
#include "hardware/sensors/SunSensor.hpp"
#include "Config.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <cmath>
#include <future>
#include <chrono>
#include <thread>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace RotationHelpers;
using namespace TimeUtils;

// Forward declaration for new timeout velocity command to motor
AttitudeControl::AttitudeControl(ThreeAxisActuator &threeAxisActuator,
                                 Sensor &sunSensor,
                                 Sensor &inclinometer,
                                 ControlLoops &controlLoops)
    : BaseTask("AttitudeControl", 0),
      threeAxisActuator_(threeAxisActuator),
      sunSensor_(sunSensor),
      inclinometer_(inclinometer),
      xVelocityLoop(controlLoops.xVelocityLoop),
      yVelocityLoop(controlLoops.yVelocityLoop),
      zVelocityLoop(controlLoops.zVelocityLoop),
      xPositionLoop(controlLoops.xPositionLoop),
      yPositionLoop(controlLoops.yPositionLoop),
      zPositionLoop(controlLoops.zPositionLoop)

{

    cout << "[AttitudeControl] Logs initialized \n";

    deltaTaskTime = AttitudeConfig::deltaTaskTime;

    // Initialize rotation queue
    auto rotations = AttitudeConfig::getRotationQueue();
    for (const auto &rotation : rotations)
    {
        rotationQueue.push(rotation);
    }
    rotationQueueInitialized = true;

    cout << "[AttitudeControl] Moves planned \n";

    // Initialize Logging in Separate Thread
    orientationQueue_ = addQueue<OrientationRow, 256>("orientation.csv");
    profileOrientationQueue_ = addQueue<OrientationRow, 256>("profile_orientation.csv");

    angularVelocityQueue_ = addQueue<VectorRow, 256>("angular_velocity.csv");
    profileAngularVelocityQueue_ = addQueue<VectorRow, 256>("profile_angular_velocity.csv");

    profileQueue_ = addQueue<ProfileRow, 256>("profile.csv");

    positionControlLoopQueue_ = addQueue<ControlLoopRow, 256>("position_control_loop.csv");
    velocityControlLoopQueue_ = addQueue<ControlLoopRow, 256>("velocity_control_loop.csv");

}

AttitudeControl::~AttitudeControl()
{

    cout << "[AttitudeControl] Shutting down \n";
}

int AttitudeControl::Run()
{
    if (GetTimeNow() < nextTaskTime)
    {
        return 0;
    }
    // update
    timeStart = GetTimeNow();
    state = nextState;
    stateName = nextStateName;

    // Read the Sun Sensor
    SunSensor &ss = dynamic_cast<SunSensor &>(sunSensor_);
    double thzSun = Deg2Rad(ss.getAngleX());
    double thySun = Deg2Rad(ss.getAngleY());
    int sunInfo = ss.getAddInfo();
    string message = ss.getAddMessage(sunInfo);
    if (sunInfo != 0)
    {
        cout << "[AttitudeControl] Sun Message: " << message << "\n";
    }

    // Read the inclinometer
    double thxIncl = Deg2Rad(inclinometer_.getAngleY()); // Corresponds to Inclinometer Y
    double thzIncl = Deg2Rad(inclinometer_.getAngleX()); // Corresponds to Inclinometer X

    // Get the current orientation and the angular velocity
    Matrix3d currentOrientation = TriadSolver::solve(thxIncl, thzIncl, thySun, thzSun);

    Vector3d angularVelocityVec{{0.0, 0.0, 0.0}};

    if (prevOrientation.isZero()) // Initial state has not been set
    {
        prevOrientation = currentOrientation;
        preTime = GetTimeNow();
        return 0; // return to avoid division by zero
    }

    // Calculate time and deltaT for this iteration
    double time = GetTimeNow();
    double deltaT = time - preTime;

    // Calculate the angular velocity
    angularVelocityVec = currentOrientation.transpose() * AngularVelocitySolver::solve(prevOrientation, currentOrientation, deltaT); // convert angular velocity to body fixed frame
    angularVelocityVec = emaFilter3d(AttitudeConfig::fc, deltaT, angularVelocityVec, preAngularVelocityVec);
    preAngularVelocityVec = angularVelocityVec;
    prevOrientation = currentOrientation;

    switch (state)
    {
    case INITIALIZING:
    {
        if (AttitudeConfig::initialKick)
        {
            iniKickEndTime = GetTimeNow() + AttitudeConfig::iniKickDuration;
            nextState = INITIALIZING_MOTION;
            nextStateName = "Initializing Motion";
            cout << "[AttitudeControl] Finished initializing. Going into initial kick \n";
        }
        else
        {
            detumblingEndTime = GetTimeNow() + AttitudeConfig::detumblingMaxDuration;
            nextState = DETUMBLING;
            nextStateName = "Detumbling";
            cout << "[AttitudeControl] Finished initializing. Going into detumbling \n";
        }

        break;
    }

    case INITIALIZING_MOTION:
    {
        if (time < iniKickEndTime)
        {
            applyTorque(AttitudeConfig::iniTorqueVec, deltaT);
        }
        else
        {
            Vector3d zeroVector{{0.0, 0.0, 0.0}};
            applyTorque(zeroVector, deltaT);
        }

        // Check if initial kick is complete
        if (time >= iniKickEndTime * 2)
        {
            detumblingEndTime = GetTimeNow() + AttitudeConfig::detumblingMaxDuration;
            nextState = DETUMBLING;
            nextStateName = "Detumbling";
            cout << "[AttitudeControl] Initial Kick Done \n";
        }
        else
        {
            nextState = INITIALIZING_MOTION;
            nextStateName = "Initializing Motion";
        }

        break;
    }

    case DETUMBLING:
    {
        Vector3d torque;
        torque(0) = xVelocityLoop.calculate(0, angularVelocityVec(0));
        torque(1) = yVelocityLoop.calculate(0, angularVelocityVec(1));
        torque(2) = zVelocityLoop.calculate(0, angularVelocityVec(2));

        applyTorque(torque, deltaT);

        double maxComponent = angularVelocityVec.cwiseAbs().maxCoeff();

        // Check if detumbling is complete
        if (time >= detumblingEndTime && maxComponent < 4.5e-3)
        {
            // Prepend find sun rotation to the front of the queue if enabled
            if (AttitudeConfig::enableFindSun)
            {
                prependFindSunRotation(currentOrientation);
                findSunDone_ = false;
                cout << "[AttitudeControl] Detumbling Done - Find Sun rotation added to queue \n";
            }
            else
            {
                findSunDone_ = true;
                cout << "[AttitudeControl] Detumbling Done - Find Sun disabled, proceeding to arbitrary rotations \n";
            }

            // If find sun not done, transition to moving as find sun and moving use the same state, this is for readability
            if (!findSunDone_)
            {
                initializeMovingProfile(currentOrientation);
                nextState = MOVING;
                nextStateName = "Moving";
            }
            // Transition to MOVING if there are rotations in the queue and enabled
            else if (!rotationQueue.empty() && movesEnabled_)
            {
                initializeMovingProfile(currentOrientation);
                nextState = MOVING;
                nextStateName = "Moving";
            }
            else
            {
                // hold current position until enabled moving
                setHoldingPosition(currentOrientation);
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                if (rotationQueue.empty())
                {
                    cout << "[AttitudeControl] No rotations in queue, holding current position \n";
                    movesDone_ = true;
                }
            }
        }
        else
        {
            nextState = DETUMBLING;
            nextStateName = "Detumbling";
        }
        Vector3d refAngularVelocityVec{{0.0, 0.0, 0.0}}; // logging
        VectorRow row = LogHelpers::flattenWithTime(time, refAngularVelocityVec);
        profileAngularVelocityQueue_->push(row);
        break;
    }

    case HOLDING_POSITION:
    {
        Vector3d refAngularVelocityVec{{0.0, 0.0, 0.0}}; // we want it to be 0 when holding position

        Vector3d torque = cascadeControl(holdingPosition, currentOrientation, refAngularVelocityVec, angularVelocityVec);

        applyTorque(torque, deltaT);

        // Transition to MOVING if there are rotations in the queue and enabled
        if (!rotationQueue.empty() && movesEnabled_)
        {   
            // use the holding position as the starting orientation for a more predictable profile
            initializeMovingProfile(holdingPosition);
            nextState = MOVING;
            nextStateName = "Moving";
        }

        // Stay in HOLDING_POSITION
        else
        {
            nextState = HOLDING_POSITION;
            nextStateName = "Holding Position";

            if (movesEnabled_)
            {
                movesDone_ = true;
            }
        }

        // logging
        VectorRow angularVelocityRow = LogHelpers::flattenWithTime(time, refAngularVelocityVec);
        OrientationRow orientationRow = LogHelpers::flattenWithTime(time, holdingPosition);

        profileAngularVelocityQueue_->push(angularVelocityRow);
        profileOrientationQueue_->push(orientationRow);
        break;
    }

    case MOVING:
    {
        // Calculate motion profile velocity and target orientation
        auto [inertialVelocityVec, movingProfileOrientation] = motionSolver_.solve(time, deltaT);

        // Convert inertial velocity to body frame
        Vector3d refAngularVelocityVec = currentOrientation.transpose() * inertialVelocityVec;

        // Log
        OrientationRow profileOrientationRow = LogHelpers::flattenWithTime(time, movingProfileOrientation);
        VectorRow angularVelocityRow = LogHelpers::flattenWithTime(time, refAngularVelocityVec);
        ProfileRow profileRow{time, motionSolver_.angleSoFar(), motionSolver_.velocityNow().norm()};

        profileOrientationQueue_->push(profileOrientationRow);
        profileAngularVelocityQueue_->push(angularVelocityRow);
        profileQueue_->push(profileRow);

        Vector3d torque = cascadeControl(movingProfileOrientation, currentOrientation, refAngularVelocityVec, angularVelocityVec);

        Vector3d error{{xVelocityLoop.getError(), yVelocityLoop.getError(), zVelocityLoop.getError()}};

        applyTorque(torque, deltaT);

        // Check if current move is complete
        double maxErrorComponent = error.cwiseAbs().maxCoeff();
        double maxVelComponent = angularVelocityVec.cwiseAbs().maxCoeff();
        double maxComponent = max(maxErrorComponent, maxVelComponent / 4);
        if (motionSolver_.isDone(time) && maxComponent <= 8.7e-3) // 8.7e-3 is 0.5 degreees
        {
            // Check if there are more moves in the queue
            if (!rotationQueue.empty() && movesEnabled_)
            {
                cout << "[AttitudeControl] Current move done! Preparing next move from queue \n";
                motionSolver_.reset();
                initializeMovingProfile(currentOrientation);
                nextState = MOVING;
                nextStateName = "Moving";
            }
            else
            {
                findSunDone_ = true; // technically only needs to be set once but this works as well
                setHoldingPosition(movingProfileOrientation);
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                cout << "[AttitudeControl] Holding position \n";
            }
        }
        else
        {
            // Continue moving
            nextState = MOVING;
            nextStateName = "Moving";
        }

        break;
    }
    }

    // Log actual state
    OrientationRow orientationRow = LogHelpers::flattenWithTime(time, currentOrientation);
    VectorRow angularVelocityRow = LogHelpers::flattenWithTime(time, angularVelocityVec);

    orientationQueue_->push(orientationRow);
    angularVelocityQueue_->push(angularVelocityRow);

    // Log
    ControlLoopRow positionControlRow = {time, xPositionLoop.getSignal(), yPositionLoop.getSignal(), zPositionLoop.getSignal(), 
        xPositionLoop.getError(), yPositionLoop.getError(), zPositionLoop.getError()};

    ControlLoopRow velocityControlRow = {time, xVelocityLoop.getSignal(), yVelocityLoop.getSignal(), zVelocityLoop.getSignal(), 
        xVelocityLoop.getError(), yVelocityLoop.getError(), zVelocityLoop.getError()};

    positionControlLoopQueue_->push(positionControlRow);
    velocityControlLoopQueue_->push(velocityControlRow);

    // Update preTime for next iteration
    preTime = time;
    timeEnd = GetTimeNow();
    nextTaskTime = timeEnd + deltaTaskTime;

    return 0;
}

void AttitudeControl::applyTorque(const Vector3d &torque, double deltaT)
{
    // Apply sign correction
    Vector3d correctedTorque = torque;
    correctedTorque(1) = torque(1);
    correctedTorque(2) = -torque(2);

    threeAxisActuator_.applyTorque(correctedTorque, deltaT);
}

void AttitudeControl::setHoldingPosition(const Matrix3d &orientation)
{
    holdingPosition = orientation;
    holdingPositionSet = true;
    cout << "[AttitudeControl] Holding position: \n"
         << holdingPosition << "\n";
}

void AttitudeControl::prependFindSunRotation(const Matrix3d &currentOrientation)
{
    // Calculate rotation needed to go from current orientation to identity (sun-pointing)
    Vector3d rotVec = calculateRotationVector(currentOrientation, Matrix3d::Identity());
    double rotAngle = rotVec.norm();

    // Create find sun rotation command using default velocity/acceleration
    RotationCommand findSunCommand(rotVec.normalized(), rotAngle);

    // Create a temporary queue to prepend the find sun command
    queue<RotationCommand> tempQueue;
    tempQueue.push(findSunCommand);

    // Move all existing commands to temp queue
    while (!rotationQueue.empty())
    {
        tempQueue.push(rotationQueue.front());
        rotationQueue.pop();
    }

    // Replace the original queue with temp queue
    rotationQueue = move(tempQueue);

    cout << "[AttitudeControl] Find Sun rotation prepended to queue: \n";
    cout << "[AttitudeControl]   Axis: " << rotVec.normalized().transpose() << "\n";
    cout << "[AttitudeControl]   Angle (rad): " << rotAngle << "\n";
    cout << "[AttitudeControl]   Angle (deg): " << Rad2Deg(rotAngle) << "\n";
    cout << "[AttitudeControl]   Total commands in queue: " << rotationQueue.size() << "\n";
}

Vector3d AttitudeControl::cascadeControl(const Matrix3d &targetOrientation, const Matrix3d &currentOrientation, const Vector3d &targetAngularVelocityVec, const Vector3d &currentAngularVelocityVec)
{
    // Calculate the position error
    Vector3d positionError = calculateRotationVector(targetOrientation, currentOrientation);
    Vector3d angularVelocity = targetAngularVelocityVec;

    // Outer loop output
    Vector3d omegaRefBody;
    omegaRefBody(0) = xPositionLoop.calculate(0, positionError(0));
    omegaRefBody(1) = yPositionLoop.calculate(0, positionError(1));
    omegaRefBody(2) = zPositionLoop.calculate(0, positionError(2));
    Vector3d omegaCmdBody = omegaRefBody + angularVelocity;

    // Feed into inner velocity loop
    Vector3d torque;
    torque(0) = xVelocityLoop.calculate(omegaCmdBody(0), currentAngularVelocityVec(0));
    torque(1) = yVelocityLoop.calculate(omegaCmdBody(1), currentAngularVelocityVec(1));
    torque(2) = zVelocityLoop.calculate(omegaCmdBody(2), currentAngularVelocityVec(2));

    return torque;
};

void AttitudeControl::initializeMovingProfile(const Matrix3d &startingOrientation)
{
    // Capture current orientation

    // Ensure there’s a command available
    if (rotationQueue.empty())
    {
        std::cout << "[AttitudeControl] No rotation configured - queue is empty \n";
        return;
    }

    // Pop the next command
    currentRotationCommand = rotationQueue.front();
    rotationQueue.pop();

    // Normalize the axis
    currentRotationCommand.axis.normalize();

    // Log details
    std::cout << "[AttitudeControl] Executing Rotation Command: \n";
    std::cout << "[AttitudeControl]   Axis: " << currentRotationCommand.axis.transpose() << "\n";
    std::cout << "[AttitudeControl]   Angle (rad): " << currentRotationCommand.angle << "\n";
    std::cout << "[AttitudeControl]   Angle (deg): " << Rad2Deg(currentRotationCommand.angle) << "\n";
    std::cout << "[AttitudeControl]   Velocity: " << currentRotationCommand.velocity << "\n";
    std::cout << "[AttitudeControl]   Acceleration: " << currentRotationCommand.acceleration << "\n";
    std::cout << "[AttitudeControl]   Remaining moves in queue: " << rotationQueue.size() << "\n";

    // Initialize motion profile solver
    double timeNow = GetTimeNow();
    motionSolver_.initialize(currentRotationCommand, timeNow, deltaTaskTime, startingOrientation);

    std::cout << "[AttitudeControl] MotionProfileSolver initialized \n";
    std::cout << "[AttitudeControl]   Acceleration End Time: " << motionSolver_.accelEnd() << "\n";
    std::cout << "[AttitudeControl]   Constant End Time: " << motionSolver_.constEnd() << "\n";
    std::cout << "[AttitudeControl]   Deceleration End Time: " << motionSolver_.decelEnd() << "\n";
}
