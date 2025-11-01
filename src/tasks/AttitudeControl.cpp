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

    cout << "[AttitudeControl] Logs initialized" << endl;

    deltaTaskTime = AttitudeConfig::deltaTaskTime;

    // Initialize rotation queue
    auto rotations = AttitudeConfig::getRotationQueue();
    for (const auto &rotation : rotations)
    {
        rotationQueue.push(rotation);
    }
    rotationQueueInitialized = true;

    cout << "[AttitudeControl] Moves planned" << endl;

    // Initialize Logging in Separate Thread
    orientationQueue_ = addQueue<OrientationRow, 1024>("orientation.csv");
    profileOrientationQueue_ = addQueue<OrientationRow, 1024>("profile_orientation.csv");

    angularVelocityQueue_ = addQueue<VectorRow, 1024>("angular_velocity.csv");
    profileAngularVelocityQueue_ = addQueue<VectorRow, 1024>("profile_angular_velocity.csv");
}

AttitudeControl::~AttitudeControl()
{

    cout << "[AttitudeControl] Shutting down" << endl;
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
        cout << "[AttitudeControl] Sun Message: " << message << endl;
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
            cout << "[AttitudeControl] Finished initializing. Going into initial kick" << endl;
        }
        else
        {
            detumblingEndTime = GetTimeNow() + AttitudeConfig::detumblingMaxDuration;
            nextState = DETUMBLING;
            nextStateName = "Detumbling";
            cout << "[AttitudeControl] Finished initializing. Going into detumbling" << endl;
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
            cout << "[AttitudeControl] Initial Kick Done" << endl;
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
                cout << "[AttitudeControl] Detumbling Done - Find Sun rotation added to queue" << endl;
            }
            else
            {
                cout << "[AttitudeControl] Detumbling Done - Find Sun disabled, proceeding to arbitrary rotations" << endl;
            }

            // Transition to MOVING if there are rotations in the queue
            if (!rotationQueue.empty())
            {
                initializeMovingProfile(currentOrientation);
                nextState = MOVING;
                nextStateName = "Moving";
            }
            else
            {
                // No moves configured, just hold current position
                setHoldingPosition(currentOrientation);
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                cout << "[AttitudeControl] No rotations in queue, holding current position" << endl;
            }
        }
        else
        {
            nextState = DETUMBLING;
            nextStateName = "Detumbling";
        }

        break;
    }

    case HOLDING_POSITION:
    {
        Vector3d refAngularVelocityVec{{0.0, 0.0, 0.0}}; // we want it to be 0 when holding position

        Vector3d torque = cascadeControl(holdingPosition, currentOrientation, refAngularVelocityVec, angularVelocityVec);

        applyTorque(torque, deltaT);

        // Stay in HOLDING_POSITION
        nextState = HOLDING_POSITION;
        nextStateName = "Holding Position";
        break;
    }

    case MOVING:
    {
        // Calculate motion profile velocity and target orientation
        auto [inertialVelocityVec, movingProfileOrientation] = motionSolver_.solve(time, deltaT);

        // Convert inertial velocity to body frame
        Vector3d refAngularVelocityVec = currentOrientation.transpose() * inertialVelocityVec;


        OrientationRow orientationRow = LogHelpers::flattenWithTime(time, currentOrientation);
        VectorRow angularVelocityRow = LogHelpers::flattenWithTime(time, refAngularVelocityVec);

        profileOrientationQueue_->push(orientationRow);
        profileAngularVelocityQueue_->push(angularVelocityRow);

        Vector3d torque = cascadeControl(movingProfileOrientation, currentOrientation, refAngularVelocityVec, angularVelocityVec);

        Vector3d error{{xVelocityLoop.getError(), yVelocityLoop.getError(), zVelocityLoop.getError()}};

        applyTorque(torque, deltaT);

        // Check if current move is complete
        double maxComponent = error.cwiseAbs().maxCoeff();
        if (motionSolver_.isDone(time) && maxComponent <= 4.5e-3)
        {
            // Check if there are more moves in the queue
            if (!rotationQueue.empty())
            {
                cout << "[AttitudeControl] Current move done! Preparing next move from queue" << endl;
                motionSolver_.reset();
                initializeMovingProfile(currentOrientation);
                nextState = MOVING;
                nextStateName = "Moving";
            }
            else
            {
                // All moves complete, hold final position
                setHoldingPosition(movingProfileOrientation);
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                cout << "[AttitudeControl] All moves completed! Holding final position" << endl;
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

    // Update preTime for next iteration
    preTime = time;

    nextTaskTime += deltaTaskTime;
    timeEnd = GetTimeNow();

    return 0;
}

void AttitudeControl::applyTorque(const Vector3d &torque, double deltaT)
{
    // Apply sign correction for Y and Z axes (hardware mounting orientation)
    Vector3d correctedTorque = torque;
    correctedTorque(1) = -torque(1); // Y axis needs to be flipped
    correctedTorque(2) = -torque(2); // Z axis needs to be flipped

    threeAxisActuator_.applyTorque(correctedTorque, deltaT);
}

void AttitudeControl::setHoldingPosition(const Matrix3d &orientation)
{
    holdingPosition = orientation;
    holdingPositionSet = true;
    cout << "[AttitudeControl] Holding position: " << endl
         << holdingPosition << endl;
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

    cout << "[AttitudeControl] Find Sun rotation prepended to queue:" << endl;
    cout << "[AttitudeControl]   Axis: " << rotVec.normalized().transpose() << endl;
    cout << "[AttitudeControl]   Angle (rad): " << rotAngle << endl;
    cout << "[AttitudeControl]   Angle (deg): " << Rad2Deg(rotAngle) << endl;
    cout << "[AttitudeControl]   Total commands in queue: " << rotationQueue.size() << endl;
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

void AttitudeControl::initializeMovingProfile(const Matrix3d &currentOrientation)
{
    // Capture current orientation

    // Ensure there’s a command available
    if (rotationQueue.empty())
    {
        std::cout << "[AttitudeControl] No rotation configured - queue is empty" << std::endl;
        return;
    }

    // Pop the next command
    currentRotationCommand = rotationQueue.front();
    rotationQueue.pop();

    // Normalize the axis
    currentRotationCommand.axis.normalize();

    // Log details
    std::cout << "[AttitudeControl] Executing Rotation Command:" << std::endl;
    std::cout << "[AttitudeControl]   Axis: " << currentRotationCommand.axis.transpose() << std::endl;
    std::cout << "[AttitudeControl]   Angle (rad): " << currentRotationCommand.angle << std::endl;
    std::cout << "[AttitudeControl]   Angle (deg): " << Rad2Deg(currentRotationCommand.angle) << std::endl;
    std::cout << "[AttitudeControl]   Velocity: " << currentRotationCommand.velocity << std::endl;
    std::cout << "[AttitudeControl]   Acceleration: " << currentRotationCommand.acceleration << std::endl;
    std::cout << "[AttitudeControl]   Remaining moves in queue: " << rotationQueue.size() << std::endl;

    // Initialize motion profile solver
    double timeNow = GetTimeNow();
    motionSolver_.initialize(currentRotationCommand, timeNow, deltaTaskTime, currentOrientation);

    std::cout << "[AttitudeControl] MotionProfileSolver initialized" << std::endl;
    std::cout << "[AttitudeControl]   Acceleration End Time: " << motionSolver_.accelEnd() << std::endl;
    std::cout << "[AttitudeControl]   Constant End Time: " << motionSolver_.constEnd() << std::endl;
    std::cout << "[AttitudeControl]   Deceleration End Time: " << motionSolver_.decelEnd() << std::endl;
}
