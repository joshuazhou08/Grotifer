#include "tasks/AttitudeControlTask.hpp"
#include "core/solvers/TriadSolver.hpp"
#include "core/solvers/AngularVelocitySolver.hpp"
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

using namespace std;
using namespace Eigen;
using namespace RotationHelpers;
using namespace TimeUtils;
// Forward delcaration for helpers
Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector);
pair<Vector3d, double> calculateRotationAxisAndAngle(const Matrix3d &fromMatrix, const Matrix3d &toMatrix);
// Forward declaration for new timeout velocity command to motor
AttitudeControl::AttitudeControl(ThreeAxisActuator& threeAxisActuator,
                                 Sensor& sunSensor,
                                 Sensor& inclinometer,
                                 ControlLoops& controlLoops)
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
    InitializeLogs();

    if (!attitudeLog || !sensorLog || !angularVelLog || !momentumWheelsLog)
    {
        throw runtime_error("Failed to open one or more log files in AttitudeControl.");
    }

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
}

AttitudeControl::~AttitudeControl()
{
    if (attitudeLog.is_open())
        attitudeLog.close();
    if (sensorLog.is_open())
        sensorLog.close();
    if (angularVelLog.is_open())
        angularVelLog.close();
    if (momentumWheelsLog.is_open())
        momentumWheelsLog.close();
    
    cout << "[AttitudeControl] Shutting down" << endl;
}

void AttitudeControl::InitializeLogs()
{
    filesystem::create_directories("logs");

    attitudeLog.open("logs/attitude.txt", ios::out | ios::trunc);
    sensorLog.open("logs/sensors.txt", ios::out | ios::trunc);
    angularVelLog.open("logs/angular_velocity.txt", ios::out | ios::trunc);
    momentumWheelsLog.open("logs/momentum_wheels.txt", ios::out | ios::trunc);

    attitudeLog << left << setw(w) << "Time (s)" << left << setw(w) << "Xb_x" << left << setw(w) << "Xb_y" << left << setw(w) << "Xb_z"
                << left << setw(w) << "Yb_x" << left << setw(w) << "Yb_y" << left << setw(w) << "Yb_z"
                << left << setw(w) << "Zb_x" << left << setw(w) << "Zb_y" << left << setw(w) << "Zb_z" << endl;

    sensorLog << left << setw(w) << "Time (s)" << left << setw(w) << "thxIncl(deg)" << left << setw(w) << "thzIncl(deg)"
              << left << setw(w) << "thySun(deg)" << left << setw(w) << "thzSun(deg)"
              << left << setw(w) << "thxEuler(deg)" << left << setw(w) << "thyEuler(deg)" << left << setw(w) << "thzEuler(deg)" << endl;

    angularVelLog << left << setw(w) << "Time(s)" << left << setw(w) << "AngVel-x(rad/s)" << left << setw(w) << "AngVel-y(rad/s)" << left << setw(w) << "AngVel-z(rad/s)"
                  << left << setw(w) << "Ref.AngVelX(rad/s)" << left << setw(w) << "Ref.AngVelY(rad/s)" << left << setw(w) << "Ref.AngVelZ(rad/s)"
                  << left << setw(w) << "Err.AngVelX(rad/s)" << left << setw(w) << "Err.AngVelY(rad/s)" << left << setw(w) << "Err.AngVelZ(rad/s)"
                  << left << setw(w) << "Rot.Angle(deg)" << left << setw(w) << "Prof.Angle(deg)" << left << setw(w) << "Prof.w(rad/s)" << left << setw(w) << "Prof.e(rad/s^2)" << endl;

    momentumWheelsLog << left << setw(w) << "Time(s)" << left << setw(w) << "TorqueX(Nm)" << left << setw(w) << "XWheelVelCmd(rpm)" << left << setw(w) << "XWheelVelAct(rpm)"
                      << left << setw(w) << "TorqueY(Nm)" << left << setw(w) << "YWheelVelCmd(rpm)" << left << setw(w) << "YWheelVelAct(rpm)"
                      << left << setw(w) << "TorqueZ(Nm)" << left << setw(w) << "ZWheelVelCmd(rpm)" << left << setw(w) << "ZWheelVelAct(rpm)" << endl;
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

    switch (state)
    {
    case INITIALIZING:
    {
        if (AttitudeConfig::initialKick)
        {
            iniMotionDone = false;
            iniKickEndTime = GetTimeNow() + AttitudeConfig::iniKickDuration;
            preTimeInitializingMotion = GetTimeNow();
            nextState = INITIALIZING_MOTION;
            nextStateName = "Initializing Motion";
            cout << "[AttitudeControl] Finished initializing. Going into initial kick" << endl;
        }
        else
        {
            iniMotionDone = true;
            detumblingEndTime = GetTimeNow() + AttitudeConfig::detumblingMaxDuration;
            preTimeDetumbling = GetTimeNow();
            nextState = DETERMINING_ATTITUDE;
            nextStateName = "Determining Attitude";
            cout << "[AttitudeControl] Finished initializing. Going into detumble" << endl;
        }

        break;
    }

    case DETERMINING_ATTITUDE:
    {
        // Read the Sun Sensor
        double thzSun = Deg2Rad(sunSensor_.getAngleX()); // Get Z-angle from Sun Sensor (corresponds to Sun X)
        double thySun = Deg2Rad(sunSensor_.getAngleY()); // Get Y-angle from Sun Sensor (corresponds to Sun Y)
        
        // Get sun sensor diagnostic info (SunSensor-specific)
        SunSensor& sunSensor = static_cast<SunSensor&>(sunSensor_);
        int sunInfo = sunSensor.getAddInfo();
        if (sunInfo != 0)
        {
            cout << "[AttitudeControl] Sun Message: " << sunSensor.getAddMessage(sunInfo) << endl;
        }

        // Read the Inclinometer
        double thxIncl = Deg2Rad(inclinometer_.getAngleY()); // Corresponds to Inclinometer Y
        double thzIncl = Deg2Rad(inclinometer_.getAngleX()); // Corresponds to Inclinometer X

        // Calculating the attitude
        currentOrientation = TriadSolver::solve(thxIncl, thzIncl, thySun, thzSun);

        // Get the axes
        Vector3d bodyX, bodyY, bodyZ;
        bodyX = currentOrientation * Vector3d(1.0, 0.0, 0.0);
        bodyY = currentOrientation * Vector3d(0.0, 1.0, 0.0);
        bodyZ = currentOrientation * Vector3d(0.0, 0.0, 1.0);

        double time = GetTimeNow(); // Get the current time

        // Calculate the angular velocity
        if (iniRotMat.isZero()) // Initial state has not been set
        {
            iniRotMat = currentOrientation;
            prevRotMat = iniRotMat;
            angularVelocityVec << 0.0, 0.0, 0.0;
            preTime = time;
        }
        else
        {
            double deltaT = time - preTime;                                                                                             // Calculate the actual delta_t between two consecutive calls
            angularVelocityVec = currentOrientation.transpose() * AngularVelocitySolver::solve(prevRotMat, currentOrientation, deltaT); // convert angular velocity to body fixed frame
            prevRotMat = currentOrientation;
        }

        angularVelocityVec = emaFilter3d(AttitudeConfig::fc, time - preTime, angularVelocityVec, preAngularVelocityVec); // Use Exponential-moving-average filter

        // TODO: Add logging

        // Update
        preAngularVelocityVec = angularVelocityVec;
        preTime = time;
        if (!iniMotionDone)
        {
            nextState = INITIALIZING_MOTION;
            nextStateName = "Initializing Motion";
            preTimeInitializingMotion = time;
        }
        else if (!detumblingDone)
        {
            nextState = DETUMBLING;
            nextStateName = "Detumbling";
            preTimeDetumbling = time;
        }

        else if (!movingDone)
        {
            nextState = MOVING;
            nextStateName = "Moving";
            preTimeMoving = time;
            if (!movingProfileCalculated)
            {
                // reset moving profile variables
                movingProfileVelocity = 0.0;
                movingProfileAngle = 0.0;
                movingProfileRotAxis << 0.0, 0.0, 0.0;
                startingOrientation = currentOrientation;

                // Get the next rotation from the queue (includes find sun if it was prepended)
                if (!rotationQueue.empty())
                {
                    currentRotationCommand = rotationQueue.front();
                    rotationQueue.pop();

                    deltaTheta = currentRotationCommand.angle;
                    movingProfileRotAxis = currentRotationCommand.axis.normalized();

                    cout << "[AttitudeControl] Executing Rotation Command:" << endl;
                    cout << "[AttitudeControl]   Axis: " << movingProfileRotAxis.transpose() << endl;
                    cout << "[AttitudeControl]   Angle (rad): " << deltaTheta << endl;
                    cout << "[AttitudeControl]   Angle (deg): " << Rad2Deg(deltaTheta) << endl;
                    cout << "[AttitudeControl]   Velocity: " << currentRotationCommand.velocity << endl;
                    cout << "[AttitudeControl]   Acceleration: " << currentRotationCommand.acceleration << endl;
                    cout << "[AttitudeControl]   Remaining moves in queue: " << rotationQueue.size() << endl;
                }
                else
                {
                    deltaTheta = 0; // No move configured
                    cout << "[AttitudeControl] No rotation configured - queue is empty" << endl;
                }
                double time = GetTimeNow();

                // Use parameters from current rotation command
                double moveVelocity = currentRotationCommand.velocity;
                double moveAcceleration = currentRotationCommand.acceleration;

                double accelDuration = abs(moveVelocity / moveAcceleration) + 2 * deltaTaskTime;
                double constantDuration = abs(deltaTheta / moveVelocity) + 2 * deltaTaskTime;
                double deccelDuration = accelDuration;

                movingProfileAccelerationEndTime = time + accelDuration;
                movingProfileConstantEndTime = time + accelDuration + constantDuration;
                movingProfileDecelerationEndTime = time + accelDuration + constantDuration + deccelDuration;
                movingProfileCalculated = true;

                cout << "[AttitudeControl] Moving Profile Calculated" << endl;
                cout << "[AttitudeControl]   Acceleration End Time: " << movingProfileAccelerationEndTime << endl;
                cout << "[AttitudeControl]   Constant End Time: " << movingProfileConstantEndTime << endl;
                cout << "[AttitudeControl]   Deceleration End Time: " << movingProfileDecelerationEndTime << endl;
            }
        }

        else
        {
            nextState = HOLDING_POSITION;
            nextStateName = "Holding Position";
        }
        break;
    }

    case INITIALIZING_MOTION:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeInitializingMotion;
        if (time < iniKickEndTime)
        {
            applyTorque(AttitudeConfig::iniTorqueVec, deltaT);
        }

        else
        {
            Vector3d zeroVector{{0.0, 0.0, 0.0}};
            applyTorque(zeroVector, deltaT);
        }
        if (time + preTimeInitializingMotion >= iniKickEndTime * 2)
        {
            iniMotionDone = true;
            cout << "[AttitudeControl] Initial Kick Done" << endl;
            detumblingEndTime = GetTimeNow() + AttitudeConfig::detumblingMaxDuration;
            preTimeDetumbling = GetTimeNow();
        }

        preTimeInitializingMotion = time;
        nextState = DETERMINING_ATTITUDE;
        nextStateName = "Determining Attitude";

        break;
    }

    case DETUMBLING:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeDetumbling;

        Vector3d torque;
        torque(0) = xVelocityLoop.calculate(0, angularVelocityVec(0));
        torque(1) = yVelocityLoop.calculate(0, angularVelocityVec(1));
        torque(2) = zVelocityLoop.calculate(0, angularVelocityVec(2));

        applyTorque(torque, deltaT);

        double maxComponent = angularVelocityVec.cwiseAbs().maxCoeff();

        if (time >= detumblingEndTime && maxComponent < 4.5e-3)
        {
            detumblingDone = true;

            // Prepend find sun rotation to the front of the queue if enabled
            if (AttitudeConfig::enableFindSun)
            {
                prependFindSunRotation();
                cout << "[AttitudeControl] Detumbling Done - Find Sun rotation added to queue" << endl;
            }
            else
            {
                cout << "[AttitudeControl] Detumbling Done - Find Sun disabled, proceeding to arbitrary rotations" << endl;
            }
        }

        nextState = DETERMINING_ATTITUDE;
        nextStateName = "Determining Attitude";

        preTimeDetumbling = time;
        break;
    }

    case HOLDING_POSITION:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeHoldingPos;

        Vector3d refAngularVelocityVec{{0.0, 0.0, 0.0}}; // we want it to be 0 when holding position

        Vector3d torque = cascadeControl(holdingPosition, currentOrientation, refAngularVelocityVec);

        applyTorque(torque, deltaT);

        nextState = DETERMINING_ATTITUDE;
        nextStateName = "Determining Attitude";
        preTimeHoldingPos = time;
        break;
    }

    case MOVING:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeMoving;

        if (time < movingProfileAccelerationEndTime)
        {
            movingProfileVelocity = movingProfileVelocity + currentRotationCommand.acceleration * deltaT;
            movingProfileAngle = movingProfileAngle + movingProfileVelocity * deltaT;
        }
        else if (time < movingProfileConstantEndTime)
        {
            movingProfileVelocity = currentRotationCommand.velocity;
            movingProfileAngle = movingProfileAngle + movingProfileVelocity * deltaT;
        }
        else if (time < movingProfileDecelerationEndTime)
        {
            movingProfileVelocity = movingProfileVelocity - currentRotationCommand.acceleration * deltaT;
            movingProfileAngle = movingProfileAngle + movingProfileVelocity * deltaT;
        }
        else
        {
            movingProfileVelocity = 0.0;
            movingProfileAngle = deltaTheta;
        }

        Vector3d refAngularVelocityVec = currentOrientation.transpose() * (movingProfileRotAxis * movingProfileVelocity);

        // Get the profile orientation by taking the starting orientation and rotating it by the profile angle
        AngleAxis aa(movingProfileAngle, movingProfileRotAxis);
        Matrix3d movingProfileOrientation = startingOrientation * aa.toRotationMatrix();

        Vector3d torque = cascadeControl(movingProfileOrientation, currentOrientation, refAngularVelocityVec);

        angularVelocityErrorVec << xVelocityLoop.GetError(), yVelocityLoop.GetError(), zVelocityLoop.GetError();

        applyTorque(torque, deltaT);

        double maxComponent = angularVelocityVec.cwiseAbs().maxCoeff();
        if (time > movingProfileDecelerationEndTime && maxComponent <= 4.5e-3)
        {
            movingDone = true;

            // Check if there are more moves in the queue
            if (!rotationQueue.empty())
            {
                cout << "[AttitudeControl] Current move done! Preparing next move from queue" << endl;
                movingProfileCalculated = false;
                movingDone = false; // Reset to continue with next move
            }
            else
            {
                setHoldingPosition(movingProfileOrientation);
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                cout << "[AttitudeControl] All moves completed! Holding final position" << endl;
                movingProfileCalculated = false;
            }
        }

        nextState = DETERMINING_ATTITUDE;
        nextStateName = "Determining Attitude";
        preTimeMoving = time;
        break;
    }
    }
    nextTaskTime += deltaTaskTime;
    timeEnd = GetTimeNow();

    AuditDataTrail();
    return 0;
}

void AttitudeControl::applyTorque(Vector3d torque, double deltaT)
{
    // Apply sign correction for Y and Z axes (hardware mounting orientation)
    Vector3d correctedTorque = torque;
    correctedTorque(1) = -torque(1);  // Y axis needs to be flipped
    correctedTorque(2) = -torque(2);  // Z axis needs to be flipped
    
    threeAxisActuator_.applyTorque(correctedTorque, deltaT);
}

void AttitudeControl::setHoldingPosition(Matrix3d position)
{
    holdingPosition = position;
    holdingPositionSet = true;
    preTimeHoldingPos = GetTimeNow();
    cout << "[AttitudeControl] Holding position: " << endl << holdingPosition << endl;
}

void AttitudeControl::prependFindSunRotation()
{
    // Calculate rotation needed to go from current orientation to identity (sun-pointing)
    auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(currentOrientation, Matrix3d::Identity());

    // Create find sun rotation command using default velocity/acceleration
    RotationCommand findSunCommand(rotAxis.normalized(), rotAngle, refVelocity, refAcceleration);

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
    cout << "[AttitudeControl]   Axis: " << rotAxis.normalized().transpose() << endl;
    cout << "[AttitudeControl]   Angle (rad): " << rotAngle << endl;
    cout << "[AttitudeControl]   Angle (deg): " << Rad2Deg(rotAngle) << endl;
    cout << "[AttitudeControl]   Total commands in queue: " << rotationQueue.size() << endl;
}

Vector3d AttitudeControl::cascadeControl(Matrix3d target, Matrix3d current, Vector3d refAngularVelocityVec)
{
    // Calculate the position error
    auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(target, current);
    rotAxis = rotAxis.normalized();
    Vector3d positionError = current.transpose() * (rotAxis * rotAngle);

    // Outer loop output
    Vector3d omegaRefBody;
    omegaRefBody(0) = xPositionLoop.calculate(0, positionError(0));
    omegaRefBody(1) = yPositionLoop.calculate(0, positionError(1));
    omegaRefBody(2) = zPositionLoop.calculate(0, positionError(2));
    Vector3d omegaCmdBody = omegaRefBody + refAngularVelocityVec;

    // Feed into inner velocity loop
    Vector3d torque;
    torque(0) = xVelocityLoop.calculate(omegaCmdBody(0), angularVelocityVec(0));
    torque(1) = yVelocityLoop.calculate(omegaCmdBody(1), angularVelocityVec(1));
    torque(2) = zVelocityLoop.calculate(omegaCmdBody(2), angularVelocityVec(2));

    return torque;
};

Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector)
{
    double a = exp(-2 * M_PI * fc * dt);
    return (1.0 - a) * curVector + a * prevVector;
}

// Helper function to calculate rotation axis and angle between two rotation matrices
pair<Vector3d, double> calculateRotationAxisAndAngle(const Matrix3d &fromMatrix, const Matrix3d &toMatrix)
{
    Matrix3d rotation = BodyToBody(fromMatrix, toMatrix);
    AngleAxisd angleAxis{rotation};

    Vector3d rotAxis = angleAxis.axis();
    double rotAngle = angleAxis.angle();

    return make_pair(rotAxis, rotAngle);
}

