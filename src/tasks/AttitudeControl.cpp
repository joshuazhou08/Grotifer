#include "tasks/AttitudeControlTask.hpp"
#include "core/Logger.hpp"
#include "solvers/TriadSolver.hpp"
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
// Forward delcaration for helpers
Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector);
std::pair<Vector3d, double> calculateRotationAxisAndAngle(const Matrix3d &fromMatrix, const Matrix3d &toMatrix);
// Forward declaration for new timeout velocity command to motor
AttitudeControl::AttitudeControl(std::unique_ptr<MaxonMotor> mmX,
                                 std::unique_ptr<MaxonMotor> mmY,
                                 std::unique_ptr<ModbusSunSensor> sunSensor,
                                 std::unique_ptr<LabJackInclinometer> inclinometer,
                                 std::unique_ptr<FanController> fanX,
                                 std::unique_ptr<FanController> fanZ)
    : BaseTask("AttitudeControl", 0),
      p_mmX(std::move(mmX)),
      p_mmY(std::move(mmY)),
      p_sunSensor(std::move(sunSensor)),
      p_inclinometer(std::move(inclinometer)),
      p_fanX(std::move(fanX)),
      p_fanZ(std::move(fanZ)),
      logger("Attitude Control"),
      xVelocityLoop(
          AttitudeConfig::xVelocityK_p,
          AttitudeConfig::xVelocityK_i,
          AttitudeConfig::xVelocityhLim,
          AttitudeConfig::xVelocitylLim,
          AttitudeConfig::xVelocityK_d),
      yVelocityLoop(
          AttitudeConfig::yVelocityK_p,
          AttitudeConfig::yVelocityK_i,
          AttitudeConfig::yVelocityhLim,
          AttitudeConfig::yVelocitylLim,
          AttitudeConfig::yVelocityK_d),
      zVelocityLoop(
          AttitudeConfig::zVelocityK_p,
          AttitudeConfig::zVelocityK_i,
          AttitudeConfig::zVelocityhLim,
          AttitudeConfig::zVelocitylLim,
          AttitudeConfig::zVelocityK_d),
      xPositionLoop(
          AttitudeConfig::xPositionK_p,
          AttitudeConfig::xPositionK_i,
          AttitudeConfig::xPositionhLim,
          AttitudeConfig::xPositionlLim,
          AttitudeConfig::xPositionK_d),
      yPositionLoop(
          AttitudeConfig::yPositionK_p,
          AttitudeConfig::yPositionK_i,
          AttitudeConfig::yPositionhLim,
          AttitudeConfig::yPositionlLim,
          AttitudeConfig::yPositionK_d),
      zPositionLoop(
          AttitudeConfig::zPositionK_p,
          AttitudeConfig::zPositionK_i,
          AttitudeConfig::zPositionhLim,
          AttitudeConfig::zPositionlLim,
          AttitudeConfig::zPositionK_d)

{
    InitializeLogs();

    if (!attitudeLog || !sensorLog || !angularVelLog || !momentumWheelsLog)
    {
        throw std::runtime_error("Failed to open one or more log files in AttitudeControl.");
    }

    logger.info("Logs Initialized");

    deltaTaskTime = AttitudeConfig::deltaTaskTime;
    maxAccCmdY = (*p_mmY).GetMaxTorque() / AttitudeConfig::momOfInertiaY;

    // Initialize rotation queue
    auto rotations = AttitudeConfig::getRotationQueue();
    for (const auto &rotation : rotations)
    {
        rotationQueue.push(rotation);
    }
    rotationQueueInitialized = true;

    logger.info("Moves planned");
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
    p_fanX->moveFan(2048);
    p_fanZ->moveFan(2048);
    logger.info("killed fans");
}

void AttitudeControl::InitializeLogs()
{
    std::filesystem::create_directories("logs");

    attitudeLog.open("logs/attitude.txt", std::ios::out | std::ios::trunc);
    sensorLog.open("logs/sensors.txt", std::ios::out | std::ios::trunc);
    angularVelLog.open("logs/angular_velocity.txt", std::ios::out | std::ios::trunc);
    momentumWheelsLog.open("logs/momentum_wheels.txt", std::ios::out | std::ios::trunc);

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
            logger.info("Finished Intializing. Going into initial kick.");
        }
        else
        {
            iniMotionDone = true;
            detumblingEndTime = GetTimeNow() + AttitudeConfig::detumblingMaxDuration;
            preTimeDetumbling = GetTimeNow();
            nextState = DETERMINING_ATTITUDE;
            nextStateName = "Determining Attitude";
            logger.info("Finished Intializing. Going into detumble.");
        }

        break;
    }

    case DETERMINING_ATTITUDE:
    {
        // Read the Sun Sensor
        double thzSun, thySun;
        int sunInfo;
        if (!(*p_sunSensor).GetAngles(thzSun, thySun, sunInfo))
        {
            logger.error("Fail to read Sun Sensor!");
        }

        thySun = Deg2Rad(thySun); // Get Y-angle from Sun Sensor
        thzSun = Deg2Rad(thzSun); // Get Z-angle from Sun Sensor

        if (sunInfo != 0)
        {
            logger.info("Sun Message: ", (*p_sunSensor).GetAddMessage(sunInfo));
        }

        // Read the Inclinometer
        double thxIncl, thzIncl;

        thxIncl = Deg2Rad((*p_inclinometer).GetAngleY());
        thzIncl = Deg2Rad((*p_inclinometer).GetAngleX());

        // Calculating the attitude
        currentOrientation = TriadSolver::solve(thxIncl, thzIncl, thySun, thzSun);
        logger.debug("Orientation Matrix Calculated");

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
            angularVelocityVec = currentOrientation.transpose() * AngularVel::GetAngularVelVec(prevRotMat, currentOrientation, deltaT); // convert angular velocity to body fixed frame
            rotAngle = AngularVel::RotAngleAboutRotAxis(currentOrientation);
            prevRotMat = currentOrientation;
        }

        angularVelocityVec = emaFilter3d(AttitudeConfig::fc, time - preTime, angularVelocityVec, preAngularVelocityVec); // Use Exponential-moving-average filter
        logger.debug("Angular Velocity Vector Calculated");

        // TODO: Add logging

        logger.debug("Attitude Data Logged");

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

                    logger.info("Executing Rotation Command:");
                    logger.info("  Axis: ", movingProfileRotAxis);
                    logger.info("  Angle (rad): ", deltaTheta);
                    logger.info("  Angle (deg): ", Rad2Deg(deltaTheta));
                    logger.info("  Velocity: ", currentRotationCommand.velocity);
                    logger.info("  Acceleration: ", currentRotationCommand.acceleration);
                    logger.info("  Remaining moves in queue: ", rotationQueue.size());
                }
                else
                {
                    deltaTheta = 0; // No move configured
                    logger.info("No rotation configured - queue is empty");
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

                logger.info("Moving Profile Calculated");
                logger.info("Moving Profile Acceleration End Time: ", movingProfileAccelerationEndTime);
                logger.info("Moving Profile Constant End Time: ", movingProfileConstantEndTime);
                logger.info("Moving Profile Deceleration End Time: ", movingProfileDecelerationEndTime);
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
            logger.info("Initial Kick Done");
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
        torque(0) = xVelocityLoop.PICalculation(0, angularVelocityVec(0));
        torque(1) = yVelocityLoop.PICalculation(0, angularVelocityVec(1));
        torque(2) = zVelocityLoop.PICalculation(0, angularVelocityVec(2));

        applyTorque(torque, deltaT);

        double maxComponent = angularVelocityVec.cwiseAbs().maxCoeff();

        if (time >= detumblingEndTime && maxComponent < 4.5e-3)
        {
            detumblingDone = true;

            // Prepend find sun rotation to the front of the queue if enabled
            if (AttitudeConfig::enableFindSun)
            {
                prependFindSunRotation();
                logger.info("Detumbling Done - Find Sun rotation added to queue");
            }
            else
            {
                logger.info("Detumbling Done - Find Sun disabled, proceeding to arbitrary rotations");
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
                logger.info("Current move done! Preparing next move from queue.");
                movingProfileCalculated = false;
                movingDone = false; // Reset to continue with next move
            }
            else
            {
                setHoldingPosition(movingProfileOrientation);
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                logger.info("All moves completed! Holding final position.");
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

bool AttitudeControl::moveXMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal)
{
    bool saturateXMomtWheelFlag = false;
    double accCmd = 0; // Command value for acceleration
    int velCmd = 0;

    double torqueCmd = torque;

    accCmd = torqueCmd / AttitudeConfig::momOfInertiaX;
    velCmd = xMomentumWheelVel + (accCmd * deltaT) * (30 / M_PI); // conversion factor

    if (accCmd >= maxAccCmdX)
        accCmd = maxAccCmdX;
    else if (accCmd <= -maxAccCmdX)
        accCmd = -maxAccCmdX;

    // Check for saturation
    if (velCmd > AttitudeConfig::maxVelX)
    {
        saturateXMomtWheelFlag = true;
        velCmd = AttitudeConfig::maxVelX;
    }
    else if (velCmd < -AttitudeConfig::maxVelX)
    {
        saturateXMomtWheelFlag = true;
        velCmd = -AttitudeConfig::maxVelX;
    }

    (*p_mmX).RunWithVelocity(velCmd);
    *torqueCmdVal = torqueCmd;
    *velCmdVal = velCmd;

    xMomentumWheelVel = velCmd;
    return saturateXMomtWheelFlag;
}

bool AttitudeControl::moveYMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal)
{
    bool saturateYMomtWheelFlag = false;
    double accCmd = 0; // Command value for acceleration
    int velCmd = 0;
    double torqueCmd = -torque; // y must be flipped
    accCmd = torqueCmd / AttitudeConfig::momOfInertiaY;
    velCmd = yMomentumWheelVel + (accCmd * deltaT) * (30 / M_PI); // conversion factor

    if (accCmd >= maxAccCmdY)
        accCmd = maxAccCmdY;
    else if (accCmd <= -maxAccCmdY)
        accCmd = -maxAccCmdY;

    // Check for saturation
    if (velCmd > AttitudeConfig::maxVelY)
    {
        saturateYMomtWheelFlag = true;
        velCmd = AttitudeConfig::maxVelY;
    }
    else if (velCmd < -AttitudeConfig::maxVelY)
    {
        saturateYMomtWheelFlag = true;
        velCmd = -AttitudeConfig::maxVelY;
    }
    (*p_mmY).RunWithVelocity(velCmd);
    *torqueCmdVal = torqueCmd;
    *velCmdVal = velCmd;

    yMomentumWheelVel = velCmd;
    return saturateYMomtWheelFlag;
}

bool AttitudeControl::moveXFanWithTorque(double torque)
{
    // Convert to fan target using helper function
    uint16_t target = torqueToFanTarget(torque);

    // Send command to fan controller with built-in rate limiting
    if (p_fanX && p_fanX->isOpen())
    {
        p_fanX->moveFan(target);
        return false; // Success
    }
    else
    {
        return true; // Fan controller not available or not open
    }
}

bool AttitudeControl::moveZFanWithTorque(double torque)
{

    // Convert to fan target using helper function
    uint16_t target = torqueToFanTarget(-1 * torque); // flipped

    // Send command to fan controller with built-in rate limiting
    if (p_fanZ && p_fanZ->isOpen())
    {
        p_fanZ->moveFan(target);
        return false; // Success
    }
    else
    {
        return true;
    }
}
void AttitudeControl::applyTorque(Vector3d torque, double deltaT)
{
    double torqueCmdY, velCmdY;

    logger.debug("Applying torque with hybrid system: ", torque);
    // Apply X torque using X fan
    if (moveXFanWithTorque(torque(0)))
        logger.warning("X Fan controller not available or not open");

    // Apply Y torque using Y momentum wheel
    if (moveYMomentumWheelWithTorque(torque(1), deltaT, &torqueCmdY, &velCmdY))
        logger.warning("Saturate Y Momentum Wheel");

    // Apply Z torque using Z fan
    if (moveZFanWithTorque(torque(2)))
        logger.warning("Z Fan controller not available or not open");

    logger.debug("Successfully applied hybrid torque");

    // Get the Y momentum wheel data (fans handle their own logging)
    yMomentumWheelVel = (*p_mmY).GetVelocityIs();

    // Log the momentum wheel data (Y axis only, fans log separately)
    momentumWheelsLog << left << setw(w) << GetTimeNow()
                      << left << setw(w) << "N/A" << left << setw(w) << "N/A" << left << setw(w) << "N/A"                    // X fan data
                      << left << setw(w) << torqueCmdY << left << setw(w) << velCmdY << left << setw(w) << yMomentumWheelVel // Y wheel data
                      << left << setw(w) << "N/A" << left << setw(w) << "N/A" << left << setw(w) << "N/A" << endl;           // Z fan data

    logger.debug("Hybrid actuator data logged");
}

void AttitudeControl::setHoldingPosition(Matrix3d position)
{
    holdingPosition = position;
    holdingPositionSet = true;
    preTimeHoldingPos = GetTimeNow();
    logger.info("holding position:", holdingPosition);
}

void AttitudeControl::prependFindSunRotation()
{
    // Calculate rotation needed to go from current orientation to identity (sun-pointing)
    auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(currentOrientation, Matrix3d::Identity());

    // Create find sun rotation command using default velocity/acceleration
    RotationCommand findSunCommand(rotAxis.normalized(), rotAngle, refVelocity, refAcceleration);

    // Create a temporary queue to prepend the find sun command
    std::queue<RotationCommand> tempQueue;
    tempQueue.push(findSunCommand);

    // Move all existing commands to temp queue
    while (!rotationQueue.empty())
    {
        tempQueue.push(rotationQueue.front());
        rotationQueue.pop();
    }

    // Replace the original queue with temp queue
    rotationQueue = std::move(tempQueue);

    logger.info("Find Sun rotation prepended to queue:");
    logger.info("  Axis: ", rotAxis.normalized());
    logger.info("  Angle (rad): ", rotAngle);
    logger.info("  Angle (deg): ", Rad2Deg(rotAngle));
    logger.info("  Total commands in queue: ", rotationQueue.size());
}

Vector3d AttitudeControl::cascadeControl(Matrix3d target, Matrix3d current, Vector3d refAngularVelocityVec)
{
    // Calculate the position error
    auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(target, current);
    rotAxis = rotAxis.normalized();
    Vector3d positionError = current.transpose() * (rotAxis * rotAngle);

    // Outer loop output
    Vector3d omegaRefBody;
    omegaRefBody(0) = xPositionLoop.PICalculation(0, positionError(0));
    omegaRefBody(1) = yPositionLoop.PICalculation(0, positionError(1));
    omegaRefBody(2) = zPositionLoop.PICalculation(0, positionError(2));
    Vector3d omegaCmdBody = omegaRefBody + refAngularVelocityVec;

    // Feed into inner velocity loop
    Vector3d torque;
    torque(0) = xVelocityLoop.PICalculation(omegaCmdBody(0), angularVelocityVec(0));
    torque(1) = yVelocityLoop.PICalculation(omegaCmdBody(1), angularVelocityVec(1));
    torque(2) = zVelocityLoop.PICalculation(omegaCmdBody(2), angularVelocityVec(2));

    return torque;
};

Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector)
{
    double a = exp(-2 * M_PI * fc * dt);
    return (1.0 - a) * curVector + a * prevVector;
}

// Helper function to calculate rotation axis and angle between two rotation matrices
std::pair<Vector3d, double> calculateRotationAxisAndAngle(const Matrix3d &fromMatrix, const Matrix3d &toMatrix)
{
    Matrix3d rotation = AngularVel::RotMatBodyToBody(fromMatrix, toMatrix);
    AngleAxisd angleAxis{rotation};

    Vector3d rotAxis = angleAxis.axis();
    double rotAngle = angleAxis.angle();

    return std::make_pair(rotAxis, rotAngle);
}

uint16_t AttitudeControl::torqueToFanTarget(double torque)
{
    // Clamp to [-1, 1] range

    double speed = AttitudeConfig::torqueToSpeed * (abs(torque));
    double neutral = 2048;
    double deadband = 75;
    double target;
    if (torque > 0)
        target = neutral + deadband + speed;
    else
        target = neutral - deadband - speed;

    // Ensure target is within valid range (should already be, but safety check)
    if (target < 1448)
        target = 1448;
    if (target > 2648)
        target = 2648;

    return static_cast<uint16_t>(target);
}
