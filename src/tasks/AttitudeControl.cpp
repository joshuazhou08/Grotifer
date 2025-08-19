#include "Tasks.hpp"
#include "Config.hpp"
#include "Logger.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <cmath>
#include <future>
#include <chrono>

using namespace std;
using namespace Eigen;
// Forward delcaration for helpers
Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector);
std::pair<Vector3d, double> calculateRotationAxisAndAngle(const Matrix3d& fromMatrix, const Matrix3d& toMatrix);
// Forward declaration for new timeout velocity command to motor
AttitudeControl::AttitudeControl(std::unique_ptr<MaxonMotor> mmX,
                                 std::unique_ptr<MaxonMotor> mmY,
                                 std::unique_ptr<MaxonMotor> mmZ,
                                 std::unique_ptr<ModbusSunSensor> sunSensor,
                                 std::unique_ptr<LabJackInclinometer> inclinometer)
    : BaseTask("AttitudeControl", 0),
      p_mmX(std::move(mmX)),
      p_mmY(std::move(mmY)),
      p_mmZ(std::move(mmZ)),
      p_sunSensor(std::move(sunSensor)),
      p_inclinometer(std::move(inclinometer)),
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

    deltaTaskTime = AttitudeConfig::deltaTaskTime;
    maxAccCmdX = (*p_mmX).GetMaxTorque() / AttitudeConfig::momOfInertiaX;
    maxAccCmdY = (*p_mmY).GetMaxTorque() / AttitudeConfig::momOfInertiaY;
    maxAccCmdZ = (*p_mmZ).GetMaxTorque() / AttitudeConfig::momOfInertiaZ;
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

    logger.info("State: ", stateName);

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
        currentOrientation = BackwardSol::AlgebraicSolutionMatrix(thxIncl, thzIncl, thySun, thzSun);
        logger.debug("Orientation Matrix Calculated");

        // Get the axes
        Vector3d bodyX, bodyY, bodyZ;
        bodyX = currentOrientation * Vector3d(1.0, 0.0, 0.0);
        bodyY = currentOrientation * Vector3d(0.0, 1.0, 0.0);
        bodyZ = currentOrientation * Vector3d(0.0, 0.0, 1.0);

        // Calculate Euler Angles
        double thxEuler, thyEuler, thzEuler;
        BackwardSol::GetEulerFromRot(currentOrientation, &thxEuler, &thyEuler, &thzEuler);
        logger.debug("Euler Angles Calculated");

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
            double deltaT = time - preTime;                                                                           // Calculate the actual delta_t between two consecutive calls
            angularVelocityVec = currentOrientation.transpose() * AngularVel::GetAngularVelVec(prevRotMat, currentOrientation, deltaT); // convert angular velocity to body fixed frame
            rotAngle = AngularVel::RotAngleAboutRotAxis(currentOrientation);
            prevRotMat = currentOrientation;
        }

        angularVelocityVec = emaFilter3d(AttitudeConfig::fc, time - preTime, angularVelocityVec, preAngularVelocityVec); // Use Exponential-moving-average filter
        logger.debug("Angular Velocity Vector Calculated");

        // Log the data
        angularVelLog << left << setw(w) << GetTimeNow() << left << setw(w) << angularVelocityVec(0)
                      << left << setw(w) << angularVelocityVec(1)
                      << left << setw(w) << angularVelocityVec(2)
                      << left << setw(w) << refAngularVelocityVec(0)
                      << left << setw(w) << refAngularVelocityVec(1)
                      << left << setw(w) << refAngularVelocityVec(2)
                      << left << setw(w) << angularVelocityErrorVec(0)
                      << left << setw(w) << angularVelocityErrorVec(1)
                      << left << setw(w) << angularVelocityErrorVec(2)
                      << left << setw(w) << Rad2Deg(rotAngle) << left << setw(w) << Rad2Deg(thetaProf) << left << setw(w) << wProf << left << setw(w) << eProf << endl;

        attitudeLog << left << setw(w) << GetTimeNow() << left << setw(w) << bodyX(0) << left << setw(w) << bodyX(1) << left << setw(w) << bodyX(2)
                    << left << setw(w) << bodyY(0) << left << setw(w) << bodyY(1) << left << setw(w) << bodyY(2)
                    << left << setw(w) << bodyZ(0) << left << setw(w) << bodyZ(1) << left << setw(w) << bodyZ(2) << endl;

        sensorLog << left << setw(w) << GetTimeNow() << left << setw(w) << Rad2Deg(thxIncl) << left << setw(w) << Rad2Deg(thzIncl)
                  << left << setw(w) << Rad2Deg(thySun) << left << setw(w) << Rad2Deg(thzSun)
                  << left << setw(w) << Rad2Deg(thxEuler) << left << setw(w) << Rad2Deg(thyEuler) << left << setw(w) << Rad2Deg(thzEuler) << endl;

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
                //reset moving profile variables
                movingProfileVelocity = 0.0;
                movingProfileAngle = 0.0;
                movingProfileRotAxis << 0.0, 0.0, 0.0;
                startingOrientation = currentOrientation;
                if (!findSunDone) 
                {
                    auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(currentOrientation, Matrix3d::Identity());
                    deltaTheta = rotAngle;
                    movingProfileRotAxis = rotAxis.normalized();
                    logger.info("Rot Axis For Finding Sun: ", movingProfileRotAxis);
                    logger.info("Delta Theta For Finding Sun: ", deltaTheta);
                }

                else
                {
                    deltaTheta = 0; //TODO: IMPLEMENT USER INPUTTED MOVES HERE
                }
                double time = GetTimeNow();
                double accelDuration = abs(refVelocity / refAcceleration) + 2 * deltaTaskTime; 
                double constantDuration = abs(deltaTheta / refVelocity) + 2 * deltaTaskTime;
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

            logger.info("Detumbling Done");
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

        // Calculating the pi signal
        auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(holdingPosition, currentOrientation);
        Vector3d signal = currentOrientation.transpose() * (rotAxis * rotAngle);

        Vector3d torque;
        torque(0) = -AttitudeConfig::xVelocityK_p * angularVelocityVec(0) + xPositionLoop.PICalculation(0, signal(0));
        torque(1) = -AttitudeConfig::yVelocityK_p * angularVelocityVec(1) + yPositionLoop.PICalculation(0, signal(1));
        torque(2) = -AttitudeConfig::zVelocityK_p * angularVelocityVec(2) + zPositionLoop.PICalculation(0, signal(2));
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
            movingProfileVelocity = movingProfileVelocity + refAcceleration * deltaT;
            movingProfileAngle = movingProfileAngle + movingProfileVelocity * deltaT;
        }
        else if (time < movingProfileConstantEndTime)
        {
            movingProfileVelocity = refVelocity;
            movingProfileAngle = movingProfileAngle + movingProfileVelocity * deltaT;
        }
        else if (time < movingProfileDecelerationEndTime)
        {
            movingProfileVelocity = movingProfileVelocity - refAcceleration * deltaT;
            movingProfileAngle = movingProfileAngle + movingProfileVelocity * deltaT;
        }
        else
        {
            movingProfileVelocity = 0.0;
            movingProfileAngle = deltaTheta;
        }
        
        refAngularVelocityVec = currentOrientation.transpose() * (movingProfileRotAxis * movingProfileVelocity);

        // Get the profile orientation by taking the starting orientation and rotating it by the profile angle
        AngleAxis aa(movingProfileAngle, movingProfileRotAxis);
        Matrix3d movingProfileOrientation = startingOrientation * aa.toRotationMatrix();

        // Calculate the position error
        auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(movingProfileOrientation, currentOrientation);
        rotAxis = rotAxis.normalized();
        Vector3d positionError = currentOrientation.transpose() * (rotAxis * rotAngle);

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

        angularVelocityErrorVec << xVelocityLoop.GetError(), yVelocityLoop.GetError(), zVelocityLoop.GetError();

        applyTorque(torque, deltaT);
        
        double maxComponent = angularVelocityVec.cwiseAbs().maxCoeff();
        if (time > movingProfileDecelerationEndTime && maxComponent <= 4.5e-3)
        {
            movingDone = true;
            if (!findSunDone) 
            {
                findSunDone = true;                
                setHoldingPosition(Eigen::Matrix3d::Identity());
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                logger.info("Find sun done!");
            }

            else 
            {
                setHoldingPosition(movingProfileOrientation);
                nextState = HOLDING_POSITION;
                nextStateName = "Holding Position";
                logger.info("Current move done!");

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
    logger.debug("Moving X");
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
    logger.debug("Moving Y");

    return saturateYMomtWheelFlag;
}

bool AttitudeControl::moveZMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal)
{
    bool saturateZMomtWheelFlag = false;
    double accCmd = 0; // Command value for acceleration
    int velCmd = 0;
    double torqueCmd = -torque; // Z is flipped

    accCmd = torqueCmd / AttitudeConfig::momOfInertiaZ;
    velCmd = zMomentumWheelVel + (accCmd * deltaT) * (30 / M_PI); // conversion factor

    if (accCmd >= maxAccCmdZ)
        accCmd = maxAccCmdZ;
    else if (accCmd <= -maxAccCmdZ)
        accCmd = -maxAccCmdZ;

    // Check for saturation
    if (velCmd > AttitudeConfig::maxVelZ)
    {
        saturateZMomtWheelFlag = true;
        velCmd = AttitudeConfig::maxVelZ;
    }
    else if (velCmd < -AttitudeConfig::maxVelZ)
    {
        saturateZMomtWheelFlag = true;
        velCmd = -AttitudeConfig::maxVelZ;
    }

    logger.debug("Giving Z Wheel a Velocity Command");

    (*p_mmZ).RunWithVelocity(velCmd);
    *torqueCmdVal = torqueCmd;
    *velCmdVal = velCmd;

    logger.debug("Z Velocity Successfully Commanded");

    zMomentumWheelVel = velCmd;

    return saturateZMomtWheelFlag;
}

void AttitudeControl::applyTorque(Vector3d torque, double deltaT)
{
    double torqueCmdX, torqueCmdY, torqueCmdZ;
    double velCmdX, velCmdY, velCmdZ;

    logger.debug("Applying torque: ", torque);

    if (moveXMomentumWheelWithTorque(torque(0), deltaT, &torqueCmdX, &velCmdX))
        logger.warning("Saturate X Momentum Wheel");
    if (moveYMomentumWheelWithTorque(torque(1), deltaT, &torqueCmdY, &velCmdY))
        logger.warning("Saturate Y Momentum Wheel");
    if (moveZMomentumWheelWithTorque(torque(2), deltaT, &torqueCmdX, &velCmdZ))
        logger.warning("Saturate Z Momentum Wheel");

    logger.debug("Successfully applied torque");

    // Get the data
    xMomentumWheelVel = (*p_mmX).GetVelocityIs();
    yMomentumWheelVel = (*p_mmY).GetVelocityIs();
    zMomentumWheelVel = (*p_mmZ).GetVelocityIs();
    // Log the data
    momentumWheelsLog << left << setw(w) << GetTimeNow() << left << setw(w) << torqueCmdX << left << setw(w) << velCmdX << left << setw(w) << xMomentumWheelVel
                      << left << setw(w) << torqueCmdY << left << setw(w) << velCmdY << left << setw(w) << yMomentumWheelVel
                      << left << setw(w) << torqueCmdZ << left << setw(w) << velCmdZ << left << setw(w) << zMomentumWheelVel << endl;
    logger.debug("Momentum Wheel Data Logged");
}

void AttitudeControl::setHoldingPosition(Matrix3d position) 
{
    holdingPosition = position;
    holdingPositionSet = true;
    preTimeHoldingPos = GetTimeNow();
    logger.info("holding position:", holdingPosition);

}

Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector)
{
    double a = exp(-2 * M_PI * fc * dt);
    return (1.0 - a) * curVector + a * prevVector;
};

// Helper function to calculate rotation axis and angle between two rotation matrices
std::pair<Vector3d, double> calculateRotationAxisAndAngle(const Matrix3d& fromMatrix, const Matrix3d& toMatrix)
{
    Matrix3d rotation = AngularVel::RotMatBodyToBody(fromMatrix, toMatrix);
    AngleAxisd angleAxis{rotation};
    
    Vector3d rotAxis = angleAxis.axis();
    rotAxis(0) *= -1;  // Flip X component. We should figure out why its flipped
    double rotAngle = angleAxis.angle();
    
    return std::make_pair(rotAxis, rotAngle);
};

