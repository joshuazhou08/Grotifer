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
// Forward delcaration for a filter
Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector);
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
      xVelocityLoop(
          config.xVelocityK_p,
          config.xVelocityK_i,
          config.xVelocityhLim,
          config.xVelocitylLim,
          config.xVelocityK_d),
      yVelocityLoop(
          config.yVelocityK_p,
          config.yVelocityK_i,
          config.yVelocityhLim,
          config.yVelocitylLim,
          config.yVelocityK_d),
      zVelocityLoop(
          config.zVelocityK_p,
          config.zVelocityK_i,
          config.zVelocityhLim,
          config.zVelocitylLim,
          config.zVelocityK_d),
      xPositionLoop(
          config.xPositionK_p,
          config.xPositionK_i,
          config.xPositionhLim,
          config.xPositionlLim,
          config.xPositionK_d),
      yPositionLoop(
          config.yPositionK_p,
          config.yPositionK_i,
          config.yPositionhLim,
          config.yPositionlLim,
          config.yPositionK_d),
      zPositionLoop(
          config.zPositionK_p,
          config.zPositionK_i,
          config.zPositionhLim,
          config.zPositionlLim,
          config.zPositionK_d)

{
    InitializeLogs();

    if (!attitudeLog || !sensorLog || !angularVelLog || !momentumWheelsLog)
    {
        throw std::runtime_error("Failed to open one or more log files in AttitudeControl.");
    }

    deltaTaskTime = config.deltaTaskTime;
    maxAccCmdX = (*p_mmX).GetMaxTorque() / config.momOfInertiaX;
    maxAccCmdY = (*p_mmY).GetMaxTorque() / config.momOfInertiaY;
    maxAccCmdZ = (*p_mmZ).GetMaxTorque() / config.momOfInertiaZ;
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

    Logger::info("State: ", stateName);

    switch (state)
    {
    case INITIALIZING:
    {
        if (config.initialKick)
        {
            iniMotionDone = false;
            iniKickEndTime = GetTimeNow() + config.iniKickDuration;
            preTimeIni = GetTimeNow();
            nextState = INITIALIZING_MOTION;
            nextStateName = "Initializing Motion";
            Logger::info("[Attitude Control] Finished Intializing. Going into initial kick.");
        }
        else
        {
            iniMotionDone = true;
            detumblingEndTime = GetTimeNow() + config.detumblingMaxDuration;
            preTimeDetumbling = GetTimeNow();
            nextState = DETERMINING_ATTITUDE;
            nextStateName = "Determining Attitude";
            Logger::info("[Attitude Control] Finished Intializing. Going into detumble.");
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
            Logger::error("Fail to read Sun Sensor!");
        }

        thySun = Deg2Rad(thySun); // Get Y-angle from Sun Sensor
        thzSun = Deg2Rad(thzSun); // Get Z-angle from Sun Sensor

        if (sunInfo != 0)
        {
            Logger::info("Sun Message: ", (*p_sunSensor).GetAddMessage(sunInfo));
        }

        // Read the Inclinometer
        double thxIncl, thzIncl;

        thxIncl = Deg2Rad((*p_inclinometer).GetAngleY());
        thzIncl = Deg2Rad((*p_inclinometer).GetAngleX());

        // Calculating the attitude
        solRotMatBackward = BackwardSol::AlgebraicSolutionMatrix(thxIncl, thzIncl, thySun, thzSun);
        Logger::debug("[DEBUG] Orientation Matrix Calculated");

        // Get the axes
        Vector3d bodyX, bodyY, bodyZ;
        bodyX = solRotMatBackward * Vector3d(1.0, 0.0, 0.0);
        bodyY = solRotMatBackward * Vector3d(0.0, 1.0, 0.0);
        bodyZ = solRotMatBackward * Vector3d(0.0, 0.0, 1.0);

        // Calculate Euler Angles
        double thxEuler, thyEuler, thzEuler;
        BackwardSol::GetEulerFromRot(solRotMatBackward, &thxEuler, &thyEuler, &thzEuler);
        Logger::debug("[DEBUG] Euler Angles Calculated");

        double time = GetTimeNow(); // Get the current time

        // Calculate the angular velocity
        if (iniRotMat.isZero()) // Initial state has not been set
        {
            iniRotMat = solRotMatBackward;
            prevRotMat = iniRotMat;
            angularVelocityVec << 0.0, 0.0, 0.0;
            preTime = time;
        }
        else
        {
            Matrix3d curRotMat = solRotMatBackward;
            double deltaT = time - preTime;                                                                           // Calculate the actual delta_t between two consecutive calls
            angularVelocityVec = curRotMat.transpose() * AngularVel::GetAngularVelVec(prevRotMat, curRotMat, deltaT); // convert angular velocity to body fixed frame
            rotAngle = AngularVel::RotAngleAboutRotAxis(curRotMat);
            prevRotMat = curRotMat;
        }

        angularVelocityVec = emaFilter3d(config.fc, time - preTime, angularVelocityVec, preAngularVelocityVec); // Use Exponential-moving-average filter
        Logger::debug("[DEBUG] Angular Velocity Vector Calculated");

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

        Logger::debug("[DEBUG] Attitude Data Logged");

        // Update
        preAngularVelocityVec = angularVelocityVec;
        preTime = time;
        if (!iniMotionDone)
        {
            nextState = INITIALIZING_MOTION;
            nextStateName = "Initializing Motion";
        }
        else if (!detumblingDone)
        {
            nextState = DETUMBLING;
            nextStateName = "Detumbling";
        }

        else
        {
            nextState = HOLDING_POSITION;
            if (!holdingPositionSet)
            {
                holdingPosition.setIdentity();
                holdingPositionSet = true;
                Logger::info("holding position: ", holdingPosition);
                preTimeHoldingPos = time;
            }
            nextStateName = "Holding Position";
        }

        break;
    }

    case INITIALIZING_MOTION:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeIni;
        if (time < iniKickEndTime)
        {
            applyTorque(config.iniTorqueVec, deltaT);
        }

        else
        {
            Vector3d zeroVector{{0.0, 0.0, 0.0}};
            applyTorque(zeroVector, deltaT);
        }
        if (time + preTimeIni >= iniKickEndTime * 2)
        {
            iniMotionDone = true;
            Logger::info("[Attitude Control] Initial Kick Done");
            detumblingEndTime = GetTimeNow() + config.detumblingMaxDuration;
            preTimeDetumbling = GetTimeNow();
        }

        preTimeIni = time;
        nextState = DETERMINING_ATTITUDE;
        nextStateName = "Determining Attitude";

        break;
    }

    case DETUMBLING:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeDetumbling;

        Vector3d torque;
        torque(0) = -config.xVelocityK_p * angularVelocityVec(0);
        torque(1) = -config.yVelocityK_p * angularVelocityVec(1);
        torque(2) = -config.zVelocityK_p * angularVelocityVec(2);

        applyTorque(torque, deltaT);

        double max_component = angularVelocityVec.cwiseAbs().maxCoeff();

        if (time >= detumblingEndTime && max_component < 4.5e-3)
        {
            detumblingDone = true;

            Logger::info("[Attitude Control] Detumbling Done");
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
        Matrix3d curRotMat = solRotMatBackward;
        Matrix3d rotation = AngularVel::RotMatBodyToBody(holdingPosition, curRotMat); // Rotation from where you are to where you want to hold
        AngleAxisd angleAxis{rotation};

        Vector3d rotAxis = angleAxis.axis();
        rotAxis(0) *= -1;
        double rotAngle = angleAxis.angle();
        Vector3d signal = curRotMat.transpose() * (rotAxis * rotAngle);

        Vector3d torque;
        torque(0) = -config.xVelocityK_p * angularVelocityVec(0) + xPositionLoop.PICalculation(0, signal(0));
        torque(1) = -config.yVelocityK_p * angularVelocityVec(1) + yPositionLoop.PICalculation(0, signal(1));
        torque(2) = -config.zVelocityK_p * angularVelocityVec(2) + zPositionLoop.PICalculation(0, signal(2));
        applyTorque(torque, deltaT);

        nextState = DETERMINING_ATTITUDE;
        nextStateName = "Determining Attitude";
        preTimeHoldingPos = time;
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

    accCmd = torqueCmd / config.momOfInertiaX;
    velCmd = xMomentumWheelVel + (accCmd * deltaT) * (30 / M_PI); // conversion factor

    if (accCmd >= maxAccCmdX)
        accCmd = maxAccCmdX;
    else if (accCmd <= -maxAccCmdX)
        accCmd = -maxAccCmdX;

    // Check for saturation
    if (velCmd > config.maxVelX)
    {
        saturateXMomtWheelFlag = true;
        velCmd = config.maxVelX;
    }
    else if (velCmd < -config.maxVelX)
    {
        saturateXMomtWheelFlag = true;
        velCmd = -config.maxVelX;
    }

    (*p_mmX).RunWithVelocity(velCmd);
    *torqueCmdVal = torqueCmd;
    *velCmdVal = velCmd;

    xMomentumWheelVel = velCmd;
    Logger::debug("[DEBUG] Moving X");
    return saturateXMomtWheelFlag;
}

bool AttitudeControl::moveYMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal)
{
    bool saturateYMomtWheelFlag = false;
    double accCmd = 0; // Command value for acceleration
    int velCmd = 0;
    double torqueCmd = -torque; // y must be flipped
    accCmd = torqueCmd / config.momOfInertiaY;
    velCmd = yMomentumWheelVel + (accCmd * deltaT) * (30 / M_PI); // conversion factor

    if (accCmd >= maxAccCmdY)
        accCmd = maxAccCmdY;
    else if (accCmd <= -maxAccCmdY)
        accCmd = -maxAccCmdY;

    // Check for saturation
    if (velCmd > config.maxVelY)
    {
        saturateYMomtWheelFlag = true;
        velCmd = config.maxVelY;
    }
    else if (velCmd < -config.maxVelY)
    {
        saturateYMomtWheelFlag = true;
        velCmd = -config.maxVelY;
    }
    (*p_mmY).RunWithVelocity(velCmd);
    *torqueCmdVal = torqueCmd;
    *velCmdVal = velCmd;

    yMomentumWheelVel = velCmd;
    Logger::debug("[DEBUG] Moving Y");

    return saturateYMomtWheelFlag;
}

bool AttitudeControl::moveZMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal)
{
    bool saturateZMomtWheelFlag = false;
    double accCmd = 0; // Command value for acceleration
    int velCmd = 0;
    double torqueCmd = -torque; // Z is flipped

    accCmd = torqueCmd / config.momOfInertiaZ;
    velCmd = zMomentumWheelVel + (accCmd * deltaT) * (30 / M_PI); // conversion factor

    if (accCmd >= maxAccCmdZ)
        accCmd = maxAccCmdZ;
    else if (accCmd <= -maxAccCmdZ)
        accCmd = -maxAccCmdZ;

    // Check for saturation
    if (velCmd > config.maxVelZ)
    {
        saturateZMomtWheelFlag = true;
        velCmd = config.maxVelZ;
    }
    else if (velCmd < -config.maxVelZ)
    {
        saturateZMomtWheelFlag = true;
        velCmd = -config.maxVelZ;
    }

    Logger::debug("[DEBUG] Giving Z Wheel a Velocity Command");

    (*p_mmZ).RunWithVelocity(velCmd);
    *torqueCmdVal = torqueCmd;
    *velCmdVal = velCmd;

    Logger::debug("[DEBUG] Z Velocity Successfully Commanded");

    zMomentumWheelVel = velCmd;

    return saturateZMomtWheelFlag;
}

void AttitudeControl::applyTorque(Vector3d torque, double deltaT)
{
    double torqueCmdX, torqueCmdY, torqueCmdZ;
    double velCmdX, velCmdY, velCmdZ;

    Logger::debug("[DEBUG] Applying torque: ", torque);

    if (moveXMomentumWheelWithTorque(torque(0), deltaT, &torqueCmdX, &velCmdX))
        Logger::warning("Saturate X Momentum Wheel");
    if (moveYMomentumWheelWithTorque(torque(1), deltaT, &torqueCmdY, &velCmdY))
        Logger::warning("Saturate Y Momentum Wheel");
    if (moveZMomentumWheelWithTorque(torque(2), deltaT, &torqueCmdX, &velCmdZ))
        Logger::warning("Saturate Z Momentum Wheel");

    Logger::debug("[DEBUG] Successfully applied torque");

    // Get the data
    xMomentumWheelVel = (*p_mmX).GetVelocityIs();
    yMomentumWheelVel = (*p_mmY).GetVelocityIs();
    zMomentumWheelVel = (*p_mmZ).GetVelocityIs();
    // Log the data
    momentumWheelsLog << left << setw(w) << GetTimeNow() << left << setw(w) << torqueCmdX << left << setw(w) << velCmdX << left << setw(w) << xMomentumWheelVel
                      << left << setw(w) << torqueCmdY << left << setw(w) << velCmdY << left << setw(w) << yMomentumWheelVel
                      << left << setw(w) << torqueCmdZ << left << setw(w) << velCmdZ << left << setw(w) << zMomentumWheelVel << endl;
    Logger::debug("[DEBUG] Momentum Wheel Data Logged");
}

Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector)
{
    double a = exp(-2 * M_PI * fc * dt);
    return (1.0 - a) * curVector + a * prevVector;
};