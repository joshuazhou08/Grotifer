#include "Tasks.hpp"
#include "Config.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem> // optional, for auto-creating log directory
#include <cmath>

using namespace std;
// Forward delcaration for a filter
Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector);

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
      p_inclinometer(std::move(inclinometer))
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

    attitudeLog.open("logs/attitude.txt", std::ios::out | std::ios::app);
    sensorLog.open("logs/sensors.txt", std::ios::out | std::ios::app);
    angularVelLog.open("logs/angular_velocity.txt", std::ios::out | std::ios::app);
    momentumWheelsLog.open("logs/momentum_wheels.txt", std::ios::out | std::ios::app);

    attitudeLog << left << setw(w) << "Time (s)" << left << setw(w) << "Xb_x" << left << setw(w) << "Xb_y" << left << setw(w) << "Xb_z"
                << left << setw(w) << "Yb_x" << left << setw(w) << "Yb_y" << left << setw(w) << "Yb_z"
                << left << setw(w) << "Zb_x" << left << setw(w) << "Zb_y" << left << setw(w) << "Zb_z" << endl;

    sensorLog << left << setw(w) << "Time (s)" << left << setw(w) << "thxIncl(deg)" << left << setw(w) << "thzIncl(deg)"
              << left << setw(w) << "thySun(deg)" << left << setw(w) << "thzSun(deg)";

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

    switch (state)
    {
    case INITIALIZING:
    {
        if (config.initialKick)
        {
            iniMotionDone = false;
            iniKickEndTime = GetTimeNow() + config.iniKickDuration;
            nextState = INITIALIZING_MOTION;
        }
        else
        {
            iniMotionDone = true;
            nextState = DETERMINING_ATTITUDE;
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
            cout << "Fail to read Sun Sensor!" << endl;
        }

        thySun = Deg2Rad(thySun); // Get Y-angle from Sun Sensor
        thzSun = Deg2Rad(thzSun); // Get Z-angle from Sun Sensor

        if (sunInfo != 0)
        {
            cout << "Sun Message: " << (*p_sunSensor).GetAddMessage(sunInfo) << endl;
        }

        // Read the Inclinometer
        double thxIncl, thzIncl;

        thxIncl = Deg2Rad((*p_inclinometer).GetAngleY());
        thzIncl = Deg2Rad((*p_inclinometer).GetAngleX());

        // Calculating the attitude
        solRotMatBackward = BackwardSol::AlgebraicSolutionMatrix(thxIncl, thzIncl, thySun, thzSun);

        // Get the axes
        Vector3d bodyX, bodyY, bodyZ;
        bodyX = solRotMatBackward * Vector3d(1.0, 0.0, 0.0);
        bodyY = solRotMatBackward * Vector3d(0.0, 1.0, 0.0);
        bodyZ = solRotMatBackward * Vector3d(0.0, 0.0, 1.0);

        // Calculate Euler Angles
        double thxEuler, thyEuler, thzEuler;
        BackwardSol::GetEulerFromRot(solRotMatBackward, &thxEuler, &thyEuler, &thzEuler);

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
            Matrix3d curRotMat = solRotMatBackward * iniRotMat.transpose(); // TODO: look into why the iniRotMat is needed
            AngularVel::time_interval = time - preTime;                     // Calculate the actual delta_t between two consecutive calls
            angularVelocityVec = AngularVel::GetAngularVelVec(prevRotMat, curRotMat);
            rotAngle = AngularVel::RotAngleAboutRotAxis(curRotMat);
            prevRotMat = curRotMat;
        }

        angularVelocityVec = emaFilter3d(config.fc, time - preTime, angularVelocityVec, preAngularVelocityVec); // Use Exponential-moving-average filter

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

        // Update
        preAngularVelocityVec = angularVelocityVec;
        preTime = time;

        if (!iniMotionDone)
        {
            nextState = INITIALIZING_MOTION;
        }
        else
        {
            nextState = DETERMINING_ATTITUDE;
        }
    }

    case INITIALIZING_MOTION:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeIni;
        if (time < iniKickEndTime)
        {
            applyTorque(config.iniTorqueVec, deltaT);
        }
        if (time + preTimeIni >= iniKickEndTime * 2)
        {
            iniMotionDone = true;
        }

        preTimeIni = time;
        nextState = DETERMINING_ATTITUDE;
        break;
    }

    case DETUMBLING:
    {
        double time = GetTimeNow();
        double deltaT = time - preTimeDetumbling;

        preTimeDetumbling = time;
    }
    }

    // update
    state = nextState;
    std::string nextStateString;
    if (nextState != state)
    {
        switch (nextState)
        {
        case DETERMINING_ATTITUDE:
        {
            nextStateString = "Determining Attitude";
        }
        case INITIALIZING_MOTION:
        {
            nextStateString = "Initializing Motion";
        }
        case DETUMBLING:
        {
            nextStateString = "Detumbling";
        }
        case FINDING_SUN:
        {
            nextStateString = "Finding Sun";
        }
        case HOLDING_POSITION:
        {
            nextStateString = "Holding Position";
        }
        case MOVING:
        {
            nextStateString = "Moving";
        }
        }
        std::cout << "[Attitude Control Task] Switching to state: " << nextStateString << std::endl;
    }
    nextTaskTime += deltaTaskTime;
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

    (*p_mmZ).RunWithVelocity(velCmd);
    *torqueCmdVal = torqueCmd;
    *velCmdVal = velCmd;

    zMomentumWheelVel = velCmd;
    return saturateZMomtWheelFlag;
}

void AttitudeControl::applyTorque(Vector3d torque, double deltaT)
{
    double torqueCmdX, torqueCmdY, torqueCmdZ;
    double velCmdX, velCmdY, velCmdZ;

    if (moveXMomentumWheelWithTorque(torque(0), deltaT, &torqueCmdX, &velCmdX))
        cout << "Saturate X Momentum Wheel" << endl;
    if (moveYMomentumWheelWithTorque(torque(1), deltaT, &torqueCmdY, &velCmdY))
        cout << "Saturate Y Momentum Wheel" << endl;
    if (moveZMomentumWheelWithTorque(torque(2), deltaT, &torqueCmdX, &velCmdZ))
        cout << "Saturate Z Momentum Wheel" << endl;

    // Get the data
    xMomentumWheelVel = (*p_mmX).GetVelocityIs();
    yMomentumWheelVel = (*p_mmY).GetVelocityIs();
    zMomentumWheelVel = (*p_mmZ).GetVelocityIs();
    // Log the data
    momentumWheelsLog << left << setw(w) << GetTimeNow() << left << setw(w) << torqueCmdX << left << setw(w) << velCmdX << left << setw(w) << xMomentumWheelVel
                      << left << setw(w) << torqueCmdY << left << setw(w) << velCmdY << left << setw(w) << yMomentumWheelVel
                      << left << setw(w) << torqueCmdZ << left << setw(w) << velCmdZ << left << setw(w) << zMomentumWheelVel << endl;
}

Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector)
{
    double a = exp(-2 * M_PI * fc * dt);
    return (1.0 - a) * curVector + a * prevVector;
};