// Header files for all the tasks
#pragma once
#include "BaseTask.hpp" // BaseTask class declaration
#include <fstream>
#include "Config.hpp"
#include "GrotiferMaster.hpp"

// -----------------------
// Attitude Control
// -----------------------

class AttitudeControl : public BaseTask
{
public:
    AttitudeControl(MaxonMotor &mmX, MaxonMotor &mmY, MaxonMotor &mmZ,
                    ModbusSunSensor &sunSensor, LabJackInclinometer &inclinometer);
    ~AttitudeControl() override;

    int Run() override;

private:
    // Configs
    AttitudeConfig config;
    // States
    static constexpr int INITIALIZING = 0;
    static constexpr int DETERMINING_ATTITUDE = 1;
    static constexpr int INITIALIZING_MOTION = 2;
    static constexpr int DETUMBLING = 3;
    static constexpr int FINDING_SUN = 4;
    static constexpr int HOLDING_POSITION = 5;
    static constexpr int MOVING = 6;

    void InitializeLogs();
    std::ofstream attitudeLog; // holds the body fixed frame
    std::ofstream sensorLog;
    std::ofstream angularVelLog;
    std::ofstream momentumWheelsLog;

    // momentum wheels
    MaxonMotor *p_mmX, *p_mmY, *p_mmZ;

    // sun sensor
    ModbusSunSensor *p_sunSensor;
    LabJackInclinometer *p_inclinometer;

    // max accelerations for momentum wheels
    double maxAccCmdX, maxAccCmdY, maxAccCmdZ;

    // for storing state to calculate angular velocity
    Matrix3d iniRotMat{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    Matrix3d prevRotMat{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double preTime;

    // position and velocity variables
    Vector3d preAngularVelocityVec{{0.0, 0.0, 0.0}};
    Vector3d angularVelocityVec{{0.0, 0.0, 0.0}};
    Matrix3d solRotMatBackward{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double rotAngle;

    // for calculating movement profiles
    Vector3d refAngularVelocityVec{{0.0, 0.0, 0.0}};
    double thetaProf, wProf, eProf;
    Vector3d angularVelocityErrorVec{{0.0, 0.0, 0.0}};
};