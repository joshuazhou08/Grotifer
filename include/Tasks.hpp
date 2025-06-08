// Header files for all the tasks
#pragma once
#include "BaseTask.hpp" // BaseTask class declaration
#include <fstream>
#include "Config.hpp"

// -----------------------
// Attitude Control
// -----------------------

class AttitudeControl : public BaseTask
{
public:
    AttitudeControl();
    ~AttitudeControl() override;

    int Run() override;

private:
    // Configs
    AttitudeConfig config;
    // States
    static constexpr int INITIALIZING = 0;
    static constexpr int DETERMINING_ALTITUDE = 1;
    static constexpr int INITIALIZING_MOTION = 2;
    static constexpr int DETUMBLING = 3;
    static constexpr int FINDING_SUN = 4;
    static constexpr int HOLDING_POSITION = 5;
    static constexpr int MOVING = 6;

    std::ofstream attitudeLog; // holds the body fixed frame
    std::ofstream sensorLog;
    std::ofstream angularVelLog;
    std::ofstream momentumWheelsLog;

    // max accelerations for momentum wheels
    double maxAccCmdX, maxAccCmdY, maxAccCmdZ;
};