#include "Tasks.hpp"
#include "Config.hpp"
#include "GrotiferMaster.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem> // optional, for auto-creating log directory

AttitudeControl::AttitudeControl()
    : BaseTask("AttitudeControl", 0)
{
    std::filesystem::create_directories("logs");

    attitudeLog.open("logs/attitude.txt", std::ios::out | std::ios::app);
    sensorLog.open("logs/sensors.txt", std::ios::out | std::ios::app);
    angularVelLog.open("logs/angular_velocity.txt", std::ios::out | std::ios::app);
    momentumWheelsLog.open("logs/momentum_wheels.txt", std::ios::out | std::ios::app);

    if (!attitudeLog || !sensorLog || !angularVelLog || !momentumWheelsLog)
    {
        throw std::runtime_error("Failed to open one or more log files in AttitudeControl.");
    }

    deltaTaskTime = config.deltaTaskTime;
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

int AttitudeControl::Run()
{
    if (GetTimeNow() < nextTaskTime)
    {
        return 0;
    }

    nextTaskTime += deltaTaskTime;
    state = nextState;

    return 0;
}