#pragma once

#include <fstream>
#include "GrotiferMaster.hpp"
#include "Config.hpp"

class Devices
{
public:
    Devices();
    ~Devices();

    // Maxon motors
    std::unique_ptr<MaxonMotor> p_mmX = nullptr;
    std::unique_ptr<MaxonMotor> p_mmY = nullptr;
    std::unique_ptr<MaxonMotor> p_mmZ = nullptr;

    // Error log
    std::ofstream eposErrFile;

    // Labjack
    std::unique_ptr<LabJackU6> p_grotiferLJU6 = nullptr;
    std::unique_ptr<LabJackInclinometer> p_inclinometer = nullptr;

    // Sun sensor
    std::unique_ptr<ModbusSunSensor> p_sunSensor = nullptr;

    // functions used in for initialization
    bool initLabJack();
    bool initInclinometer();
    bool initSunSensor();
    bool initMaxonMotors();

private:
    HANDLE hDevice = nullptr;      // labjack handle
    modbus_t *modbusCtx = nullptr; // modbus context

    // motor parameter structs
    maxon xMotorParams;
    maxon yMotorParams;
    maxon zMotorParams;
};