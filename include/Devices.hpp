#pragma once

#include <fstream>
#include "GrotiferMaster.hpp"
#include "Config.hpp"

class Devices
{
public:
    Devices();
    ~Devices();

    // Sun sensor
    std::unique_ptr<ModbusSunSensor> p_sunSensor = nullptr;

    // functions used in for initialization
    bool initLabJack();
    bool initInclinometer();
    bool initSunSensor();
    bool initMaxonMotors();
    bool initFanControllers();

    // Release functions to transfer ownership
    std::unique_ptr<MaxonMotor> releaseMMX();
    std::unique_ptr<MaxonMotor> releaseMMY();
    std::unique_ptr<MaxonMotor> releaseMMZ();
    std::unique_ptr<LabJackU6> releaseLabJackU6();
    std::unique_ptr<LabJackInclinometer> releaseInclinometer();
    std::unique_ptr<ModbusSunSensor> releaseSunSensor();
    std::unique_ptr<FanController> releaseFanX();
    std::unique_ptr<FanController> releaseFanZ();

private:
    HANDLE hDevice = nullptr;      // labjack handle
    modbus_t *modbusCtx = nullptr; // modbus context

    // motor parameter structs
    maxon xMotorParams;
    maxon yMotorParams;
    maxon zMotorParams;

    // Maxon motors
    std::unique_ptr<MaxonMotor> p_mmX = nullptr;
    std::unique_ptr<MaxonMotor> p_mmY = nullptr;
    std::unique_ptr<MaxonMotor> p_mmZ = nullptr;

    // Error log
    std::ofstream eposErrFile;

    // Labjack
    std::unique_ptr<LabJackU6> p_grotiferLJU6 = nullptr;
    std::unique_ptr<LabJackInclinometer> p_inclinometer = nullptr;

    // Fan controllers
    std::unique_ptr<FanController> p_fanX = nullptr;
    std::unique_ptr<FanController> p_fanZ = nullptr;
};