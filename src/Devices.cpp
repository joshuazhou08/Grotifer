#include <fstream>
#include <iostream>
#include "Devices.hpp"
#include "GrotiferMaster.hpp"
#include "Logger.hpp"
#include <cstdlib>

#define SUNSENSOR_SERIAL_PORT "/dev/digital_sun_sensor"
#define X_FAN_SERIAL_PORT "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_21v3_00464559-if01"
#define Z_FAN_SERIAL_PORT "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_21v3_00464472-if01"
// Constructor
Devices::Devices()
{
}

// Destructor
Devices::~Devices()
{
}

// Initialize functions
bool Devices::initLabJack()
{
    if (!OpenLabJackU6(hDevice))
    {
        Logger::error("Failed to open LabJack");
        return false;
    }
    Logger::info("[LabJack] Successfully Opened");
    p_grotiferLJU6 = std::make_unique<LabJackU6>(hDevice, 0);
    return true;
}

bool Devices::initInclinometer()
{
    if (!p_grotiferLJU6)
    {
        Logger::error("Failed to open the inclinometer. Make sure to call initLabjack first!");
        return false;
    }
    Logger::info("[Inclinometer] Successfully Opened");
    p_inclinometer = std::make_unique<LabJackInclinometer>(*p_grotiferLJU6, 0, 1);
    return true;
}

bool Devices::initSunSensor()
{
    if (!OpenSunSensor(SUNSENSOR_SERIAL_PORT, 115200, modbusCtx))
    {
        Logger::error("Failed to open Sun Sensor");
        return false;
    }
    Logger::info("[Sun Sensor] Successfully Opened");
    p_sunSensor = std::make_unique<ModbusSunSensor>(modbusCtx);
    return true;
}

bool Devices::initMaxonMotors()
{
    const unsigned int noOfOpenedPorts = 5;
    void *keyHandle[noOfOpenedPorts];
    uint32_t serialNo[noOfOpenedPorts];

    // start error file log
    eposErrFile.open("logs/Maxon EPOS Error Log.txt");
    if (!eposErrFile.is_open())
    {
        Logger::error("Failed to open EPOS Error Log file");
        exit(EXIT_FAILURE);
    }

    // motor parameter structs
    xMotorParams = makeXMotor();
    yMotorParams = makeYMotor();
    zMotorParams = makeZMotor();

    std::string *availPortNameList;

    if (!openMaxonMotors(keyHandle, serialNo, availPortNameList))
    {
        return false;
    }

    // Match handle with Maxon board
    for (unsigned int i = 0; i < noOfOpenedPorts; i++)
    {
        if (serialNo[i] == xMotorParams.serialNo)
        {
            xMotorParams.keyHandle = keyHandle[i];
            xMotorParams.portName = &availPortNameList[i][0];
            Logger::info("[Maxon Motor] X found");
        }
        if (serialNo[i] == yMotorParams.serialNo)
        {
            yMotorParams.keyHandle = keyHandle[i];
            yMotorParams.portName = &availPortNameList[i][0];
            Logger::info("[Maxon Motor] Y found");
        }
        /* if (serialNo[i] == zMotorParams.serialNo)
        {
            zMotorParams.keyHandle = keyHandle[i];
            zMotorParams.portName = &availPortNameList[i][0];
            Logger::info("[Maxon Motor] Z found");
        } */
    }

    // initialize Maxon objects
    p_mmX = std::make_unique<MaxonMotor>(eposErrFile, xMotorParams, 'v');
    p_mmY = std::make_unique<MaxonMotor>(eposErrFile, yMotorParams, 'v');
    // p_mmZ = std::make_unique<MaxonMotor>(eposErrFile, zMotorParams, 'v');

    Logger::info("[Maxon Motor] Has been initialized!");

    return true;
}

bool Devices::initFanControllers()
{
    p_fanX = std::make_unique<FanController>(X_FAN_SERIAL_PORT, 9600);
    p_fanZ = std::make_unique<FanController>(Z_FAN_SERIAL_PORT, 9600);

    p_fanX->moveFan(2048);
    p_fanZ->moveFan(2048);
    Logger::info("[Fan Controllers] X and Z fans initialized");
    return true;
}

// Ownership transfer functions
std::unique_ptr<MaxonMotor> Devices::releaseMMX()
{
    if (!p_mmX)
    {
        Logger::error("Motor X has not been initialized!");
    }
    return std::move(p_mmX);
}

std::unique_ptr<MaxonMotor> Devices::releaseMMY()
{
    if (!p_mmY)
    {
        Logger::error("Motor Y has not been initialized!");
    }
    return std::move(p_mmY);
}

std::unique_ptr<MaxonMotor> Devices::releaseMMZ()
{
    if (!p_mmZ)
    {
        Logger::error("Motor Z has not been initialized!");
    }
    return std::move(p_mmZ);
}

std::unique_ptr<LabJackU6> Devices::releaseLabJackU6()
{
    if (!p_grotiferLJU6)
    {
        Logger::error("Labjack has not been initialized!");
    }
    return std::move(p_grotiferLJU6);
}

std::unique_ptr<LabJackInclinometer> Devices::releaseInclinometer()
{
    if (!p_inclinometer)
    {
        Logger::error("Inclinometer has not been initialized!");
    }
    return std::move(p_inclinometer);
}

std::unique_ptr<ModbusSunSensor> Devices::releaseSunSensor()
{
    if (!p_sunSensor)
    {
        Logger::error("Sun Sensor has not been initialized!");
    }
    return std::move(p_sunSensor);
}

std::unique_ptr<FanController> Devices::releaseFanX()
{
    if (!p_fanX)
    {
        Logger::error("Fan X has not been initialized!");
    }
    return std::move(p_fanX);
}

std::unique_ptr<FanController> Devices::releaseFanZ()
{
    if (!p_fanZ)
    {
        Logger::error("Fan Z has not been initialized!");
    }
    return std::move(p_fanZ);
}