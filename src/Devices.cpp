#pragma once

#include <fstream>
#include <iostream>
#include "Devices.hpp"
#include "GrotiferMaster.hpp"
#include <cstdlib>

#define SUNSENSOR_SERIAL_PORT "/dev/digital_sun_sensor"
// Constructor
Devices::Devices()
{
}

// Destructor
Devices::~Devices()
{
}

// Helper functions
bool Devices::initLabJack()
{
    if (!OpenLabJackU6(hDevice))
    {
        std::cerr << "Failed to open LabJack\n";
        return false;
    }
    p_grotiferLJU6 = std::make_unique<LabJackU6>(hDevice, 0);
    return true;
}

bool Devices::initInclinometer()
{
    if (!p_grotiferLJU6)
    {
        std::cerr << "Failed to open the inclinometer. Make sure to call initLabjack first! \n";
        return false;
    }
    p_inclinometer = std::make_unique<LabJackInclinometer>(*p_grotiferLJU6, 0, 1);
    return true;
}

bool Devices::initSunSensor()
{
    if (!OpenSunSensor(SUNSENSOR_SERIAL_PORT, 115200, modbusCtx))
    {
        std::cerr << "Failed to open Sun Sensor\n";
        return false;
    }
    p_sunSensor = std::make_unique<ModbusSunSensor>(modbusCtx);
    return true;
}

bool Devices::initMaxonMotors()
{
    const unsigned int noOfOpenedPorts = 5;
    void *keyHandle[noOfOpenedPorts];
    uint32_t serialNo[noOfOpenedPorts];

    // start error file log
    eposErrFile.open("SystemData/Maxon EPOS Error Log.txt");
    if (!eposErrFile.is_open())
    {
    std:
        cerr << "Failed to open EPOS Error Log file/n";
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
        }
        if (serialNo[i] == yMotorParams.serialNo)
        {
            yMotorParams.keyHandle = keyHandle[i];
            yMotorParams.portName = &availPortNameList[i][0];
        }
        if (serialNo[i] == zMotorParams.serialNo)
        {
            zMotorParams.keyHandle = keyHandle[i];
            zMotorParams.portName = &availPortNameList[i][0];
        }
    }

    // initialize Maxon objects
    p_mmX = std::make_unique<MaxonMotor>(eposErrFile, xMotorParams, 'v');
    p_mmY = std::make_unique<MaxonMotor>(eposErrFile, yMotorParams, 'v');
    p_mmZ = std::make_unique<MaxonMotor>(eposErrFile, zMotorParams, 'v');

    return true;
}
