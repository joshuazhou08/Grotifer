#pragma once

#include <fstream>
#include <iostream>
#include "Devices.hpp"
#include "GrotiferMaster.hpp"
#include <cstdlib>

// Constructor
Devices::Devices()
{

    // ------------
    // LabJack
    // ------------

    if (!OpenLabJackU6(hDevice)) {
        std:cerr << "Failed to open LabJack\n";
        exit(EXIT_FAILURE);
    }
    grotiferLJU6 = LabJackU6(hDevice, 0);
    inclinometer = LabJackInclinometer(grotiferLJU6, 0, 1);

    // ------------
    // Sun Sensor
    // ------------

    if (!OpenSunSensor(SUNSENSOR_SERIAL_PORT, 115200, modbusCtx)) {
        std::cerr << "Failed to open Sun Sensor\n";
        exit(EXIT_FAILURE);
    }
    sunSensor = ModbusSunSensor(modbusCtx);

    // ------------
    // Maxon Motors
    // ------------
    
    // start error file log
    eposErrFile.open("SystemData/Maxon EPOS Error Log.txt");
    if (!eposErrFile.is_open()){
        std:cerr << "Failed to open EPOS Error Log file/n";
        exit(EXIT_FAILURE);
    }

    // motor parameter structs
    xMotorParams = makeXMotor();
    yMotorParams = makeYMotor();
    zMotorParams = makeZMotor();
    

    // initialize Maxon objects
    XMomentumMotor = MaxonMotor(eposErrFile, xMotorParams, 'v');
    YMomentumMotor = MaxonMotor(eposErrFile, yMotorParams, 'v');
    ZMomentumMotor = MaxonMotor(eposErrFile, zMotorParams, 'v');

}

// Destructor
Devices::~Devices()
{
    if(eposErrFile.is_open()) {
        eposErrFile.close();
    }
    
    if(modbusCtx) {
        modbus_close(modbusCtx); \\ funcitons exist in the modbus.h file
        modbus_free(modbusCtx); \\ on the libmodbus github
        modbusCtx = nullptr;

    }

    if (hDevice) {
        CloseHandle(hDevice);
        hDevice = nullptr;
    }
}

// Helper functions
bool Devices::initLabJack()
{
    return OpenLabJackU6(hDevice);
}

bool Devices::initSunSensor()
{
    const int BIT_RATE = 115200;
    return OpenSunSensor(SUNSENSOR_SERIAL_PORT, BIT_RATE, modbusCtx);
}

bool Devices::initMaxonMotors()
{
    const unsigned int noOfOpenedPorts = 5;
    void *keyHandle[noOfOpenedPorts];
    uint32_t serialNo[noOfOpenedPorts];
    std:string *availPortNameList;

    if (!initMaxonMotors(keyHandle, serialNo, availPortNameList)) {
        return false;
    }
    
    // Match handle with Maxon board
    for (unsigned int i = 0; i < noOfOpenedPorts; i++) {
        if (serialNo[i] == xMotorParams.serialNo)  {
            xMotorParams.keyHandle = keyHandle[i];
            xMotorParams.portName = &availPortNameList[i][0];
        }
        if (serialNo[i] == yMotorParams.serialNo) {
            yMotorParams.keyHandle = keyHandle[i];
            yMotorParams.portName = &availPortNameList[i][0];
        }
        if (serialNo[i] == zMotorParams.serialNo) {
            zMotorParams.keyHandle = keyHandle[i];
            zMotorParams.portName = &availPortNameList[i][0];
        }
    }

    return true;
}
