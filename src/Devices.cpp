#include <fstream>
#include <iostream>
#include "Devices.hpp"
#include "GrotiferMaster.hpp"
#include <cstdlib>

using namespace std;

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
        cerr << "[Devices] Failed to open LabJack" << endl;
        return false;
    }
    cout << "[Devices] LabJack successfully opened" << endl;
    p_grotiferLJU6 = make_unique<LabJackU6>(hDevice, 0);
    return true;
}

bool Devices::initInclinometer()
{
    if (!p_grotiferLJU6)
    {
        cerr << "[Devices] Failed to open inclinometer. Make sure to call initLabJack first!" << endl;
        return false;
    }
    cout << "[Devices] Inclinometer successfully opened" << endl;
    p_inclinometer = make_unique<LabJackInclinometer>(*p_grotiferLJU6, 0, 1);
    return true;
}

bool Devices::initSunSensor()
{
    if (!OpenSunSensor(SUNSENSOR_SERIAL_PORT, 115200, modbusCtx))
    {
        cerr << "[Devices] Failed to open Sun Sensor" << endl;
        return false;
    }
    cout << "[Devices] Sun Sensor successfully opened" << endl;
    p_sunSensor = make_unique<ModbusSunSensor>(modbusCtx);
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
        cerr << "[Devices] Failed to open EPOS Error Log file" << endl;
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
            cout << "[Devices] Maxon Motor X found" << endl;
        }
        if (serialNo[i] == yMotorParams.serialNo)
        {
            yMotorParams.keyHandle = keyHandle[i];
            yMotorParams.portName = &availPortNameList[i][0];
            cout << "[Devices] Maxon Motor Y found" << endl;
        }
        /* if (serialNo[i] == zMotorParams.serialNo)
        {
            zMotorParams.keyHandle = keyHandle[i];
            zMotorParams.portName = &availPortNameList[i][0];
            cout << "[Devices] Maxon Motor Z found" << endl;
        } */
    }

    // initialize Maxon objects
    p_mmX = make_unique<MaxonMotor>(eposErrFile, xMotorParams, 'v');
    p_mmY = make_unique<MaxonMotor>(eposErrFile, yMotorParams, 'v');
    // p_mmZ = make_unique<MaxonMotor>(eposErrFile, zMotorParams, 'v');

    cout << "[Devices] Maxon Motors have been initialized" << endl;

    return true;
}

bool Devices::initFanControllers()
{
    p_fanX = make_unique<FanController>(X_FAN_SERIAL_PORT, 9600);
    p_fanZ = make_unique<FanController>(Z_FAN_SERIAL_PORT, 9600);

    p_fanX->moveFan(2048);
    p_fanZ->moveFan(2048);
    cout << "[Devices] X and Z fans initialized" << endl;
    return true;
}

// Ownership transfer functions
unique_ptr<MaxonMotor> Devices::releaseMMX()
{
    if (!p_mmX)
    {
        cerr << "[Devices] Motor X has not been initialized!" << endl;
    }
    return move(p_mmX);
}

unique_ptr<MaxonMotor> Devices::releaseMMY()
{
    if (!p_mmY)
    {
        cerr << "[Devices] Motor Y has not been initialized!" << endl;
    }
    return move(p_mmY);
}

unique_ptr<MaxonMotor> Devices::releaseMMZ()
{
    if (!p_mmZ)
    {
        cerr << "[Devices] Motor Z has not been initialized!" << endl;
    }
    return move(p_mmZ);
}

unique_ptr<LabJackU6> Devices::releaseLabJackU6()
{
    if (!p_grotiferLJU6)
    {
        cerr << "[Devices] LabJack has not been initialized!" << endl;
    }
    return move(p_grotiferLJU6);
}

unique_ptr<LabJackInclinometer> Devices::releaseInclinometer()
{
    if (!p_inclinometer)
    {
        cerr << "[Devices] Inclinometer has not been initialized!" << endl;
    }
    return move(p_inclinometer);
}

unique_ptr<ModbusSunSensor> Devices::releaseSunSensor()
{
    if (!p_sunSensor)
    {
        cerr << "[Devices] Sun Sensor has not been initialized!" << endl;
    }
    return move(p_sunSensor);
}

unique_ptr<FanController> Devices::releaseFanX()
{
    if (!p_fanX)
    {
        cerr << "[Devices] Fan X has not been initialized!" << endl;
    }
    return move(p_fanX);
}

unique_ptr<FanController> Devices::releaseFanZ()
{
    if (!p_fanZ)
    {
        cerr << "[Devices] Fan Z has not been initialized!" << endl;
    }
    return move(p_fanZ);
}