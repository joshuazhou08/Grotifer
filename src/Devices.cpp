#include <fstream>
#include <iostream>
#include "Devices.hpp"
#include "GrotiferMaster.hpp"
#include <cstdlib>

#define SUNSENSOR_SERIAL_PORT "/dev/digital_sun_sensor"
#define RIGHT_STEPPER_1 "/dev/right_boom_stepper_1"
#define RIGHT_STEPPER_2 "/dev/right_boom_stepper_2"
#define LEFT_STEPPER_1 "/dev/left_boom_stepper_1"
#define LEFT_STEPPER_2 "/dev/left_boom_stepper_2"

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
        std::cerr << "Failed to open LabJack\n";
        return false;
    }
    std::cout << "[LabJack] Successfully Opened" << std::endl;
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
    std::cout << "[Inclinometer] Successfully Opened" << std::endl;
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
    std::cout << "[Sun Sensor] Successfully Opened" << std::endl;
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
        std::cerr << "Failed to open EPOS Error Log file/n";
        exit(EXIT_FAILURE);
    }

    // motor parameter structs
    xMotorParams = makeXMotor();
    yMotorParams = makeYMotor();
    zMotorParams = makeZMotor();
    lMotorParams = makeLeftMotor();
    rMotorParams = makeRightMotor();

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
            std::cout << "[Maxon Motor] X found" << std::endl;
        }
        if (serialNo[i] == yMotorParams.serialNo)
        {
            yMotorParams.keyHandle = keyHandle[i];
            yMotorParams.portName = &availPortNameList[i][0];
            std::cout << "[Maxon Motor] Y found" << std::endl;
        }
        if (serialNo[i] == zMotorParams.serialNo)
        {
            zMotorParams.keyHandle = keyHandle[i];
            zMotorParams.portName = &availPortNameList[i][0];
            std::cout << "[Maxon Motor] Z found" << std::endl;
        }
        if (serialNo[i] == lMotorParams.serialNo)
        {
            lMotorParams.keyHandle = keyHandle[i];
            lMotorParams.portName = &availPortNameList[i][0];
            std::cout << "[Maxon Motor] Left found" << std::endl;
        }
        if (serialNo[i] == rMotorParams.serialNo)
        {
            rMotorParams.keyHandle = keyHandle[i];
            rMotorParams.portName = &availPortNameList[i][0];
            std::cout << "[Maxon Motor] Right found" << std::endl;
        }
    }

    // initialize Maxon objects
    p_mmX = std::make_unique<MaxonMotor>(eposErrFile, xMotorParams, 'v');
    p_mmY = std::make_unique<MaxonMotor>(eposErrFile, yMotorParams, 'v');
    p_mmZ = std::make_unique<MaxonMotor>(eposErrFile, zMotorParams, 'v');
    p_mmL = std::make_unique<MaxonMotor>(eposErrFile, lMotorParams, 'v');
    p_mmR = std::make_unique<MaxonMotor>(eposErrFile, rMotorParams, 'v');

    std::cout << "[Maxon Motor] Has been initialized!" << std::endl;

    return true;
}

bool Devices::initUSDigiEnc()
{
    // ***************** FILL THIS IN ************************
};

bool Devices::initSteppers()
{
    
    leftStepper2Para.sm = open(LEFT_STEPPER_2, O_RDWR | O_NOCTTY);
    rightStepper1Para.sm = open(RIGHT_STEPPER_1, O_RDWR | O_NOCTTY);
    rightStepper2Para.sm = open(RIGHT_STEPPER_2, O_RDWR | O_NOCTTY);

    if (leftStepper1Para.sm == -1)
    {
        perror(LEFT_STEPPER_1);
        return -1;
    }

    if (leftStepper2Para.sm == -1)
    {
        perror(LEFT_STEPPER_2);
        return -1;
    }

    if (rightStepper1Para.sm == -1)
    {
        perror(RIGHT_STEPPER_1);
        return -1;
    }

    if (rightStepper2Para.sm == -1)
    {
        perror(RIGHT_STEPPER_2);
        return -1;
    }
}

// Ownership transfer functions
std::unique_ptr<MaxonMotor> Devices::releaseMMX()
{
    if (!p_mmX)
    {
        std::cerr << "Motor X has not been initialized!" << std::endl;
    }
    return std::move(p_mmX);
}

std::unique_ptr<MaxonMotor> Devices::releaseMMY()
{
    if (!p_mmY)
    {
        std::cerr << "Motor Y has not been initialized!" << std::endl;
    }
    return std::move(p_mmY);
}

std::unique_ptr<MaxonMotor> Devices::releaseMMZ()
{
    if (!p_mmZ)
    {
        std::cerr << "Motor Z has not been initialized!" << std::endl;
    }
    return std::move(p_mmZ);
}

std::unique_ptr<MaxonMotor> Devices::releaseMML()
{
    if (!p_mmL)
    {
        std::cerr << "Left Motor has not been initialized!" << std::endl;
    }
    return std::move(p_mmL);
}

std::unique_ptr<MaxonMotor> Devices::releaseMMR()
{
    if (!p_mmR)
    {
        std::cerr << "Right Motor has not been initialized!" << std::endl;
    }
    return std::move(p_mmR);
}



std::unique_ptr<LabJackU6> Devices::releaseLabJackU6()
{
    if (!p_grotiferLJU6)
    {
        std::cerr << "Labjack has not been initialized!" << std::endl;
    }
    return std::move(p_grotiferLJU6);
}

std::unique_ptr<LabJackInclinometer> Devices::releaseInclinometer()
{
    if (!p_inclinometer)
    {
        std::cerr << "Inclinometer has not been initialized!" << std::endl;
    }
    return std::move(p_inclinometer);
}

std::unique_ptr<ModbusSunSensor> Devices::releaseSunSensor()
{
    if (!p_sunSensor)
    {
        std::cerr << "Sun Sensor has not been initialized!" << std::endl;
    }
    return std::move(p_sunSensor);
}