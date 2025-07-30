#include <fstream>
#include <iostream>
#include "Devices.hpp"
#include "Actuators.hpp"
#include "GrotiferMaster.hpp"
#include <cstdlib>


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
    p_lgEnc3C_r = std::make_unique<LJEncoder3Channels>(*p_grotiferLJU6, 400, 1);
    p_lgEnc3C_l = std::make_unique<LJEncoder3Channels>(*p_grotiferLJU6, 400, 2);
};

bool Devices::initSteppers()
{
    
    leftStepper1Para = leftStepper1Params();
    leftStepper2Para = leftStepper2Params();
    rightStepper1Para = rightStepper1Params();
    rightStepper2Para = rightStepper2Params();

    if (leftStepper1Para.sm == -1)
    {
        perror("LEFT_STEPPER_1 failed to open");
        return -1;
    }

    if (leftStepper2Para.sm == -1)
    {
        perror("LEFT_STEPPER_2 failed to open");
        return -1;
    }

    if (rightStepper1Para.sm == -1)
    {
        perror("RIGHT_STEPPER_1n failed to open");
        return -1;
    }

    if (rightStepper2Para.sm == -1)
    {
        perror("RIGHT_STEPPER_2 failed to open");
        return -1;
    }

    p_l_st1 = std::make_unique<StepperMotor>(leftStepper1Para);
    p_l_st2 = std::make_unique<StepperMotor>(leftStepper2Para);
    p_r_st1 = std::make_unique<StepperMotor>(rightStepper1Para);
    p_r_st2 = std::make_unique<StepperMotor>(rightStepper2Para);
    
}

bool Devices::initJrkController()
{
   p_JrkX = std::make_unique<JrkController>(JRK1_SERIAL_PORT, 9600);
   p_JrkZ = std::make_unique<JrkController>(JRK2_SERIAL_PORT, 9600);

   bool okX = p_JrkX && p_JrkX->getFd() >= 0;
   bool okZ = p_JrkZ && p_JrkZ->getFd() >= 0;

   if (!okX) { std::cerr << "Failed to open Jrk X\n"; return false; }
   if (!okZ) { std::cerr << "Failed to open Jrk Z\n"; return false; }

   std::cout << "Jrk Controllers opened successfully!\n";
   return true;
}

std::unique_ptr<JrkController> Devices::releaseJrkX()
{
    if (!p_JrkX)
    {
        std::cerr << "Jrk Controller has not been initialized!" << std::endl;
    }
    return std::move(p_JrkX);
}

std::unique_ptr<JrkController> Devices::releaseJrkZ()
{
    if (!p_JrkZ)
    {
        std::cerr << "Jrk Controller has not been initialized!" << std::endl;
    }
    return std::move(p_JrkZ);
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

std::shared_ptr<LJEncoder3Channels> Devices::releaseEncoder()
{
    if (!p_lgEnc3C)
    {
        std::cerr << "Encoder has not been initialized!" << std::endl;
    }
    return std::move(p_lgEnc3C);
}

std::unique_ptr<StepperMotor> Devices::releaseLeftStepper1()
{
    if (!p_l_st1)
    {
        std::cerr << "Left Stepper 1 has not been initialized!" << std::endl;
    }
    return std::move(p_l_st1);
}

std::unique_ptr<StepperMotor> Devices::releaseLeftStepper2()
{
    if (!p_l_st2)
    {
        std::cerr << "Left Stepper 2 has not been initialized!" << std::endl;
    }
    return std::move(p_l_st2);
}

std::unique_ptr<StepperMotor> Devices::releaseRightStepper1()
{
    if (!p_r_st1)
    {
        std::cerr << "Left Stepper 2 has not been initialized!" << std::endl;
    }
    return std::move(p_r_st1);
}

std::unique_ptr<StepperMotor> Devices::releaseRightStepper2()
{
    if (!p_r_st2)
    {
        std::cerr << "Left Stepper 2 has not been initialized!" << std::endl;
    }
    return std::move(p_r_st2);
}


