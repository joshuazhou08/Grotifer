#pragma once

#include <fstream>
#include "GrotiferMaster.hpp"
#include "Config.hpp"
#include <memory>

#define SUNSENSOR_SERIAL_PORT "/dev/digital_sun_sensor"


class Devices
{
public:
    Devices();
    ~Devices();

    // functions used in for initialization
    bool initLabJack();
    bool initInclinometer();
    bool initSunSensor();
    bool initMaxonMotors();
    bool initUSDigiEnc();
    bool initSteppers();

    // Release functions to transfer ownership
    std::unique_ptr<MaxonMotor> releaseMMX();
    std::unique_ptr<MaxonMotor> releaseMMY();
    std::unique_ptr<MaxonMotor> releaseMMZ();
    std::unique_ptr<MaxonMotor> releaseMMR();
    std::unique_ptr<MaxonMotor> releaseMML();
    std::unique_ptr<LabJackU6> releaseLabJackU6();
    std::unique_ptr<LabJackInclinometer> releaseInclinometer();
    std::unique_ptr<ModbusSunSensor> releaseSunSensor();
    std::unique_ptr<LJEncoder3Channels> releaseEncoder();
    std::unique_ptr<StepperMotor> releaseLeftStepper1();
    std::unique_ptr<StepperMotor> releaseLeftStepper2();
    std::unique_ptr<StepperMotor> releaseRightStepper1();
    std::unique_ptr<StepperMotor> releaseRightStepper2();

private:
    HANDLE hDevice = nullptr;      // labjack handle
    modbus_t *modbusCtx = nullptr; // modbus context

    // motor parameter structs
    maxon xMotorParams;
    maxon yMotorParams;
    maxon zMotorParams;
    maxon lMotorParams;
    maxon rMotorParams;

    // Maxon motors
    std::unique_ptr<MaxonMotor> p_mmX = nullptr;
    std::unique_ptr<MaxonMotor> p_mmY = nullptr;
    std::unique_ptr<MaxonMotor> p_mmZ = nullptr;
    std::unique_ptr<MaxonMotor> p_mmR = nullptr;
    std::unique_ptr<MaxonMotor> p_mmL = nullptr;

    // Error log
    std::ofstream eposErrFile;

    // Sun sensor
    std::unique_ptr<ModbusSunSensor> p_sunSensor = nullptr;

    // Labjack
    std::unique_ptr<LabJackU6> p_grotiferLJU6 = nullptr;
    std::unique_ptr<LabJackInclinometer> p_inclinometer = nullptr;

    // Encoder
    std::unique_ptr<LJEncoder3Channels> p_lgEnc3C = nullptr;
    std::unique_ptr<LJEncoder3Channels> p_lgEnc3C_l = nullptr;
    std::unique_ptr<LJEncoder3Channels> p_lgEnc3C_r = nullptr;
    
    // Stepper motors
    std::unique_ptr<StepperMotor> p_l_st1 = nullptr;
    std::unique_ptr<StepperMotor> p_l_st2 = nullptr;
    std::unique_ptr<StepperMotor> p_r_st1 = nullptr;
    std::unique_ptr<StepperMotor> p_r_st2 = nullptr;

    // Stepper parameter variables
    stepperPara leftStepper1Para;
    stepperPara leftStepper2Para;
    stepperPara rightStepper1Para;
    stepperPara rightStepper2Para;

};