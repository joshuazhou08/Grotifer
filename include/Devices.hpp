#pragma once

#include <fstream>
#include "GrotiferMaster.hpp"
#include "Config.hpp"

class Devices {
    public:
        Devices();
        ~Devices();

        // Maxon motors
        MaxonMotor XMomentumMotor;
        MaxonMotor YMomentumMotor;
        MaxonMotor ZMomentumMotor;

        // Error log
        std::ofstream eposErrFile;

        // Labjack
        LabJackU6 grotiferLJU6;
        LabJackInclinometer inclinometer;

        // Sun sensor
        ModbusSunSensor sunSensor;

    private:
        HANDLE hDevice = nullptr; // labjack handle
        modbus_t* modbusCtx = nullptr; // modbus context

        // motor parameter structs
        maxon xMotorParams;
        maxon yMotorParams;
        maxon zMotorParams;
        
        // functions used in Devices.cpp for initialization
        bool initLabJack();
        bool initSunSensor();
        bool initMaxonMotors();
        
};