#include "BaseTask.hpp"
#include "Tasks.hpp"
#include "Config.hpp"
#include "Logger.hpp"
#include "hardware/MaxonMotor.hpp"
#include "hardware/Fan.hpp"
#include "hardware/ThreeAxisActuator.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <thread>

#define X_FAN_SERIAL_PORT "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_21v3_00464559-if01"
#define Z_FAN_SERIAL_PORT "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_21v3_00464472-if01"

// X Momentum Wheel parameter configuration
MaxonParameters makeXMotorParams() {
    MaxonParameters params;
    params.motorName = "xMomMotor";
    params.serialNo = 0x37058243;
    params.K_P = 259266;
    params.K_I = 2450065;
    params.KFF_VEL = 5306;
    params.KFF_ACC = 6859;
    params.K_TUNING = 1;
    params.KP_CURR = 2000035;
    params.KI_CURR = 3362724;
    params.KT = 24.6e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.momentOfInertia = 3.6383e-5; // [kg*m^2]
    params.maxVelocity = 7500;          // [rpm]
    return params;
}

// Y Momentum Wheel parameter configuration
MaxonParameters makeYMotorParams() {
    MaxonParameters params;
    params.motorName = "yMomMotor";
    params.serialNo = 0x37058261;
    params.K_P = 260961;
    params.K_I = 2466086;
    params.KFF_VEL = 4699;
    params.KFF_ACC = 6904;
    params.K_TUNING = 1;
    params.KP_CURR = 1822756;
    params.KI_CURR = 2513998;
    params.KT = 24.6e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.momentOfInertia = 3.6383e-5; // [kg*m^2]
    params.maxVelocity = 7500;          // [rpm]
    return params;
}

// Z Momentum Wheel parameter configuration
MaxonParameters makeZMotorParams() {
    MaxonParameters params;
    params.motorName = "zMomMotor";
    params.serialNo = 0x37059351;
    params.K_P = 266273;
    params.K_I = 2516281;
    params.KFF_VEL = 6600;
    params.KFF_ACC = 7044;
    params.K_TUNING = 1;
    params.KP_CURR = 1789845;
    params.KI_CURR = 2370753;
    params.KT = 24.6e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.momentOfInertia = 3.6383e-5; // [kg*m^2]
    params.maxVelocity = 7500;          // [rpm]
    return params;
}

void setNonBlockingInput(bool enable)
{
    static termios oldt;
    termios newt;
    if (enable)
    {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // disable buffering and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // non-blocking stdin
    }
    else
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore terminal
        fcntl(STDIN_FILENO, F_SETFL, 0);
    }
}

bool wasEKeyPressed()
{
    char ch;
    if (read(STDIN_FILENO, &ch, 1) > 0)
    {
        return ch == 'e' || ch == 'E';
    }
    return false;
}

int main()
{
    // Initialize the logger
    Logger::init("logs/debug.log", Logger::Level::INFO);
    
    // Initialize all the devices (will be removed)
    Devices devices;

    if (!devices.initLabJack() || !devices.initInclinometer() ||
        !devices.initSunSensor() || !devices.initMaxonMotors() || !devices.initFanControllers())
    {
        return -1;
    }

    // Initialize the actuators

    Fan xFan(X_FAN_SERIAL_PORT, 9600, Axis::X);
    if (!xFan.isOpen()) {
        Logger::error("Failed to initialize X Fan (new implementation)");
        return -1;
    }
    Logger::info("X Fan initialized with new Fan class");
    

    MaxonParameters yMotorParams = makeYMotorParams();
    MaxonMotor yMotor(yMotorParams, Axis::Y);
    
    if (!yMotor.isOpen()) {
        Logger::error("Failed to initialize Y Momentum Wheel (new implementation)");
        return -1;
    }
    Logger::info("Y Momentum Wheel initialized with new MaxonMotor class");

    Fan zFan(Z_FAN_SERIAL_PORT, 9600, Axis::Z);
    if (!zFan.isOpen()) {
        Logger::error("Failed to initialize Z Fan (new implementation)");
        return -1;
    }
    Logger::info("Z Fan initialized with new Fan class");
    
    // Create three-axis actuator system
    ThreeAxisActuator threeAxisActuator(xFan, yMotor, zFan);
    Logger::info("ThreeAxisActuator created with X Fan, Y Wheel, Z Fan");

    // Sleep to let us read the device initialize logs
    std::this_thread::sleep_for(std::chrono::seconds(3));

    AttitudeControl attitudeControl(
        threeAxisActuator,
        devices.releaseSunSensor(),
        devices.releaseInclinometer());

    // Initialize Task List
    constexpr int NUM_TASKS = 1;

    BaseTask *taskTable[NUM_TASKS] = {
        &attitudeControl};

    double startTime = GetTimeNow();
    double T_PROGRAM = 60;
    int i = 0;

    setNonBlockingInput(true); // Enable non-blocking input for smooth kill signal
    while (GetTimeNow() - startTime <= T_PROGRAM)
    {
        if (wasEKeyPressed())
        {
            Logger::info("[KILLING] Kill signal detected (E pressed).");
            break;
        }
        taskTable[i]->Run();
        i = (i + 1) % NUM_TASKS;
    }
    setNonBlockingInput(false); // Enable non-blocking input for smooth kill signal
    
    // Cleanup logger
    Logger::cleanup();
}
