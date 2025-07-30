#include "BaseTask.hpp"
#include "Tasks.hpp"
#include "Config.hpp"
#include "Devices.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <thread>

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
    // Initialize all the devices
    Devices devices;

    if (!devices.initLabJack() || !devices.initInclinometer() ||
        !devices.initSunSensor() || !devices.initMaxonMotors() || 
        !devices.initUSDigiEnc() || !devices.initSteppers() ||
        !devices.initJrkController())
    {
        return -1;
    }

    // Sleep to let us read the device initialize logs
    std::this_thread::sleep_for(std::chrono::seconds(3));

    AttitudeControl attitudeControl(
        devices.releaseMMX(),
        devices.releaseMMY(),
        devices.releaseMMZ(),
        devices.releaseSunSensor(),
        devices.releaseInclinometer());

    // Initialize Task List
    constexpr int NUM_TASKS = 3;

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
            std::cout << "[KILLING] Kill signal detected (E pressed).\n";
            break;
        }
        taskTable[i]->Run();
        i = (i + 1) % NUM_TASKS;
    }
    setNonBlockingInput(false); // Enable non-blocking input for smooth kill signal

    auto JrkX = devices.releaseJrkX();
    auto JrkZ = devices.releaseJrkZ();
    if (!JrkX || !JrkX->isOpen() ||
        !JrkZ || !JrkZ->isOpen())
    {
        std::cerr << "Failed to retrieve the open JrkControllers\n";
        return 1;
    }


    while (true) {
        int targetX;
        int targetZ;
        std::cout << "Enter X and Z motor target speed or -1 to quit: ";
        std::cin >> targetX >> targetZ;

        if(targetX == -1 || targetZ == -1)
        {
            std::cout << "stopping motors, exiting...\n";
            JrkX->setTarget(2048);
            JrkZ->setTarget(2048);
            break;
        }

        std::cout << "setting X motor to " << targetX << "and Z motor to " << targetZ << "\n";
        JrkX->setTarget(targetX);
        JrkZ->setTarget(targetZ);
    }

    return 0;
}