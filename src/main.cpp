#include "BaseTask.hpp"
#include "Tasks.hpp"
#include "Config.hpp"
#include "Devices.hpp"

int main()
{
    /* Initialize all the devices */
    Devices devices;

    if (!devices.initLabJack())
    {
        return -1;
    }

    if (!devices.initInclinometer())
    {
        return -1;
    }

    if (!devices.initSunSensor())
    {
        return -1;
    }

    if (!devices.initMaxonMotors())
    {
        return -1;
    }

    AttitudeControl attitudeControl(
        devices.releaseMMX(),
        devices.releaseMMY(),
        devices.releaseMMZ(),
        devices.releaseSunSensor(),
        devices.releaseInclinometer());
}
