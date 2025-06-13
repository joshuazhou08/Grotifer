#include "BaseTask.hpp"
#include "Tasks.hpp"
#include "Config.hpp"
#include "Devices.hpp"

#define SUNSENSOR_SERIAL_PORT "/dev/digital_sun_sensor"

int main()
{
    Devices devices;

    // Now we can access devices here
    // For example:
    double angleX = devices.sunSensor.GetAngleX();
    // or
    double voltage = devices.grotiferLJU6.GetRawVoltageAtChannel(0);

    
}
