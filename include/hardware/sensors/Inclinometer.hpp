#pragma once
#include "hardware/sensors/Sensors.hpp"
#include "hardware/sensors/Labjack.hpp"

// Inclinometer sensor using LabJack analog inputs
// Reads X and Y tilt angles from dual-axis inclinometer
class Inclinometer : public Sensor {
public:
    // Constructor - connects to LabJack and sets analog input channels for X and Y axes
    Inclinometer(LabJackU6& lju6, long xChannel, long yChannel);
    ~Inclinometer() override;

    // Sensor interface implementation
    bool isOpen() const override { return lju6_ != nullptr; }
    double getAngleX() override;
    double getAngleY() override;

private:
    LabJackU6* lju6_ = nullptr;
    long xChannel_;     // Analog input channel for X-axis
    long yChannel_;     // Analog input channel for Y-axis
    double xAngle_;     // Cached X-axis angle in degrees
    double yAngle_;     // Cached Y-axis angle in degrees
};

