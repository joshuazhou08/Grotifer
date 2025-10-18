#pragma once

// Abstract interface for sensors that provide angular measurements
// All sensors must implement angle reading and status checking
class Sensor {
public:
    virtual ~Sensor() = default;

    // Check if the sensor is open and operational
    virtual bool isOpen() const = 0;

    // Get X-axis angle in degrees
    virtual double getAngleX() = 0;

    // Get Y-axis angle in degrees
    virtual double getAngleY() = 0;
};

