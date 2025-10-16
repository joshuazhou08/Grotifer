#pragma once
#include "hardware/sensors/Sensors.hpp"
#include <cstdint>
#include <string>

// Forward declaration of modbus types
struct _modbus;
typedef struct _modbus modbus_t;

// Digital Sun Sensor using Modbus RTU communication
// Provides sun angle measurements with additional diagnostic information
class SunSensor : public Sensor {
public:
    // Constructor - opens connection to sun sensor on specified path
    // Automatically reads the field of view on initialization
    SunSensor(const char* pathName, int bitRate);
    ~SunSensor() override;

    // Sensor interface implementation
    bool isOpen() const override { return ctx_ != nullptr; }
    double getAngleX() override;
    double getAngleY() override;

    // Get the field of view of the sun sensor in degrees
    int getFov();

    // Get radiation data from the sun sensor in W/m^2
    int getRadiation();

    // Get temperature in degrees Celsius
    double getTemperatureC();

    // Get additional diagnostic information code
    int getAddInfo();

    // Convert additional info code to human-readable message
    std::string getAddMessage(int addInfoCode);

private:
    // Open the sun sensor device and create modbus context
    bool openConnection(const char* pathName, int bitRate);

    // Helper function to convert 2's complement number to signed decimal
    int16_t convert2CompToSignedDecimal(uint16_t numb);

    // Get angle with Butterworth filter for a specific axis
    double getAngleWithFilter(char axis);

    // Modbus communication context
    modbus_t* ctx_ = nullptr;

    // Field of view of the sun sensor in degrees
    uint16_t fov_;

    // Additional info code about the data from the sensor
    // 0 = normal/no information
    int addInfoCode_ = 0;
};

