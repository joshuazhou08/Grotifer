#include "hardware/sensors/SunSensor.hpp"
#include <modbus/modbus-rtu.h>
#include <iostream>
#include <cstring>
#include <cerrno>

using namespace std;

// Constructor - opens connection to sun sensor on specified path
SunSensor::SunSensor(const char* pathName, int bitRate)
    : ctx_(nullptr),
      fov_(0),
      addInfoCode_(0)
{
    // Open the connection
    if (openConnection(pathName, bitRate)) {
        // Set RS485 mode
        modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485);

        // Get the field of view of the sun sensor immediately
        fov_ = getFov();

        cout << "[SunSensor] Initialized with FOV = " << fov_ << " degrees" << endl;
    }
}

SunSensor::~SunSensor() {
    if (ctx_ != nullptr) {
        modbus_flush(ctx_);
        modbus_close(ctx_);
        modbus_free(ctx_);
        cout << "[SunSensor] Closed connection" << endl;
    }
}

// Open the sun sensor device and create modbus context
bool SunSensor::openConnection(const char* pathName, int bitRate) {
    const int SUNSENSOR_ID = 1; // Modbus default ID of the sun sensor
    char PARITY = 'N';          // Default parity
    const int DATA_BITS = 8;    // Default number of bits of data
    const int STOP_BIT = 1;     // Default stop bit

    // Create modbus RTU context
    ctx_ = modbus_new_rtu(pathName, bitRate, PARITY, DATA_BITS, STOP_BIT);
    if (ctx_ == NULL) {
        cerr << "[SunSensor] Unable to create the libmodbus context" << endl;
        return false;
    }

    // Connect to the sun sensor via modbus
    if (modbus_connect(ctx_) == -1) {
        cerr << "[SunSensor] Connection failed: " << modbus_strerror(errno) << endl;
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }

    // Set the sun sensor ID
    modbus_set_slave(ctx_, SUNSENSOR_ID);

    cout << "[SunSensor] Successfully opened sun sensor on " << pathName << endl;
    return true;
}

// Get the field of view of the sun sensor in degrees
int SunSensor::getFov() {
    uint16_t readResult = 0;
    modbus_read_registers(ctx_, 2, 1, &readResult);
    int fov = readResult;
    modbus_flush(ctx_);
    return fov;
}

// Get X-axis angle in degrees (with Butterworth filter)
double SunSensor::getAngleX() {
    return getAngleWithFilter('X');
}

// Get Y-axis angle in degrees (with Butterworth filter)
double SunSensor::getAngleY() {
    return getAngleWithFilter('Y');
}

// Get radiation data from the sun sensor in W/m^2
int SunSensor::getRadiation() {
    uint16_t readResult = 0;
    modbus_read_registers(ctx_, 9, 1, &readResult);
    int radiation = readResult;
    modbus_flush(ctx_);
    return radiation;
}

// Get temperature in degrees Celsius
double SunSensor::getTemperatureC() {
    uint16_t readResult = 0;
    modbus_read_registers(ctx_, 10, 1, &readResult);
    int resultTemp = convert2CompToSignedDecimal(readResult);
    double temperatureDegC = static_cast<double>(resultTemp) * 0.1;
    modbus_flush(ctx_);
    return temperatureDegC;
}

// Get additional diagnostic information code
int SunSensor::getAddInfo() {
    uint16_t readResult = 0;
    modbus_read_registers(ctx_, 8, 1, &readResult);
    int addInfoCode = readResult;
    modbus_flush(ctx_);
    return addInfoCode;
}

// Convert additional info code to human-readable message
string SunSensor::getAddMessage(int addInfoCode) {
    switch (addInfoCode) {
        case 0xFF:
            return "Radiation is less than 300 W/m^2";
        case 0x33:
            return "Sun is out of FOV";
        case 0x01:
            return "Sun is to X positive reference";
        case 0x02:
            return "Sun is to X negative reference";
        case 0x10:
            return "Sun is to Y positive reference";
        case 0x20:
            return "Sun is to Y negative reference";
        case 0x11:
            return "Sun is to X positive and Y positive reference";
        case 0x12:
            return "Sun is to X negative and Y positive reference";
        case 0x21:
            return "Sun is to X positive and Y negative reference";
        case 0x22:
            return "Sun is to X negative and Y negative reference";
        case 0x00:
            return "Normal / No Information";
        default:
            return "Unknown status code";
    }
}

// Helper function to convert 2's complement number to signed decimal
int16_t SunSensor::convert2CompToSignedDecimal(uint16_t numb) {
    return ((numb & 0x8000) == 0) ? numb : -(~numb + 1);
}

// Get angle with Butterworth filter for a specific axis
double SunSensor::getAngleWithFilter(char axis) {
    int addr;  // Address for the register of the angle data
    if (axis == 'X') {
        addr = 11;
    } else if (axis == 'Y') {
        addr = 12;
    } else {
        cerr << "[SunSensor] Invalid axis '" << axis << "', using X" << endl;
        addr = 11;
    }

    uint16_t readResult = 0;
    modbus_read_registers(ctx_, addr, 1, &readResult);
    
    // Determine scale factor based on FOV
    double scaleFactor;
    if (fov_ == 60) {
        scaleFactor = 0.01;
    } else if (fov_ == 5 || fov_ == 15 || fov_ == 25) {
        scaleFactor = 0.001;
    } else {
        scaleFactor = 0.01; // Default to 60 degree scale
    }

    int16_t resultTemp = convert2CompToSignedDecimal(readResult);
    double angleWithFilter = static_cast<double>(resultTemp) * scaleFactor;
    modbus_flush(ctx_);
    return angleWithFilter;
}

