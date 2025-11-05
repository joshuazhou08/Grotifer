#pragma once
#include "hardware/actuators/enums.hpp"
#include <cstdint>
#include <string>
#include <set>



// Stepper configuration parameters
struct StepperParameters
{
    std::string devicePath;    
    uint32_t baudrate;         // Baudrate
    uint8_t stepMode;
    uint32_t maxSpeed;         // Maximum speed, [microstep/10000sec]
    uint32_t startSpeed;       // Starting speed, [microstep/10000sec]
    uint16_t maxCurr;           // Current limit, [mA]
};

class StepperMotor
{
public:

    StepperMotor(StepperParameters& params, StepperMot stepperMot); // Constructor for the class
    
    ~StepperMotor();

    // Actuator interface implementation
    bool isOpen() { return isOpen_; }

    void energizeMotors();

    void DeEnergizeMotors();

    void runToPosition(int32_t position);

protected:

    StepperMot stepperMot_;

private: 

    // Hardware interface
    std::string path_;
    int fd_= -1;
    uint32_t baudrate_ = 0;
    uint8_t stepMode_ = 0;

    // State tracking
    bool isOpen_ = false;

    // ==== Stepper Functions ==== /

    // Static port tracking to prevent multiple motors from opening the same port
    static std::set<std::string> openPorts_;

    // Initial Setting function
    void InitSetting();

    // Function to write bytes to the serial port.
    int write_port(uint8_t *buffer, size_t size);

    // Funtion to reads bytes from the serial port.
    ssize_t read_port(uint8_t *buffer, size_t size);

    // Function to clear all driver error
    void ClearDriverError();

    // Function to reset the driver
    void Reset();

    // Function to reset command timeout
    void ResetCmdTimeout();

    // Function to de-energizethe motor
    void DeEnergize();

    // Function to energize the motor
    void Energize();

    // Function to exit safe start
    void ExitSafeStart();

    // Function to set step mode
    void SetStepMode(uint8_t stepMode);

    // Function to set current limit
    void SetCurrentLimit(uint8_t maxCurr);

    // Function to set max speed
    void SetMaxSpeed(uint32_t maxSpeed);

    // Function to set starting speed
    void SetStartSpeed(uint32_t startSpeed);

    // Retrieves errors for GetErrorOccuredIs();
    int tic_get_variable(uint8_t offset, uint8_t *buffer, uint8_t length);

    // Function to set target position
    void setTargetPosition(int32_t position);

    // Function to run motors to target position
    void runToTargetPosition(int32_t position);

    // Get error returned from other setting/clearing functions
    uint32_t GetErrorOccuredIs();
};