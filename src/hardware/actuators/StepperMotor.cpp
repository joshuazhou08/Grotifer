#include "hardware/actuators/StepperMotor.hpp"
#include "hardware/actuators/enums.hpp"
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <cerrno>
#include <string>
#include <termios.h>
#include <unistd.h>

using namespace std;

StepperMotor::StepperMotor(StepperParameters& params, StepperMot stepperMot)
    : path_(params.devicePath),
      baudrate_(params.baudrate),
      stepMode_(params.stepMode),
      name(params.devicePath),
      fd_( open(path_.c_str(), O_RDWR | O_NOCTTY | O_SYNC) )
{

    if (fd_ < 0) {
        cerr << "open(" << path_ << ") failed: " << std::strerror(errno) << endl;
        return;
    }

    cout << "[Stepper] open(" << path_ 
          << ") -> fd=" << fd_ << "\n";


    InitSetting(); // Set up serial port
    Reset(); // Reset the driver

    ResetCmdTimeout(); // Reset cmd timeout

    ClearDriverError(); // Clear all driver errors

    SetCurrentLimit(params.maxCurr); // Set current limit

    SetStepMode(params.stepMode); // Set step mode

    SetMaxSpeed(params.maxSpeed); // Set max speed
 
    SetStartSpeed(params.startSpeed); // Set starting speed

    SetMaxAccel(50'000'000); 
 
    SetMaxDecel(50'000'000);

    Energize(); // Energize the motor
    ExitSafeStart();

    isOpen_ = true;
}

StepperMotor::~StepperMotor()
{
    if (fd_ >= 0) 
    DeEnergize(); // De-energize the motor
    close(fd_); // Close serial port
    cout << "[Stepper Motor] Motor closed" << endl;
}

// Function to set up serial port
void StepperMotor::InitSetting() {
    // Flush away any bytes previously read or written
    tcflush(fd_, TCIOFLUSH);

    // Get the current configuration of the serial port
    struct termios serialConfiguration;
    tcgetattr(fd_, &serialConfiguration);

    // Turn off any options that might interfere with our ability to send and receive raw binary bytes.
    serialConfiguration.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    serialConfiguration.c_oflag &= ~(ONLCR | OCRNL);
    serialConfiguration.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set up timeouts: Calls to read() will return as soon as there is at least one byte available or when 100 ms has passed.
    serialConfiguration.c_cc[VTIME] = 1;
    serialConfiguration.c_cc[VMIN] = 0;
    
    // Set baudrate
    switch(baudrate_)
    {
    case 4800:
        cfsetospeed(&serialConfiguration, B4800);
        break;
    case 9600:
        cfsetospeed(&serialConfiguration, B9600);
        break;
    case 19200:
        cfsetospeed(&serialConfiguration, B19200);
        break;
    case 38400:
        cfsetospeed(&serialConfiguration, B38400);
        break;
    case 115200:
        cfsetospeed(&serialConfiguration, B115200);
        break;
    }

    cfsetispeed(&serialConfiguration, cfgetospeed(&serialConfiguration));
    tcsetattr(fd_, TCSANOW, &serialConfiguration);
}

// Function to write to port
int StepperMotor::write_port(uint8_t *buffer, size_t size) {
    if (fd_ < 0) return -1;

    size_t written = 0;
    while (written < size) {
        ssize_t r = write(fd_, buffer + written, size - written);
        if (r < 0) {
            perror("write failed");
            return -1;
        }
        written += r;
    }
    return 0;
}

// Function to read bytes from port
// Returns after all desired bytes have been read or if timeout/other error
// Returns number of bytes read into the buffer or -1 if error
ssize_t StepperMotor::read_port(uint8_t *buffer, size_t size) {
    if (fd_ < 0) return -1;
    size_t received = 0;
    while (received < size) {
        ssize_t r = read(fd_, buffer + received, size - received);
        if (r < 0) {
            cerr << "Failed to read from Stepper Motor Port" << endl;
            return -1;
        }
        if (r == 0) {
            cerr << "Timed out while reading from Stepper Motor port" << endl;
            break;
        }
        received += r;
    }
    return received;
}

// Function to exit safe start
void StepperMotor::ExitSafeStart() {
    uint8_t command[] = {0x83};
    if (write_port(command, sizeof(command)) < 0)
    {
        cout << "Failed to exit safe start" << endl;
    }
}

// Function to reset the driver
void StepperMotor::Reset() {
    uint8_t command[] = {0xB0};
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to reset driver, error code is: " << error << endl;
    }
}

// Function to reset the command timeout
void StepperMotor::ResetCmdTimeout() {
    uint8_t command[] = {0x8C};
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to reset command timeout, error code is: " << error << endl;
    }
}

// Function to clear all driver errors
void StepperMotor::ClearDriverError() {
    uint8_t command[] = {0x8A};
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to clear driver, error code is: " << error << endl;
    }
}

// Function to set current limit
void StepperMotor::SetCurrentLimit(uint8_t maxCurr) {
    uint8_t command[2];
    command[0] = 0x91;
    command[1] = maxCurr;
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to set current limit, error code is: " << error << endl;
    }
}

// Function to set step mode
void StepperMotor::SetStepMode(uint8_t stepMode) {
    uint8_t command[2];
    command[0] = 0x94;
    command[1] = stepMode;
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to set step mode, error code is: " << error << endl;
    }
}

// Function to set max speed
void StepperMotor::SetMaxSpeed(uint32_t maxSpeed) {
    uint32_t value = maxSpeed;
    uint8_t command[6];
    command[0] = 0xE6;
    command[1] = ((value >> 7) & 1) |
                 ((value >> 14) & 2) |
                 ((value >> 21) & 4) |
                 ((value >> 28) & 8);
    command[2] = value >> 0 & 0x7F;
    command[3] = value >> 8 & 0x7F;
    command[4] = value >> 16 & 0x7F;
    command[5] = value >> 24 & 0x7F;
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to set max speed, error code is: " << error << endl;
    }
}

// Function to set starting speed
void StepperMotor::SetStartSpeed(uint32_t startSpeed) {
    uint32_t value = startSpeed;
    uint8_t command[6];
    command[0] = 0xE5;
    command[1] = ((value >> 7) & 1) |
                 ((value >> 14) & 2) |
                 ((value >> 21) & 4) |
                 ((value >> 28) & 8);
    command[2] = value >> 0 & 0x7F;
    command[3] = value >> 8 & 0x7F;
    command[4] = value >> 16 & 0x7F;
    command[5] = value >> 24 & 0x7F;
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to set start speed, error code is: " << error << endl;
    }
}

// Function to energize the motor
void StepperMotor::Energize() {
    uint8_t command[] = {0x85};
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to energize, error code is: " << error << endl;
    }
}

// Function to de-energize the motor
void StepperMotor::DeEnergize() {
    uint8_t command[] = {0x86};
    if (write_port(command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cerr << "Failed to de-energize, error code is: " << error << endl;
    }
}

// Retrieves errors for GetError...() functions
int StepperMotor::tic_get_variable(uint8_t offset, uint8_t *buffer, uint8_t length) {
    if (fd_ < 0) return -1;

    uint8_t command[] = {0xA2, offset, length};
    int result = write_port(command, sizeof(command));
    if (result != 0)
    {
        return -1;
    }
    ssize_t received = read_port(buffer, length);
    if (received < 0)
    {
        return -1;
    }
    if (received != length)
    {
        fprintf(stderr, "read timeout: expected %u bytes, got %zu\n", length, received);
        return -1;
    }
    return 0;
}

// Retrieves errors from clear/parameter setting functions for Stepper Motors
uint32_t StepperMotor::GetErrorOccuredIs() {
    uint32_t output = 0;
    uint8_t buffer[4];
    if (tic_get_variable(0x04, buffer, sizeof(buffer)) < 0)
    {
        cerr << "Failed to get the error that occured" << endl;
        output = 0;
    }
    else
        output = buffer[0] + ((uint32_t)buffer[1] << 8) + ((uint32_t)buffer[2] << 16) + ((uint32_t)buffer[3] << 24);
    return output;
}

// Function to set target position
void StepperMotor::setTargetPosition(int32_t position)
{
    uint32_t value = position;
    uint8_t command[6];
    command[0] = 0xE0;
    command[1] = ((value >> 7) & 1) |
                 ((value >> 14) & 2) |
                 ((value >> 21) & 4) |
                 ((value >> 28) & 8);
    command[2] = value >> 0 & 0x7F;
    command[3] = value >> 8 & 0x7F;
    command[4] = value >> 16 & 0x7F;
    command[5] = value >> 24 & 0x7F;
    if (write_port(command, sizeof(command)) < 0)
    {
        std::cerr << "write_port failed: " << strerror(errno) << std::endl;
    }
}

// Function to run motors to target position
void StepperMotor::runToPosition(int32_t position) {
    setTargetPosition(position);
}

// Public functions for accessing private class functions
void StepperMotor::energizeMotors() {

    Energize();
}

void StepperMotor::DeEnergizeMotors() {

    DeEnergize();

}


// Logging

// Error status (current, blocking)
uint16_t StepperMotor::GetErrorStatusIs()
{
    uint16_t output = 0;
    uint8_t buffer[2];
    if (tic_get_variable(0x02, buffer, sizeof(buffer)) < 0)
    {
        std::cerr << "[Stepper] Failed to get error status\n";
        output = 0;
    }
    else
    {
        output = buffer[0] + (uint16_t(buffer[1]) << 8);
    }
    return output;
}

// Current position (to see if the Tic is accepting your commands)
int32_t StepperMotor::GetCurrentPositionIs()
{
    int32_t output = 0;
    uint8_t buffer[4];
    if (tic_get_variable(0x22, buffer, sizeof(buffer)) < 0)
    {
        std::cerr << "[Stepper] Failed to get current position\n";
        output = 0;
    }
    else
    {
        uint32_t u =  uint32_t(buffer[0])
                    | (uint32_t(buffer[1]) << 8)
                    | (uint32_t(buffer[2]) << 16)
                    | (uint32_t(buffer[3]) << 24);
        output = int32_t(u);
    }
    return output;
}



void StepperMotor::PrintBasicStatus()
{
    uint8_t op;
    uint8_t misc;
    if (tic_get_variable(0x00, &op, 1) == 0 &&
        tic_get_variable(0x01, &misc, 1) == 0)
    {
        std::cout << "[Stepper] op_state=" << int(op)
                  << " (10 = normal, 2 = de-energized)\n";
        std::cout << "[Stepper] misc_flags=" << int(misc)
                  << " (bit0=energized=" << ((misc & 0x1) ? 1 : 0) << ")\n";
    }
}


void StepperMotor::SetMaxAccel(uint32_t accel)
{
    uint8_t command[6];
    command[0] = 0xE7;
    command[1] = ((accel >> 7) & 1) |
                 ((accel >> 14) & 2) |
                 ((accel >> 21) & 4) |
                 ((accel >> 28) & 8);
    command[2] = accel >> 0  & 0x7F;
    command[3] = accel >> 8  & 0x7F;
    command[4] = accel >> 16 & 0x7F;
    command[5] = accel >> 24 & 0x7F;

    if (write_port(command, sizeof(command)) < 0)
    {
        std::cerr << "Failed to set max acceleration" << std::endl;
    }
}

// Function to set max deceleration
void StepperMotor::SetMaxDecel(uint32_t decel)
{
    uint8_t command[6];
    command[0] = 0xE8;
    command[1] = ((decel >> 7) & 1) |
                 ((decel >> 14) & 2) |
                 ((decel >> 21) & 4) |
                 ((decel >> 28) & 8);
    command[2] = decel >> 0  & 0x7F;
    command[3] = decel >> 8  & 0x7F;
    command[4] = decel >> 16 & 0x7F;
    command[5] = decel >> 24 & 0x7F;

    if (write_port(command, sizeof(command)) < 0)
    {
        std::cerr << "Failed to set max deceleration" << std::endl;
    }
}
