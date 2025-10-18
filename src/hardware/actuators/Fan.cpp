#include "hardware/actuators/Fan.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <cerrno>
#include <ctime>

using namespace std;
using namespace Eigen;

Fan::Fan(const char *device, uint32_t baudRate, Axis axis)
    : Actuator(axis),
      fd_(-1),
      lastTarget_(0),
      lastSendTime_(0.0),
      firstSend_(true)
{
    fd_ = openSerialPort(device, baudRate);
    if (fd_ < 0)
    {
        cerr << "[Fan] Failed to open serial port: " << device
             << " (" << strerror(errno) << ")\n";
    }
    else
    {
        cout << "[Fan] Connected to " << device << " @ " << baudRate << " baud\n";
    }
}

Fan::~Fan()
{
    setTarget(2048); // zero out fan
    if (fd_ >= 0)
    {
        close(fd_);
        cout << "[Fan] Port closed\n";
    }
}

void Fan::closePort()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

// Serial interface

int Fan::openSerialPort(const char *device, uint32_t baudRate)
{
    fd_ = open(device, O_RDWR | O_NOCTTY);
    if (fd_ == -1)
    {
        cerr << "[Fan] Error opening " << device
             << ": " << strerror(errno) << endl;
        return -1;
    }

    // flush previously read/written bytes
    int result = tcflush(fd_, TCIOFLUSH);
    if (result)
    {
        cerr << "[Fan] tcflush failed: " << strerror(errno) << endl;
    }

    // get current serial port config

    struct termios options{};
    if (tcgetattr(fd_, &options))
    {
        cerr << "[Fan] tcgetattr failed: " << strerror(errno) << endl;
        close(fd_);
        return -1;
    }

    // Raw mode
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ICANON | ISIG | IEXTEN);
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    speed_t speed = B9600;
    switch (baudRate)
    {
    case 4800:
        speed = B4800;
        break;
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        cout << "Unsupported baud, using 9600\n";
        speed = B9600;
        break;
    }

    cfsetospeed(&options, speed);
    cfsetispeed(&options, speed);
    if (tcsetattr(fd_, TCSANOW, &options))
    {
        cerr << "[Fan] tcgetattr failed: " << strerror(errno) << endl;
        close(fd_);
        return -1;
    }

    return fd_;
}

int Fan::writePort(uint8_t *buffer, size_t size)
{
    ssize_t result = write(fd_, buffer, size);
    if (result != static_cast<ssize_t>(size))
    {
        std::cerr << "[Fan] write failed: "
                  << std::strerror(errno) << " (" << result
                  << "/" << size << " bytes written)" << std::endl;
        return -1;
    }
    return 0;
}

ssize_t Fan::readPort(uint8_t *buffer, size_t size)
{
    size_t received = 0;
    while (received < size)
    {
        ssize_t r = read(fd_, buffer + received, size - received);
        if (r < 0)
        {
            cerr << "[Fan] Read error: " << strerror(errno) << endl;
            return -1;
        }
        if (r == 0)
            break; // timeout
        received += r;
    }

    if (received != size)
    {
        cerr << "[Fan] Warning: expected " << size
             << " bytes, got " << received << endl;
    }

    return received;
}

int Fan::setTarget(uint16_t target)
{
    target = std::clamp(target, minTarget_, maxTarget_);
    uint8_t command[2] = {
        static_cast<uint8_t>(0xC0 + (target & 0x1F)),
        static_cast<uint8_t>((target >> 5) & 0x7F)};
    return writePort(command, sizeof(command));
}

void Fan::moveFan(uint16_t target)
{
    // Simple rate limiting: 50Hz send cap
    double now = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    bool changed = (target != lastTarget_);
    bool time_ok = (now - lastSendTime_) >= 0.02;

    if (isOpen() && (firstSend_ || (changed && time_ok)))
    {
        if (setTarget(target) == 0)
        {
            lastTarget_ = target;
            lastSendTime_ = now;
            firstSend_ = false;
        }
        else
        {
            cerr << "[Fan] Failed to send target " << target << endl;
        }
    }
}

// Actuator implementation

uint16_t Fan::torqueToFanSpeed(double torque) const
{
    double speed = torqueToSpeed_ * std::abs(torque);
    double deadband = 75.0;
    double target;

    if (torque > 0)
        target = neutral_ + deadband + speed;
    else
        target = neutral_ - deadband - speed;

    return static_cast<uint16_t>(std::clamp(target, (double)minTarget_, (double)maxTarget_));
}

void Fan::applyTorque(const Vector3d &torqueCmd, double deltaT)
{
    // deltaT not needed for fans
    double torque = torqueCmd(static_cast<int>(axis_));
    uint16_t target = torqueToFanSpeed(torque);
    moveFan(target);
}