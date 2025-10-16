#pragma once
#include "hardware/Actuator.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <string>
class Fan : public Actuator {
public:

    Fan(const char *device, uint32_t baudRate = 9600, Axis axis);
    ~Fan();

    void closePort();
    
    // implement the actuator interface
    bool isOpen() const override { return fd_ >= 0; }
    void applyTorque(const Eigen::Vector3d& torqueCmd, double deltaT) override;
    Eigen::Vector3d getTorque() const override { return Eigen::Vector3d::Zero(); } // implement later
    double getSpeed() const override { return static_cast<double>(lastTarget_); } // returns last commanded fan speed


private:
    // ====== Internal serial functions ======
    int openSerialPort(const char* device, uint32_t baudRate);
    int writePort(uint8_t* buffer, size_t size);
    ssize_t readPort(uint8_t* buffer, size_t size);
    int setTarget(uint16_t target);
    void moveFan(uint16_t target);
    int getTarget();
    uint16_t torqueToFanSpeed(double torque) const;

    int fd_;
    int lastTarget_;
    double lastSendTime_;
    bool firstSend_;

    // ====== Configuration ======
    double torqueToSpeed_   = 40000;
    uint16_t minTarget_     = 1448;
    uint16_t maxTarget_     = 2648;
    uint16_t neutral_       = 2048;
};