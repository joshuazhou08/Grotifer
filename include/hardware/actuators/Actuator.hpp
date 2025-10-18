#pragma once
#include <Eigen/Dense>
#include "hardware/actuators/enums.hpp"

// a single actuator (fan, momentum wheel, magnetorquer, etc)
class Actuator {
public:
    explicit Actuator(Axis axis) : axis_(axis) {}
    virtual ~Actuator() = default;

    // Check if the actuator is open and operational
    virtual bool isOpen() const = 0;

    // apply torque command over time period deltaT
    virtual void applyTorque(const Eigen::Vector3d& torqueCmd, double deltaT) = 0;

    // return the actual torque being applied
    virtual Eigen::Vector3d getTorque() const = 0;

    // return the current speed (fan speed, motor velocity, etc.)
    virtual double getSpeed() const = 0;

    Axis getAxis() const { return axis_; }

protected:
    Axis axis_;
};