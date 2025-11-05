#pragma once
#include <Eigen/Dense>
#include "hardware/actuators/enums.hpp"
#include <optional>



// a single actuator (fan, momentum wheel, magnetorquer, etc)
class Actuator {
public:

    // Axis-type actuator
    explicit Actuator(Axis axis)
        : kind_(ActuatorParam::Axis), axis_(axis), side_(Side::Unused) {}

    // Side-type actuator
    explicit Actuator(Side side)
        : kind_(ActuatorParam::Side), axis_(Axis::Unused), side_(side) {}

    virtual ~Actuator() = default;

    // Check if the actuator is open and operational
    virtual bool isOpen() const = 0;

    // apply torque command over time period deltaT
    virtual void applyTorque(const Eigen::Vector3d& torqueCmd, double deltaT) = 0;

    // return the actual torque being applied
    virtual Eigen::Vector3d getTorque() const = 0;

    // return the current speed (fan speed, motor velocity, etc.)
    virtual double getSpeed() const = 0;
    virtual double getMotPos() const = 0;

    // Helpers to identify motor function:
    bool isAxisType() const { return kind_ == ActuatorParam::Axis; }
    bool isSideType() const { return kind_ == ActuatorParam::Side; }

    // Use when numberic axis index needed
    int axisIndex() const { return static_cast<int>(axis_); }

protected:

ActuatorParam kind_;
Axis axis_{Axis::Unused};
Side side_{Side::Unused};

};