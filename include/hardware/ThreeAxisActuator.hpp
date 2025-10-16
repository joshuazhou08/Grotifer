#pragma once
#include "hardware/Actuator.hpp"
#include "hardware/enums.hpp"
#include <Eigen/Dense>
#include <memory>
#include <array>

// Three-axis actuator system - manages X, Y, Z actuators
class ThreeAxisActuator {
public:
    ThreeAxisActuator(Actuator& x,
                      Actuator& y,
                      Actuator& z);

    void applyTorque(const Eigen::Vector3d& torqueCmd, double deltaT);
    Eigen::Vector3d getTorque() const;
    
    // Access individual actuators
    Actuator* getXActuator() { return actuators_[0]; }
    Actuator* getYActuator() { return actuators_[1]; }
    Actuator* getZActuator() { return actuators_[2]; }

private:
    std::array<Actuator*, 3> actuators_;
};