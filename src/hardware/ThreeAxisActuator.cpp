#include "hardware/ThreeAxisActuator.hpp"

using namespace Eigen;

ThreeAxisActuator::ThreeAxisActuator(Actuator& x,
                                     Actuator& y,
                                     Actuator& z) {
    actuators_[0] = &x;
    actuators_[1] = &y;
    actuators_[2] = &z;
}

void ThreeAxisActuator::applyTorque(const Vector3d& torqueCmd, double deltaT) {
    // Apply torque to each actuator
    for (auto actuator : actuators_) {
        if (actuator && actuator->isOpen()) {
            actuator->applyTorque(torqueCmd, deltaT);
        }
    }
}

Vector3d ThreeAxisActuator::getTorque() const {
    Vector3d totalTorque = Vector3d::Zero();
    
    // Sum torques from all actuators
    for (const auto actuator : actuators_) {
        if (actuator && actuator->isOpen()) {
            totalTorque += actuator->getTorque();
        }
    }
    
    return totalTorque;
}
