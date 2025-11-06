#include "hardware/actuators/MaxonMotor.hpp"
#include "hardware/actuators/TorpMaxonActuator.hpp"

TorpMaxonActuator::TorpMaxonActuator(Actuator& L,
                                     Actuator& R) 
{

    actuators_[0] = &L;
    actuators_[1] = &R;
}

// Pick actuator by side
static MaxonMotor* asMaxon(Actuator* a) {
    auto* maxon_ = dynamic_cast<MaxonMotor*>(a);
    if (!maxon_) throw std::runtime_error("Actuator is not MaxonMotor");
    return maxon_;
}

// Homing parameter getters
double TorpMaxonActuator::getHomingVel(Side s) { return asMaxon(get(s))->getHomingVel(); }
double TorpMaxonActuator::getMaxAcc(Side s) { return asMaxon(get(s))->getMaxAcc(); }
double TorpMaxonActuator::getOffsetPos (Side s) { return asMaxon(get(s))->getOffsetPos(); }
double TorpMaxonActuator::getOffsetPosLim(Side s) { return asMaxon(get(s))->getOffsetPosLim(); }
double TorpMaxonActuator::getStartPosLim (Side s) { return asMaxon(get(s))->getStartPosLim(); }

// Motor control functions
void TorpMaxonActuator::haltMotion(Side s) { return asMaxon(get(s))->haltMotion(); }
bool TorpMaxonActuator::setVelocity(Side s, int velocityRPM) { return asMaxon(get(s))->setVelocity(velocityRPM); }