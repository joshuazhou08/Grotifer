#pragma once
#include "hardware/actuators/Actuator.hpp"
#include "hardware/actuators/MaxonMotor.hpp"
#include "hardware/actuators/enums.hpp"
#include <memory>
#include <array>

// Class to manage L/R Maxon motors for Torp arms
class TorpMaxonActuator {

public:
    TorpMaxonActuator(Actuator& L,
                      Actuator& R);

    // Pick actuator by side
    Actuator* get(Side s) { return (s == Side::L) ? actuators_[0]
                                                  : actuators_[1]; }
    // Maxon class functions
    double getMotPos(Side s) { return get(s)->getMotPos(); }
    double getSpeed(Side s) { return get(s)->getSpeed(); }

    // Homing parameter getters
    double getHomingVel(Side s);
    double getMaxAcc(Side s);
    double getOffsetPos(Side s);
    double getOffsetPosLim(Side s);
    double getStartPosLim(Side s);

    // Motor control functions
    void haltMotion(Side s);
    bool setVelocity(Side s, int velocityRPM);

    // Access individual actuators

    Actuator* getLActuator() { return actuators_[0]; }
    Actuator* getRActuator() { return actuators_[1]; }


private:
        std::array<Actuator*, 2> actuators_;

};