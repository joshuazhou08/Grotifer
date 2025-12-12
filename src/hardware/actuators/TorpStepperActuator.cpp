#include "hardware/actuators/StepperMotor.hpp"
#include "hardware/actuators/TorpStepperActuator.hpp"
#include <iostream>

TorpStepperActuator::TorpStepperActuator(StepperMotor& L1,
                                         StepperMotor& L2,
                                         StepperMotor& R1,
                                         StepperMotor& R2) 
{
    steppers_[0] = &L1;
    steppers_[1] = &L2;
    steppers_[2] = &R1;
    steppers_[3] = &R2;
}

StepperMotor* TorpStepperActuator::get(StepperMot mot) {
    const auto idx = static_cast<size_t>(mot);
    if (idx < steppers_.size() && steppers_[idx]) {
        return steppers_[idx];
    }
    std::cerr << "[TorpStepperActuator] Invalid StepperMot: "
              << static_cast<int>(mot) << std::endl;
    return nullptr;
}

static std::array<StepperMot, 2> stepperPairFor(Side s) {
    
    return (s == Side::L)
        ? std::array<StepperMot,2>{StepperMot::L1, StepperMot::L2}
        : std::array<StepperMot,2>{StepperMot::R1, StepperMot::R2};

}
// Function to Energize motors
void TorpStepperActuator::energizeSide(Side s) {

    auto pair = stepperPairFor(s);
    for (auto m : pair) get(m)->energizeMotors();

}

// Function to DeEnergize motors

void TorpStepperActuator::DeEnergizeSide(Side s) {

    auto pair = stepperPairFor(s);
    for (auto m : pair) get(m)->DeEnergizeMotors();

}

// Function to run motors to set position

void TorpStepperActuator::runToPositionSide(Side s, int32_t position) {

    auto pair = stepperPairFor(s);
    for (auto m : pair) {
        get(m)->runToPosition(position);
    }

}
