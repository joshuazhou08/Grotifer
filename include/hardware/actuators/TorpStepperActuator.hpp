#pragma once
#include "hardware/actuators/StepperMotor.hpp"
#include "hardware/actuators/enums.hpp"
#include <iostream>
#include <array>

using namespace std;

// Class to manage stepper motors for torp arms

class TorpStepperActuator {

public:
    TorpStepperActuator(StepperMotor& L1,
                        StepperMotor& L2,
                        StepperMotor& R1,
                        StepperMotor& R2);


    // Pick motor by position
    StepperMotor* get(StepperMot mot);

    // Stepper motor control functions
    void energizeSide (Side s);
    void DeEnergizeSide (Side s);
    void runToPositionSide (Side s, int32_t position);

    // Access individual actuators

    StepperMotor* getL1Stepper() { return steppers_[0]; }
    StepperMotor* getL2Stepper() { return steppers_[1]; }
    StepperMotor* getR1Stepper() { return steppers_[2]; }
    StepperMotor* getR2Stepper() { return steppers_[3]; }


private:
        std::array<StepperMotor*, 4> steppers_;

        
};